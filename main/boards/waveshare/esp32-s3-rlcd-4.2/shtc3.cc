#include "shtc3.h"

#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

constexpr char kTag[] = "Shtc3";

/* ── SHTC3 命令字（16 位，大端发送）────────────────────────────────────
 * 取自 SHTC3 数据手册的 command table。
 * 测量命令有两种模式：
 *   0x7CA2 = 温度在前、正常模式（测量约 12.1ms，精度好）  ← 本驱动使用
 *   0x7866 = 温度在前、低功耗模式（约 0.8ms，噪声大）
 * 语音助手场景不在乎这 12ms，所以选精度更好的正常模式。
 */
constexpr uint16_t kCmdWakeUp        = 0x3517;
constexpr uint16_t kCmdSleep         = 0xB098;
constexpr uint16_t kCmdSoftReset     = 0x805D;
constexpr uint16_t kCmdMeasureTFirst = 0x7CA2;

constexpr int kMeasureDelayMs = 15;   /* 正常模式典型 12.1ms，留余量 */

/* ── Sensirion CRC-8 ──────────────────────────────────────────────────
 * 多项式 x^8 + x^5 + x^4 + 1（写作 0x31），初值 0xFF。
 * SHTC3 的 6 字节读回数据里，第 3 和第 6 字节分别是前两字节的 CRC。
 */
uint8_t Crc8(const uint8_t* data, int len) {
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

}  // namespace

Shtc3::Shtc3(i2c_master_bus_handle_t bus, uint8_t address) {
    if (bus == nullptr) {
        ESP_LOGE(kTag, "i2c bus is null");
        return;
    }

    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = address;
    cfg.scl_speed_hz = 100000;

    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &dev_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "add device 0x%02X failed: %s", address, esp_err_to_name(err));
        dev_ = nullptr;
        return;
    }

    /* 软复位，确保从干净状态开始（上次上电可能停在测量中途） */
    SendCommand(kCmdSoftReset);
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_LOGI(kTag, "SHTC3 ready on i2c addr 0x%02X", address);
}

Shtc3::~Shtc3() {
    if (dev_ != nullptr) {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
}

bool Shtc3::SendCommand(uint16_t cmd) {
    if (dev_ == nullptr) return false;
    uint8_t buf[2] = {
        static_cast<uint8_t>(cmd >> 8),
        static_cast<uint8_t>(cmd & 0xFF),
    };
    esp_err_t err = i2c_master_transmit(dev_, buf, sizeof(buf), 100);
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "cmd 0x%04X failed: %s", cmd, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool Shtc3::Read(float& temperature_c, float& humidity_percent) {
    if (dev_ == nullptr) return false;

    /* 1) 唤醒（传感器测完会自动睡，每次读之前都要唤醒） */
    if (!SendCommand(kCmdWakeUp)) return false;
    vTaskDelay(pdMS_TO_TICKS(1));            /* 唤醒时间最大 240us，1ms 足够 */

    /* 2) 触发测量 */
    if (!SendCommand(kCmdMeasureTFirst)) return false;
    vTaskDelay(pdMS_TO_TICKS(kMeasureDelayMs));

    /* 3) 读 6 字节：T_MSB T_LSB T_CRC RH_MSB RH_LSB RH_CRC */
    uint8_t d[6] = {0};
    esp_err_t err = i2c_master_receive(dev_, d, sizeof(d), 200);
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "read failed: %s", esp_err_to_name(err));
        return false;
    }

    if (Crc8(d, 2) != d[2] || Crc8(d + 3, 2) != d[5]) {
        ESP_LOGW(kTag, "crc mismatch: %02X%02X/%02X  %02X%02X/%02X",
                 d[0], d[1], d[2], d[3], d[4], d[5]);
        return false;
    }

    const uint16_t raw_t = static_cast<uint16_t>((d[0] << 8) | d[1]);
    const uint16_t raw_h = static_cast<uint16_t>((d[3] << 8) | d[4]);

    /* 数据手册给出的换算公式 */
    temperature_c    = -45.0f + 175.0f * static_cast<float>(raw_t) / 65536.0f;
    humidity_percent = 100.0f * static_cast<float>(raw_h) / 65536.0f;

    /* 4) 读完立刻睡，省电 */
    SendCommand(kCmdSleep);
    return true;
}
