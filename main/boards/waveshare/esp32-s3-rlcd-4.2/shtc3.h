#pragma once

#include <cstdint>
#include <driver/i2c_master.h>

/**
 * SHTC3 温湿度传感器 —— 最小驱动
 *
 * 只用 ESP-IDF 5.x/6.x 的「新 I2C 主机 API」，不依赖任何外部组件，
 * 因此可以整份丢进板级目录，靠 CMake 的 file(GLOB) 自动编译。
 *
 * 硬件位置：I2C 总线 SDA=13 / SCL=14，地址 0x70
 * 与 ES8311(0x18)、ES7210(0x40) 共用同一条总线，地址不冲突。
 */
class Shtc3 {
public:
    explicit Shtc3(i2c_master_bus_handle_t bus, uint8_t address = 0x70);
    ~Shtc3();

    bool IsReady() const { return dev_ != nullptr; }

    /**
     * 读一次温湿度。
     * @param temperature_c     温度，摄氏度
     * @param humidity_percent  相对湿度，百分比
     * @return 成功 true；I2C 失败或 CRC 校验不过返回 false
     */
    bool Read(float& temperature_c, float& humidity_percent);

private:
    bool SendCommand(uint16_t cmd);

    i2c_master_dev_handle_t dev_ = nullptr;
};
