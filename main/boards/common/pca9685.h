#pragma once

#include <cstdint>
#include <driver/i2c_master.h>
#include "i2c_device.h"
#include <esp_log.h>

/**
 * PCA9685 — 16路PWM驱动器（I2C接口）
 * 
 * 驱动舵机、LED、电机速度控制等。
 * 默认地址 0x40，可通过 A0-A5 引脚修改。
 * 
 * 用法：
 *   PCA9685 pca(i2c_bus, 0x40);
 *   pca.SetServoPulse(0, 1500);  // 通道0 输出 1500us 脉冲（SG90 90°）
 *   pca.SetDutyCycle(5, 0.5);    // 通道0 50% 占空比（电机速度）
 */
class PCA9685 : public I2cDevice {
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x40;

    /**
     * @param i2c_bus  I2C 总线句柄
     * @param addr     I2C 地址（默认 0x40，A0-A5 拉高可修改）
     * @param freq_hz  PWM 频率（舵机=50Hz，LED=1000Hz，电机=1000~20000Hz）
     */
    PCA9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr = DEFAULT_ADDR, float freq_hz = 50.0f)
        : I2cDevice(i2c_bus, addr) {
        // 复位
        Reset();
        // 设置 PWM 频率
        SetFrequency(freq_hz);
        ESP_LOGI(TAG, "PCA9685 initialized at 0x%02X, freq=%.1fHz", addr, freq_hz_);
    }

    /** 软件复位 */
    void Reset() {
        WriteReg(0x00, 0x00);  // MODE1: 正常模式
        WriteReg(0x01, 0x04);  // MODE2: 推挽输出
    }

    /** 设置 PWM 频率 */
    void SetFrequency(float freq_hz) {
        freq_hz_ = freq_hz;

        // 进入 sleep 模式才能修改 PRE_SCALE
        uint8_t mode1 = ReadReg(0x00);
        WriteReg(0x00, (mode1 & 0x7F) | 0x10);  // sleep bit

        // prescale = round(osc_clock / (4096 * freq)) - 1
        // PCA9685 内部振荡器 = 25MHz
        float prescale_val = 25000000.0f / (4096.0f * freq_hz) - 1.0f;
        uint8_t prescale = (uint8_t)(prescale_val + 0.5f);
        prescale = prescale < 3 ? 3 : prescale;  // 最小值 3
        WriteReg(0xFE, prescale);

        // 唤醒
        WriteReg(0x00, mode1 & 0x7F);
        vTaskDelay(pdMS_TO_TICKS(1));
        // 重启振荡器
        WriteReg(0x00, mode1 | 0x80);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /**
     * 设置指定通道的 PWM 占空比
     * @param channel  通道号 0~15
     * @param duty     占空比 0.0~1.0
     */
    void SetDutyCycle(uint8_t channel, float duty) {
        if (channel > 15) return;
        duty = duty < 0 ? 0 : (duty > 1.0f ? 1.0f : duty);

        uint16_t on = 0;
        uint16_t off = (uint16_t)(duty * 4095.0f);

        // 完全关闭
        if (duty <= 0.0f) {
            off = 0;
            on = 0;
            SetChannelRaw(channel, 0, 4096);  // LED_FULL_OFF
            return;
        }
        // 完全打开
        if (duty >= 1.0f) {
            SetChannelRaw(channel, 4096, 0);  // LED_FULL_ON
            return;
        }

        SetChannelRaw(channel, on, off);
    }

    /**
     * 设置指定通道的舵机脉冲宽度（微秒）
     * @param channel   通道号 0~15
     * @param pulse_us  脉冲宽度（us），SG90: 500~2500
     */
    void SetServoPulse(uint8_t channel, float pulse_us) {
        if (channel > 15) return;

        // 每个tick的时间 = (1/freq) / 4096 秒 = 1000000 / (freq * 4096) 微秒
        float us_per_tick = 1000000.0f / (freq_hz_ * 4096.0f);
        uint16_t ticks = (uint16_t)(pulse_us / us_per_tick);

        SetChannelRaw(channel, 0, ticks);
    }

    /**
     * 设置舵机角度
     * @param channel  通道号 0~15
     * @param angle    角度 0~180
     */
    void SetServoAngle(uint8_t channel, float angle) {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        // 0°=500us, 90°=1500us, 180°=2500us
        float pulse_us = 500.0f + (angle / 180.0f) * 2000.0f;
        SetServoPulse(channel, pulse_us);
    }

    /** 获取当前 PWM 频率 */
    float freq_hz() const { return freq_hz_; }

private:
    /**
     * 写入通道的 ON/OFF 计数器
     * PCA9685 每个通道 4 个寄存器: LEDn_ON_L, LEDn_ON_H, LEDn_OFF_L, LEDn_OFF_H
     */
    void SetChannelRaw(uint8_t channel, uint16_t on, uint16_t off) {
        uint8_t reg = 0x06 + channel * 4;
        uint8_t data[4];
        data[0] = on & 0xFF;
        data[1] = (on >> 8) & 0x0F;
        data[2] = off & 0xFF;
        data[3] = (off >> 8) & 0x0F;

        // 批量写入 4 字节
        WriteReg(reg, data[0]);
        WriteReg(reg + 1, data[1]);
        WriteReg(reg + 2, data[2]);
        WriteReg(reg + 3, data[3]);
    }

    float freq_hz_ = 50.0f;
    static constexpr const char* TAG = "PCA9685";
};
