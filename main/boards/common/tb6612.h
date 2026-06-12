#pragma once

#include <driver/gpio.h>
#include <esp_log.h>

/**
 * TB6612FNG — 双路直流电机驱动器
 * 
 * 每路电机需要：AIN1/AIN2（方向）+ PWMA（速度）
 * 速度 PWM 由 PCA9685 提供，方向由 GPIO 控制。
 * 
 * 接线示例（电机A）：
 *   ESP32 GPIO2  → AIN1
 *   ESP32 GPIO8  → AIN2
 *   PCA9685 CH5  → PWMA
 *   电机A 线1    → A01
 *   电机A 线2    → A02
 */
class TB6612 {
public:
    struct MotorPins {
        gpio_num_t in1;       // 方向引脚1
        gpio_num_t in2;       // 方向引脚2
        uint8_t pca_channel;  // PCA9685 PWM 通道号
    };

    TB6612(const MotorPins& motor_a, const MotorPins& motor_b)
        : motor_a_(motor_a), motor_b_(motor_b) {
        // 配置方向引脚
        ConfigureGpio(motor_a_.in1);
        ConfigureGpio(motor_a_.in2);
        ConfigureGpio(motor_b_.in1);
        ConfigureGpio(motor_b_.in2);

        // 初始化为停止
        StopA();
        StopB();

        ESP_LOGI(TAG, "TB6612 initialized: A(IN1=%d,IN2=%d,PWM=CH%d) B(IN1=%d,IN2=%d,PWM=CH%d)",
                 motor_a_.in1, motor_a_.in2, motor_a_.pca_channel,
                 motor_b_.in1, motor_b_.in2, motor_b_.pca_channel);
    }

    /**
     * 设置电机A速度和方向
     * @param pca      PCA9685 实例指针
     * @param speed    速度 -1.0~1.0（负=反转，正=正转，0=停止）
     */
    template<typename PcaType>
    void SetMotorA(PcaType* pca, float speed) {
        SetMotor(pca, motor_a_, speed);
    }

    /**
     * 设置电机B速度和方向
     * @param pca      PCA9685 实例指针
     * @param speed    速度 -1.0~1.0
     */
    template<typename PcaType>
    void SetMotorB(PcaType* pca, float speed) {
        SetMotor(pca, motor_b_, speed);
    }

    /** 电机A紧急停止（短路刹车） */
    void StopA() {
        gpio_set_level(motor_a_.in1, 1);
        gpio_set_level(motor_a_.in2, 1);
    }

    /** 电机B紧急停止 */
    void StopB() {
        gpio_set_level(motor_b_.in1, 1);
        gpio_set_level(motor_b_.in2, 1);
    }

    /** 两个电机都停止 */
    void StopAll() {
        StopA();
        StopB();
    }

    uint8_t pca_channel_a() const { return motor_a_.pca_channel; }
    uint8_t pca_channel_b() const { return motor_b_.pca_channel; }

private:
    void ConfigureGpio(gpio_num_t pin) {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = (1ULL << pin);
        cfg.mode = GPIO_MODE_OUTPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
    }

    template<typename PcaType>
    void SetMotor(PcaType* pca, const MotorPins& motor, float speed) {
        if (speed > 1.0f) speed = 1.0f;
        if (speed < -1.0f) speed = -1.0f;

        if (speed > 0.01f) {
            // 正转: IN1=1, IN2=0
            gpio_set_level(motor.in1, 1);
            gpio_set_level(motor.in2, 0);
            pca->SetDutyCycle(motor.pca_channel, speed);
        } else if (speed < -0.01f) {
            // 反转: IN1=0, IN2=1
            gpio_set_level(motor.in1, 0);
            gpio_set_level(motor.in2, 1);
            pca->SetDutyCycle(motor.pca_channel, -speed);
        } else {
            // 停止: IN1=0, IN2=0 (滑行)
            gpio_set_level(motor.in1, 0);
            gpio_set_level(motor.in2, 0);
            pca->SetDutyCycle(motor.pca_channel, 0);
        }
    }

    MotorPins motor_a_;
    MotorPins motor_b_;
    static constexpr const char* TAG = "TB6612";
};
