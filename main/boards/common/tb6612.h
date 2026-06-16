#pragma once

#include <driver/gpio.h>
#include <esp_log.h>
#include <functional>

/**
 * TB6612FNG — 双路直流电机驱动器
 * 
 * 每路电机需要：AIN1/AIN2（方向）+ PWMA（速度）
 * 速度 PWM 由 PCA9685 提供，方向由 GPIO 或 IO 扩展芯片（如 XL9555）控制。
 * 
 * 支持两种引脚控制方式：
 *   1. 直连 GPIO（传统方式）：MotorPins 中 in1/in2 设为有效 GPIO 号
 *   2. IO 扩展回调：MotorPins 中 in1/in2 设为 GPIO_NUM_NC，
 *      通过 set_pin_callback 注册回调函数（如 XL9555 SetOutputState）
 * 
 * 接线示例（电机A，GPIO直连）：
 *   ESP32 GPIO2  → AIN1
 *   ESP32 GPIO8  → AIN2
 *   PCA9685 CH5  → PWMA
 * 
 * 接线示例（电机A，XL9555 扩展）：
 *   XL9555 P00   → AIN1   (callback bit=0)
 *   XL9555 P01   → AIN2   (callback bit=1)
 *   PCA9685 CH9  → PWMA
 */
class TB6612 {
public:
    /**
     * 引脚写回调：void(uint8_t bit, uint8_t level)
     *   bit   — XL9555 的 IO 位号 (0-15)
     *   level — 0=低电平, 1=高电平
     */
    using PinWriteCallback = std::function<void(uint8_t bit, uint8_t level)>;

    struct MotorPins {
        gpio_num_t in1;       // 方向引脚1 (GPIO_NUM_NC=使用回调)
        gpio_num_t in2;       // 方向引脚2 (GPIO_NUM_NC=使用回调)
        uint8_t pca_channel;  // PCA9685 PWM 通道号
        uint8_t xl9555_in1_bit;  // XL9555 IO 位号 (仅 in1=GPIO_NUM_NC 时有效)
        uint8_t xl9555_in2_bit;  // XL9555 IO 位号 (仅 in2=GPIO_NUM_NC 时有效)
    };

    TB6612(const MotorPins& motor_a, const MotorPins& motor_b)
        : motor_a_(motor_a), motor_b_(motor_b), pin_callback_(nullptr) {
        // 配置直连 GPIO 方向引脚（非 NC 的引脚）
        if (motor_a_.in1 != GPIO_NUM_NC) ConfigureGpio(motor_a_.in1);
        if (motor_a_.in2 != GPIO_NUM_NC) ConfigureGpio(motor_a_.in2);
        if (motor_b_.in1 != GPIO_NUM_NC) ConfigureGpio(motor_b_.in1);
        if (motor_b_.in2 != GPIO_NUM_NC) ConfigureGpio(motor_b_.in2);

        // 初始化为停止
        StopA();
        StopB();

        ESP_LOGI(TAG, "TB6612 initialized: A(IN1=%d[%s],IN2=%d[%s],PWM=CH%d) B(IN1=%d[%s],IN2=%d[%s],PWM=CH%d)",
                 motor_a_.in1, motor_a_.in1 == GPIO_NUM_NC ? "XL9555" : "GPIO",
                 motor_a_.in2, motor_a_.in2 == GPIO_NUM_NC ? "XL9555" : "GPIO",
                 motor_a_.pca_channel,
                 motor_b_.in1, motor_b_.in1 == GPIO_NUM_NC ? "XL9555" : "GPIO",
                 motor_b_.in2, motor_b_.in2 == GPIO_NUM_NC ? "XL9555" : "GPIO",
                 motor_b_.pca_channel);
    }

    /** 注册 IO 扩展回调（如 XL9555 SetOutputState 封装） */
    void SetPinCallback(PinWriteCallback callback) {
        pin_callback_ = callback;
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
        WritePin(motor_a_.in1, motor_a_.xl9555_in1_bit, 1);
        WritePin(motor_a_.in2, motor_a_.xl9555_in2_bit, 1);
    }

    /** 电机B紧急停止 */
    void StopB() {
        WritePin(motor_b_.in1, motor_b_.xl9555_in1_bit, 1);
        WritePin(motor_b_.in2, motor_b_.xl9555_in2_bit, 1);
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

    /** 写引脚电平：直连GPIO走 gpio_set_level，NC走回调 */
    void WritePin(gpio_num_t pin, uint8_t xl9555_bit, uint8_t level) {
        if (pin != GPIO_NUM_NC) {
            gpio_set_level(pin, level);
        } else if (pin_callback_) {
            pin_callback_(xl9555_bit, level);
        }
    }

    template<typename PcaType>
    void SetMotor(PcaType* pca, const MotorPins& motor, float speed) {
        if (speed > 1.0f) speed = 1.0f;
        if (speed < -1.0f) speed = -1.0f;

        if (speed > 0.01f) {
            // 正转: IN1=1, IN2=0
            WritePin(motor.in1, motor.xl9555_in1_bit, 1);
            WritePin(motor.in2, motor.xl9555_in2_bit, 0);
            pca->SetDutyCycle(motor.pca_channel, speed);
        } else if (speed < -0.01f) {
            // 反转: IN1=0, IN2=1
            WritePin(motor.in1, motor.xl9555_in1_bit, 0);
            WritePin(motor.in2, motor.xl9555_in2_bit, 1);
            pca->SetDutyCycle(motor.pca_channel, -speed);
        } else {
            // 停止: IN1=0, IN2=0 (滑行)
            WritePin(motor.in1, motor.xl9555_in1_bit, 0);
            WritePin(motor.in2, motor.xl9555_in2_bit, 0);
            pca->SetDutyCycle(motor.pca_channel, 0);
        }
    }

    MotorPins motor_a_;
    MotorPins motor_b_;
    PinWriteCallback pin_callback_;
    static constexpr const char* TAG = "TB6612";
};
