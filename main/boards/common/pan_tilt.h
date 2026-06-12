#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <algorithm>

/**
 * PanTilt — 二自由度云台舵机驱动
 * 
 * 支持两种驱动模式：
 * 1. LEDC 模式：直接用 ESP32-S3 LEDC 硬件 PWM（2个舵机直连GPIO）
 * 2. PCA9685 模式：通过 I2C PCA9685 驱动（支持更多舵机）
 * 
 * 支持：绝对/相对角度控制、平滑移动、自动扫描。
 */
class PanTilt {
public:
    enum class DriverMode {
        LEDC,     // ESP32 LEDC 硬件 PWM
        PCA9685,  // I2C PCA9685 PWM
    };

    /** LEDC 模式配置 */
    struct LedcConfig {
        gpio_num_t pan_gpio;
        gpio_num_t tilt_gpio;
        ledc_channel_t pan_channel;
        ledc_channel_t tilt_channel;
        ledc_timer_t timer;
    };

    /** PCA9685 模式配置（通道号） */
    struct PcaConfig {
        uint8_t pan_channel;    // PCA9685 通道号 (0~15)
        uint8_t tilt_channel;
    };

    struct Config {
        DriverMode mode = DriverMode::LEDC;
        float min_angle = 0.0f;
        float max_angle = 180.0f;
        float center_angle = 90.0f;

        // LEDC 模式参数
        LedcConfig ledc = {
            .pan_gpio = GPIO_NUM_2,
            .tilt_gpio = GPIO_NUM_8,
            .pan_channel = LEDC_CHANNEL_1,
            .tilt_channel = LEDC_CHANNEL_2,
            .timer = LEDC_TIMER_1,
        };

        // PCA9685 模式参数
        PcaConfig pca = {
            .pan_channel = 0,
            .tilt_channel = 1,
        };
    };

    PanTilt(const Config& cfg);
    ~PanTilt();

    // 禁止拷贝
    PanTilt(const PanTilt&) = delete;
    PanTilt& operator=(const PanTilt&) = delete;

    /** 设置水平舵机绝对角度 */
    void SetPanAngle(float angle);

    /** 设置俯仰舵机绝对角度 */
    void SetTiltAngle(float angle);

    /** 水平舵机相对移动（度） */
    void PanBy(float delta_degrees);

    /** 俯仰舵机相对移动（度） */
    void TiltBy(float delta_degrees);

    /** 同时设置两个轴的绝对角度 */
    void MoveTo(float pan, float tilt);

    /** 平滑移动到目标角度 */
    void SmoothMoveTo(float pan, float tilt, float speed_deg_per_sec = 60.0f);

    /** 回到居中位置 */
    void Home();

    /** 获取当前角度 */
    float pan_angle() const { return pan_angle_; }
    float tilt_angle() const { return tilt_angle_; }

    /** 启动自动扫描（找不到人时使用） */
    void StartSweep(float pan_min = 30.0f, float pan_max = 150.0f,
                    float tilt_center = 80.0f, float speed = 30.0f);

    /** 停止自动扫描 */
    void StopSweep();

    /** 是否正在扫描 */
    bool IsSweeping() const { return sweeping_; }

    /**
     * 设置 PCA9685 实例指针（PCA9685 模式必须调用）
     * 模板参数避免头文件循环依赖
     */
    template<typename PcaType>
    void SetPcaDriver(PcaType* pca) {
        pca_driver_ = pca;
        pca_set_servo_angle_fn_ = [](void* ctx, uint8_t ch, float angle) {
            static_cast<PcaType*>(ctx)->SetServoAngle(ch, angle);
        };
        // 设置到居中位置
        SetPanAngle(config_.center_angle);
        SetTiltAngle(config_.center_angle);
    }

private:
    void SetAngleLedc(ledc_channel_t channel, float angle);
    void SetAnglePca9685(uint8_t channel, float angle);
    float ClampAngle(float angle) const;
    static void SweepTaskFunc(void* arg);
    void SweepTask();

    Config config_;
    float pan_angle_;
    float tilt_angle_;
    bool sweeping_ = false;
    TaskHandle_t sweep_task_ = nullptr;

    float sweep_pan_min_ = 30.0f;
    float sweep_pan_max_ = 150.0f;
    float sweep_tilt_center_ = 80.0f;
    float sweep_speed_ = 30.0f;

    // PCA9685 驱动（类型擦除）
    void* pca_driver_ = nullptr;
    void (*pca_set_servo_angle_fn_)(void*, uint8_t, float) = nullptr;

    static constexpr const char* TAG = "PanTilt";

    // SG90 舵机 PWM 参数（50Hz, 14-bit resolution = 16384）
    static constexpr uint32_t PWM_RESOLUTION = 16384;
    static constexpr float PWM_PERIOD_US = 20000.0f;
    static constexpr float SERVO_MIN_PULSE_US = 500.0f;
    static constexpr float SERVO_MAX_PULSE_US = 2500.0f;
};
