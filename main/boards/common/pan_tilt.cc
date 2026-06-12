#include "pan_tilt.h"

PanTilt::PanTilt(const Config& cfg)
    : config_(cfg)
    , pan_angle_(cfg.center_angle)
    , tilt_angle_(cfg.center_angle) {

    // 配置 LEDC 定时器
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution = LEDC_TIMER_14_BIT;
    timer_cfg.timer_num = config_.timer;
    timer_cfg.freq_hz = 50;  // SG90: 50Hz
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // 配置 Pan 通道
    ledc_channel_config_t pan_cfg = {};
    pan_cfg.gpio_num = config_.pan_gpio;
    pan_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    pan_cfg.channel = config_.pan_channel;
    pan_cfg.intr_type = LEDC_INTR_DISABLE;
    pan_cfg.timer_sel = config_.timer;
    pan_cfg.duty = 0;
    pan_cfg.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&pan_cfg));

    // 配置 Tilt 通道
    ledc_channel_config_t tilt_cfg = {};
    tilt_cfg.gpio_num = config_.tilt_gpio;
    tilt_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    tilt_cfg.channel = config_.tilt_channel;
    tilt_cfg.intr_type = LEDC_INTR_DISABLE;
    tilt_cfg.timer_sel = config_.timer;
    tilt_cfg.duty = 0;
    tilt_cfg.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&tilt_cfg));

    // 初始化到居中位置
    SetPanAngle(config_.center_angle);
    SetTiltAngle(config_.center_angle);

    ESP_LOGI(TAG, "PanTilt initialized: pan=GPIO%d ch%d, tilt=GPIO%d ch%d, timer=%d",
             config_.pan_gpio, config_.pan_channel,
             config_.tilt_gpio, config_.tilt_channel, config_.timer);
}

PanTilt::~PanTilt() {
    StopSweep();
}

float PanTilt::ClampAngle(float angle) const {
    return std::clamp(angle, config_.min_angle, config_.max_angle);
}

void PanTilt::SetAngle(ledc_channel_t channel, float angle) {
    angle = ClampAngle(angle);

    // 将角度转换为 PWM 占空比
    float pulse_us = SERVO_MIN_PULSE_US +
                     (angle / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    uint32_t duty = (uint32_t)(pulse_us / PWM_PERIOD_US * PWM_RESOLUTION);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void PanTilt::SetPanAngle(float angle) {
    pan_angle_ = ClampAngle(angle);
    SetAngle(config_.pan_channel, pan_angle_);
    ESP_LOGD(TAG, "Pan: %.1f°", pan_angle_);
}

void PanTilt::SetTiltAngle(float angle) {
    tilt_angle_ = ClampAngle(angle);
    SetAngle(config_.tilt_channel, tilt_angle_);
    ESP_LOGD(TAG, "Tilt: %.1f°", tilt_angle_);
}

void PanTilt::PanBy(float delta) {
    SetPanAngle(pan_angle_ + delta);
}

void PanTilt::TiltBy(float delta) {
    SetTiltAngle(tilt_angle_ + delta);
}

void PanTilt::MoveTo(float pan, float tilt) {
    SetPanAngle(pan);
    SetTiltAngle(tilt);
}

void PanTilt::SmoothMoveTo(float pan, float tilt, float speed_deg_per_sec) {
    float pan_diff = pan - pan_angle_;
    float tilt_diff = tilt - tilt_angle_;
    float max_diff = std::max(std::abs(pan_diff), std::abs(tilt_diff));

    if (max_diff < 0.5f) {
        // 已到位，直接设置
        MoveTo(pan, tilt);
        return;
    }

    // 按速度计算总步数，约 60fps
    int steps = std::max(1, (int)(max_diff / speed_deg_per_sec * 60));
    float pan_step = pan_diff / steps;
    float tilt_step = tilt_diff / steps;

    for (int i = 1; i <= steps; i++) {
        SetPanAngle(pan_angle_ + pan_step);
        SetTiltAngle(tilt_angle_ + tilt_step);
        vTaskDelay(pdMS_TO_TICKS(16));  // ~60fps
    }
}

void PanTilt::Home() {
    SmoothMoveTo(config_.center_angle, config_.center_angle);
}

void PanTilt::StartSweep(float pan_min, float pan_max, float tilt_center, float speed) {
    if (sweeping_) return;

    sweep_pan_min_ = pan_min;
    sweep_pan_max_ = pan_max;
    sweep_tilt_center_ = tilt_center;
    sweep_speed_ = speed;
    sweeping_ = true;

    xTaskCreate(SweepTaskFunc, "sweep", 2048, this, 5, &sweep_task_);
    ESP_LOGI(TAG, "Sweep started: pan [%.0f~%.0f], tilt=%.0f, speed=%.0f°/s",
             pan_min, pan_max, tilt_center, speed);
}

void PanTilt::StopSweep() {
    if (!sweeping_) return;
    sweeping_ = false;
    if (sweep_task_) {
        // 等待扫描任务退出
        vTaskDelay(pdMS_TO_TICKS(100));
        sweep_task_ = nullptr;
    }
    ESP_LOGI(TAG, "Sweep stopped");
}

void PanTilt::SweepTaskFunc(void* arg) {
    auto self = static_cast<PanTilt*>(arg);
    self->SweepTask();
}

void PanTilt::SweepTask() {
    // 先转到扫描起始位置
    SetTiltAngle(sweep_tilt_center_);

    float pos = pan_angle_;
    int direction = 1;

    while (sweeping_) {
        // 每步移动的角度 = 速度 / fps
        float step = sweep_speed_ / 30.0f;  // 30fps
        pos += direction * step;

        if (pos >= sweep_pan_max_) {
            pos = sweep_pan_max_;
            direction = -1;
        }
        if (pos <= sweep_pan_min_) {
            pos = sweep_pan_min_;
            direction = 1;
        }

        SetPanAngle(pos);
        vTaskDelay(pdMS_TO_TICKS(33));  // ~30fps
    }

    vTaskDelete(NULL);
}
