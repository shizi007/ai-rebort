/*
 * 正点原子 ATK-DNESP32S3  瓦利机器人版
 * 
 * 基于基础版 atk_dnesp32s3 机器人：
 *   - PCA9685 I2C 16路PWM驱动板（5个舵机 + 2路电机PWM + 2路眼睛灯PWM）
 *   - TB6612 双路直流电机驱动（2个N20减速电机，履带行走）
 *   - PanTilt 云台（头部旋转 + 颈部俯仰，PCA9685 模式）
 *   - 双臂舵机控制（左臂 + 右臂）
 *   - 双眼 WS2812 RGB 灯珠（GPIO44 RMT驱动，全彩1600万色，呼吸/闪烁/颜色切换）
 *   - VL53L0X 激光测距（I2C，精确距离控制，替代视觉估算）
 *   - PersonTracker 人物追踪器（拍照→AI分析→云台跟踪+底盘跟随+激光测距）
 *   - 外放喇叭（ES8388 OUT2→NS4150功放→喇叭）
 *   - 统一供电（5V降压模块同时给 ESP32-S3 和电机供电）
 *   - 16 个 MCP 工具，支持语音控制
 * 
 * 硬件：ATK-DNESP32S3 + OV2640 + PCA9685 + TB6612 + VL53L0X + NS4150 + SG90×5 + N20×2 + 喇叭 + WS2812×2
 * 
 * 舵机接线（PCA9685）：
 *   CH0 — 头部水平旋转 (Pan)
 *   CH1 — 颈部俯仰 (Tilt)
 *   CH2 — 左臂
 *   CH3 — 右臂
 *   CH4 — 备用
 * 
 * 电机接线（TB6612）：
 *   AIN1=GPIO2, AIN2=GPIO8, PWMA=PCA9685_CH5（左电机）
 *   BIN1=GPIO19, BIN2=GPIO20, PWMB=PCA9685_CH6（右电机）
 * 
 * 眼睛灯接线（WS2812）：
 *   GPIO44 → WS2812 DIN → 左眼 → 右眼（级联2颗）
 *   3.3V 供电，无需电平转换
 * 
 * 喇叭接线：
 *   ES8388 OUT2 → NS4150 功放 → 喇叭（8Ω 1W）
 * 
 * 供电方案：
 *   18650×2 (7.4V) → LM2596 5V/3A → ESP32-S3 + PCA9685 + TB6612 + NS4150
 */

#include "wifi_board.h"
#include "codecs/es8388_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "led/single_led.h"
#include "esp_video.h"
#include "pan_tilt.h"
#include "pca9685.h"
#include "tb6612.h"
#include "mcp_server.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <cmath>

#define TAG "atk_dnesp32s3"

// ============ XL9555 IO 扩展芯片（同基础版） ============
class XL9555 : public I2cDevice {
public:
    XL9555(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x06, 0x03);
        WriteReg(0x07, 0xF0);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint16_t data;
        int index = bit;

        if (bit < 8) {
            data = ReadReg(0x02);
        } else {
            data = ReadReg(0x03);
            index -= 8;
        }

        data = (data & ~(1 << index)) | (level << index);

        if (bit < 8) {
            WriteReg(0x02, data);
        } else {
            WriteReg(0x03, data);
        }
    }
};

// ============ WALL-E 眼睛灯控制器（WS2812 RGB） ============
class WalleEyes {
public:
    enum EyeMode {
        kOff = 0, kOn, kBreathe, kBlink, kAngry, kSleepy,
    };
    struct Color {
        uint8_t r, g, b;
        Color() : r(0), g(0), b(0) {}
        Color(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
        Color Scale(float factor) const {
            return Color((uint8_t)std::clamp((int)(r*factor),0,255),
                         (uint8_t)std::clamp((int)(g*factor),0,255),
                         (uint8_t)std::clamp((int)(b*factor),0,255));
        }
        static Color WarmYellow() { return Color(EYE_COLOR_DEFAULT_R,EYE_COLOR_DEFAULT_G,EYE_COLOR_DEFAULT_B); }
        static Color Red()        { return Color(255,0,0); }
        static Color Blue()       { return Color(50,100,255); }
        static Color Green()      { return Color(0,255,100); }
        static Color White()      { return Color(255,255,255); }
        static Color Black()      { return Color(0,0,0); }
    };
    WalleEyes(gpio_num_t gpio) { SetBrightness(EYE_BRIGHTNESS_DEFAULT); color_ = Color::WarmYellow(); InitRmt(gpio); }
    ~WalleEyes() {
        StopAnimation();
        if (encoder_) rmt_del_encoder(encoder_);
        if (rmt_chan_) rmt_del_channel(rmt_chan_);
    }
    void SetMode(EyeMode mode) {
        mode_ = mode; StopAnimation();
        switch (mode) {
            case kOff: UpdateLeds(Color::Black(),Color::Black()); ESP_LOGI(TAG,"Eyes: OFF"); break;
            case kOn:  UpdateLeds(color_.Scale(brightness_),color_.Scale(brightness_)); ESP_LOGI(TAG,"Eyes: ON"); break;
            case kBreathe: case kSleepy: StartAnimation(mode==kSleepy?4000:2000); break;
            case kBlink: StartAnimation(800); break;
            case kAngry: color_=Color::Red(); StartAnimation(200); break;
        }
    }
    void SetBrightness(float level) {
        brightness_=std::clamp(level,0.0f,EYE_BRIGHTNESS_MAX);
        if(mode_==kOn) UpdateLeds(color_.Scale(brightness_),color_.Scale(brightness_));
    }
    void SetColor(uint8_t r,uint8_t g,uint8_t b) { color_=Color(r,g,b); if(mode_==kOn) UpdateLeds(color_.Scale(brightness_),color_.Scale(brightness_)); }
    void SetColor(Color c) { color_=c; if(mode_==kOn) UpdateLeds(color_.Scale(brightness_),color_.Scale(brightness_)); }
    // Wrapper methods for walle_debug_server.cc compatibility
    void turnOff() { SetMode(kOff); }
    void setColor(uint8_t r,uint8_t g,uint8_t b) { SetColor(r,g,b); }
    void setBreath(uint8_t r,uint8_t g,uint8_t b) { color_=Color(r,g,b); SetMode(kBreathe); }
    void setBlink(uint8_t r,uint8_t g,uint8_t b) { color_=Color(r,g,b); SetMode(kBlink); }
    void setRainbow() { SetColor(255,0,255); SetMode(kBreathe); }
    void BlinkOnce() {
        Color saved=color_; UpdateLeds(Color::Black(),Color::Black());
        vTaskDelay(pdMS_TO_TICKS(100)); UpdateLeds(saved.Scale(brightness_),saved.Scale(brightness_));
    }
    void BlinkOnceAsync() { xTaskCreate([](void* a){static_cast<WalleEyes*>(a)->BlinkOnce();vTaskDelete(NULL);},"blink",1024,this,3,nullptr); }
    EyeMode mode() const { return mode_; }
    float brightness() const { return brightness_; }
    Color color() const { return color_; }
private:
    void InitRmt(gpio_num_t gpio) {
        rmt_tx_channel_config_t tx_cfg={}; tx_cfg.gpio_num=gpio; tx_cfg.clk_src=RMT_CLK_SRC_DEFAULT;
        tx_cfg.resolution_hz=10000000; tx_cfg.mem_block_symbols=64; tx_cfg.trans_queue_depth=4;
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg,&rmt_chan_)); ESP_ERROR_CHECK(rmt_enable(rmt_chan_));
        rmt_copy_encoder_config_t enc_cfg={}; ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg,&encoder_));
        ESP_LOGI(TAG,"WS2812 RMT initialized on GPIO%d",gpio);
    }
    void Transmit(const uint8_t* grb_data, size_t len) {
        rmt_symbol_word_t symbols[256]; size_t sc=0;
        for(size_t i=0;i<len;i++){uint8_t byte=grb_data[i];
            for(int b=7;b>=0;b--){bool o=byte&(1<<b);
                symbols[sc].level0=1; symbols[sc].duration0=o?9:3; symbols[sc].level1=0; symbols[sc].duration1=o?3:9; sc++;}}
        symbols[sc].level0=0; symbols[sc].duration0=500; symbols[sc].level1=0; symbols[sc].duration1=0; sc++;
        rmt_transmit_config_t tx_cfg={}; tx_cfg.loop_count=0; tx_cfg.flags.eot_level=0;
        rmt_transmit(rmt_chan_,encoder_,symbols,sc*sizeof(rmt_symbol_word_t),&tx_cfg);
        rmt_tx_wait_all_done(rmt_chan_,100);
    }
    void UpdateLeds(Color l, Color r) {
        uint8_t buf[6]={l.g,l.r,l.b,r.g,r.r,r.b}; Transmit(buf,6); left_color_=l; right_color_=r;
    }
    void StartAnimation(int ms) { if(animating_)return; animating_=true; anim_period_ms_=ms; xTaskCreate(AnimTaskFunc,"eye_anim",2048,this,3,&anim_task_); }
    void StopAnimation() { animating_=false; if(anim_task_){vTaskDelay(pdMS_TO_TICKS(100));anim_task_=nullptr;} }
    static void AnimTaskFunc(void* a){static_cast<WalleEyes*>(a)->AnimTask();}
    void AnimTask() {
        int step=0;
        while(animating_){
            float t=(float)step/40.0f;
            switch(mode_){
                case kBreathe:{float v=brightness_*(0.1f+0.9f*(0.5f+0.5f*sinf(t*6.28318f))); UpdateLeds(color_.Scale(v),color_.Scale(v)); break;}
                case kSleepy: {float v=brightness_*(0.3f+0.7f*(0.5f+0.5f*sinf(t*6.28318f))); UpdateLeds(color_.Scale(v),color_.Scale(v)); break;}
                case kBlink:  {bool on=(step%20)<12; UpdateLeds(on?color_.Scale(brightness_):Color::Black(),on?color_.Scale(brightness_):Color::Black()); break;}
                case kAngry:  {bool on=(step%6)<4; Color ac=on?Color::Red().Scale(brightness_):Color::Red().Scale(0.05f); UpdateLeds(ac,ac); break;}
                default: animating_=false; break;
            }
            step=(step+1)%40; vTaskDelay(pdMS_TO_TICKS(anim_period_ms_/40));
        }
        anim_task_=nullptr; vTaskDelete(NULL);
    }
    rmt_channel_handle_t rmt_chan_=nullptr;
    rmt_encoder_handle_t encoder_=nullptr;
    EyeMode mode_=kOn;
    float brightness_=EYE_BRIGHTNESS_DEFAULT;
    Color color_;
    Color left_color_;
    Color right_color_;
    bool animating_=false;
    int anim_period_ms_=2000;
    TaskHandle_t anim_task_=nullptr;
};

// ============ WALL-E 表情控制器 ============
class WalleExpression {
public:
    enum Expression {
        kNeutral = 0,   // 中性
        kHappy,          // 开心
        kSad,            // 难过
        kCurious,        // 好奇
        kScared,         // 害怕
        kWave,           // 挥手
    };

    WalleExpression(PCA9685* pca, WalleEyes* eyes)
        : pca_(pca), eyes_(eyes) {
        // 初始化到默认姿态
        SetArmLeft(ARM_LEFT_CENTER);
        SetArmRight(ARM_RIGHT_CENTER);
    }

    void SetExpression(Expression expr) {
        current_expr_ = expr;
        // 眼睛联动（颜色+模式）
        if (eyes_) {
            switch (expr) {
                case kHappy:   eyes_->SetColor(WalleEyes::Color::WarmYellow()); eyes_->SetMode(WalleEyes::kOn); eyes_->SetBrightness(1.0f); break;
                case kSad:     eyes_->SetColor(WalleEyes::Color::Blue()); eyes_->SetMode(WalleEyes::kSleepy); eyes_->SetBrightness(0.4f); break;
                case kCurious: eyes_->SetColor(WalleEyes::Color::Green()); eyes_->BlinkOnceAsync(); eyes_->SetMode(WalleEyes::kOn); eyes_->SetBrightness(0.8f); break;
                case kScared:  eyes_->SetColor(WalleEyes::Color::White()); eyes_->SetMode(WalleEyes::kBlink); eyes_->SetBrightness(1.0f); break;
                case kWave:    eyes_->SetColor(WalleEyes::Color::WarmYellow()); eyes_->SetMode(WalleEyes::kOn); eyes_->SetBrightness(0.8f); break;
                case kNeutral:
                default:       eyes_->SetColor(WalleEyes::Color::WarmYellow()); eyes_->SetMode(WalleEyes::kOn); eyes_->SetBrightness(EYE_BRIGHTNESS_DEFAULT); break;
            }
        }
        // 手臂联动
        switch (expr) {
            case kHappy:
                SetArmLeft(40.0f);    // 左臂举起
                SetArmRight(140.0f);  // 右臂举起
                break;
            case kSad:
                SetArmLeft(80.0f);    // 双臂下垂
                SetArmRight(100.0f);
                break;
            case kCurious:
                SetArmLeft(50.0f);    // 左臂微抬
                SetArmRight(110.0f);  // 右臂自然
                break;
            case kScared:
                SetArmLeft(30.0f);    // 双臂举起
                SetArmRight(150.0f);
                break;
            case kWave:
                StartWave();
                return;
            case kNeutral:
            default:
                SetArmLeft(ARM_LEFT_CENTER);
                SetArmRight(ARM_RIGHT_CENTER);
                break;
        }
        StopWave();
    }

    Expression current_expression() const { return current_expr_; }

    void SetArmLeft(float angle) {
        left_arm_angle_ = std::clamp(angle, ARM_MIN, ARM_MAX);
        pca_->SetServoAngle(SERVO_LEFT_ARM_CH, left_arm_angle_);
        ESP_LOGD(TAG, "Left arm: %.1f°", left_arm_angle_);
    }

    void SetArmRight(float angle) {
        right_arm_angle_ = std::clamp(angle, ARM_MIN, ARM_MAX);
        pca_->SetServoAngle(SERVO_RIGHT_ARM_CH, right_arm_angle_);
        ESP_LOGD(TAG, "Right arm: %.1f°", right_arm_angle_);
    }

    float left_arm_angle() const { return left_arm_angle_; }
    float right_arm_angle() const { return right_arm_angle_; }

    /** 挥手动画 */
    void StartWave() {
        if (waving_) return;
        waving_ = true;
        xTaskCreate(WaveTaskFunc, "wave", 2048, this, 3, &wave_task_);
    }

    void StopWave() {
        waving_ = false;
        if (wave_task_) {
            vTaskDelay(pdMS_TO_TICKS(300));
            wave_task_ = nullptr;
        }
    }

    bool IsWaving() const { return waving_; }

private:
    static void WaveTaskFunc(void* arg) {
        auto self = static_cast<WalleExpression*>(arg);
        self->WaveTask();
    }

    void WaveTask() {
        float base = right_arm_angle_;
        // 挥手3次
        for (int i = 0; i < 3 && waving_; i++) {
            SetArmRight(140.0f);
            vTaskDelay(pdMS_TO_TICKS(200));
            SetArmRight(110.0f);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        SetArmRight(base);
        waving_ = false;
        wave_task_ = nullptr;
        vTaskDelete(NULL);
    }

    PCA9685* pca_;
    WalleEyes* eyes_;
    Expression current_expr_ = kNeutral;
    float left_arm_angle_ = ARM_LEFT_CENTER;
    float right_arm_angle_ = ARM_RIGHT_CENTER;
    bool waving_ = false;
    TaskHandle_t wave_task_ = nullptr;
};


// ============ VL53L0X 激光测距传感器 ============
class VL53L0X {
public:
    VL53L0X(i2c_master_bus_handle_t i2c_bus, uint8_t addr = VL53L0X_I2C_ADDR)
        : i2c_bus_(i2c_bus), addr_(addr), last_distance_mm_(0), initialized_(false) {}

    bool Init() {
        // 检测设备是否在线：读取 VL53L0X 标识寄存器 0x00C0
        // 期望值: 0xEEAA (VL53L0X identification)
        uint8_t id_buf[2] = {0xC0, 0x00};
        uint8_t data[2] = {0};
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr_,
            .scl_speed_hz = 400000,
        };
        i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &i2c_dev_);

        // 读取 WHO_AM_I 寄存器
        esp_err_t ret = i2c_master_transmit_receive(
            i2c_dev_, id_buf, 2, data, 2, 100);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "VL53L0X not detected at 0x%02X", addr_);
            return false;
        }

        uint16_t model_id = (data[0] << 8) | data[1];
        if (model_id != 0xEEAA) {
            ESP_LOGW(TAG, "VL53L0X unexpected model ID: 0x%04X", model_id);
            return false;
        }

        // 初始化序列：设置信号速率、VCSEL 脉冲周期、测量预算
        // 简化初始化 — 写入必要寄存器使传感器进入测距模式
        WriteReg16(0x0080, 0x0001);  // SYSRANGE_START bit0=1 (单次测距)
        WriteReg16(0x0083, 0x0004);  // SYSTEM_INTERRUPT_CONFIG_GPIO = 4 (new sample ready)

        initialized_ = true;
        ESP_LOGI(TAG, "VL53L0X initialized at I2C 0x%02X", addr_);
        return true;
    }

    /** 读取距离（毫米），失败返回 -1 */
    int ReadDistanceMm() {
        if (!initialized_) return -1;

        // 启动单次测距
        WriteReg16(0x0018, 0x0001);

        // 等待测量完成（轮询方式，最大 30ms）
        bool ready = false;
        for (int i = 0; i < 30; i++) {
            uint8_t status = 0;
            ReadReg8(0x0013, &status);  // RESULT_INTERRUPT_STATUS
            if (status & 0x07) { ready = true; break; }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!ready) {
            ESP_LOGD(TAG, "VL53L0X measurement timeout");
            return last_distance_mm_;
        }

        // 读取结果
        uint8_t dist_buf[2] = {0};
        uint8_t reg_buf[2] = {0x00, 0x1E};
        esp_err_t ret = i2c_master_transmit_receive(
            i2c_dev_, reg_buf, 2, dist_buf, 2, 100);
        if (ret != ESP_OK) return last_distance_mm_;

        int distance = (dist_buf[0] << 8) | dist_buf[1];

        // 清除中断标志
        WriteReg16(0x0015, 0x0001);  // SYSTEM_INTERRUPT_CLEAR

        if (distance > VL53L0X_MAX_RANGE_MM || distance < VL53L0X_MIN_RANGE_MM) {
            // 超出有效范围
            return last_distance_mm_;
        }

        last_distance_mm_ = distance;
        return distance;
    }

    /** 带中值滤波的读取（3次采样取中值，抗干扰） */
    int ReadDistanceMmFiltered() {
        int d1 = ReadDistanceMm();
        vTaskDelay(pdMS_TO_TICKS(5));
        int d2 = ReadDistanceMm();
        vTaskDelay(pdMS_TO_TICKS(5));
        int d3 = ReadDistanceMm();
        // 三值取中
        if (d1 > d2) std::swap(d1, d2);
        if (d2 > d3) std::swap(d2, d3);
        if (d1 > d2) std::swap(d1, d2);
        return d2;
    }

    bool IsInitialized() const { return initialized_; }
    int last_distance_mm() const { return last_distance_mm_; }

private:
    void WriteReg16(uint16_t reg, uint16_t val) {
        uint8_t buf[4] = {
            (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
            (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)
        };
        i2c_master_transmit(i2c_dev_, buf, 4, 100);
    }

    void ReadReg8(uint16_t reg, uint8_t* out) {
        uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
        i2c_master_transmit_receive(i2c_dev_, reg_buf, 2, out, 1, 100);
    }

    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t i2c_dev_ = nullptr;
    uint8_t addr_;
    int last_distance_mm_ = 0;
    bool initialized_ = false;
};

// ============ 人物追踪器（WALL-E 增强：云台+底盘跟随+激光测距） ============
class PersonTracker {
public:
    struct TrackResult {
        bool person_found = false;
        float center_x = 0.5f;
        float center_y = 0.5f;
        float confidence = 0.0f;
        int distance_mm = -1;  // VL53L0X 测距结果，-1=不可用
    };

    PersonTracker(PanTilt* pan_tilt, Camera* camera, TB6612* motor, PCA9685* pca, VL53L0X* tof)
        : pan_tilt_(pan_tilt), camera_(camera), motor_(motor), pca_(pca), tof_(tof) {}

    ~PersonTracker() {
        StopTracking();
    }

    /** 单次追踪：拍照 → AI分析 → 测距 → 驱动云台+底盘 */
    bool TrackOnce() {
        if (!camera_->Capture()) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }

        std::string question =
            "Analyze this photo. If you see a person, return JSON: "
            "{\"person_found\":true,\"center_x\":0.5,\"center_y\":0.5,\"confidence\":0.9} "
            "where center_x and center_y are the person's center position in the image "
            "(0.0=left/top, 1.0=right/bottom, 0.5=center). "
            "If no person, return: "
            "{\"person_found\":false,\"center_x\":0.5,\"center_y\":0.5,\"confidence\":0.0} "
            "Return ONLY the JSON, no other text.";

        auto result_str = camera_->Explain(question);
        auto track_result = ParseTrackResult(result_str);

        // 读取 VL53L0X 测距（如果可用）
        if (tof_ && tof_->IsInitialized()) {
            track_result.distance_mm = tof_->ReadDistanceMmFiltered();
            ESP_LOGI(TAG, "VL53L0X distance: %d mm", track_result.distance_mm);
        }

        if (track_result.person_found && track_result.confidence > 0.3f) {
            AdjustPanTilt(track_result);
            AdjustChassis(track_result);
            ESP_LOGI(TAG, "Person at (%.2f, %.2f), conf=%.2f, dist=%dmm, pan=%.1f°, tilt=%.1f°",
                     track_result.center_x, track_result.center_y,
                     track_result.confidence, track_result.distance_mm,
                     pan_tilt_->pan_angle(), pan_tilt_->tilt_angle());
        } else {
            ESP_LOGI(TAG, "No person detected");
        }

        last_result_ = track_result;
        return track_result.person_found;
    }

    /** 启动持续追踪 */
    void StartTracking(int interval_ms = 2000) {
        if (tracking_) return;
        tracking_ = true;
        tracking_interval_ms_ = interval_ms;
        no_person_count_ = 0;

        xTaskCreate(TrackingTaskFunc, "tracker", 4096, this, 3, &track_task_);
        ESP_LOGI(TAG, "Person tracking started (interval=%dms)", interval_ms);
    }

    /** 停止持续追踪 */
    void StopTracking() {
        if (!tracking_) return;
        tracking_ = false;
        pan_tilt_->StopSweep();
        // 停止底盘
        motor_->StopAll();
        pca_->SetDutyCycle(MOTOR_LEFT_PWM_CH, 0);
        pca_->SetDutyCycle(MOTOR_RIGHT_PWM_CH, 0);
        if (track_task_) {
            vTaskDelay(pdMS_TO_TICKS(200));
            track_task_ = nullptr;
        }
        ESP_LOGI(TAG, "Person tracking stopped");
    }

    bool IsTracking() const { return tracking_; }
    const TrackResult& last_result() const { return last_result_; }

    void SetDeadzone(float dz) { deadzone_ = dz; }
    void SetPanGain(float gain) { pan_gain_ = gain; }
    void SetTiltGain(float gain) { tilt_gain_ = gain; }
    void SetFollowSpeed(float speed) { follow_speed_ = speed; }

private:
    static void TrackingTaskFunc(void* arg) {
        auto self = static_cast<PersonTracker*>(arg);
        self->TrackingTask();
    }

    void TrackingTask() {
        while (tracking_) {
            bool found = TrackOnce();

            if (found) {
                no_person_count_ = 0;
                if (pan_tilt_->IsSweeping()) {
                    pan_tilt_->StopSweep();
                }
            } else {
                no_person_count_++;
                if (no_person_count_ >= 3 && !pan_tilt_->IsSweeping()) {
                    ESP_LOGI(TAG, "Person lost, starting sweep");
                    pan_tilt_->StartSweep();
                    // 扫描时停止底盘
                    motor_->StopAll();
                    pca_->SetDutyCycle(MOTOR_LEFT_PWM_CH, 0);
                    pca_->SetDutyCycle(MOTOR_RIGHT_PWM_CH, 0);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(tracking_interval_ms_));
        }
        vTaskDelete(NULL);
    }

    TrackResult ParseTrackResult(const std::string& response) {
        TrackResult result;

        auto json_start = response.find('{');
        auto json_end = response.rfind('}');
        if (json_start == std::string::npos || json_end == std::string::npos) {
            ESP_LOGW(TAG, "No JSON found in AI response");
            return result;
        }

        std::string json_str = response.substr(json_start, json_end - json_start + 1);
        cJSON* root = cJSON_Parse(json_str.c_str());
        if (!root) {
            ESP_LOGW(TAG, "JSON parse failed");
            return result;
        }

        auto* pf = cJSON_GetObjectItem(root, "person_found");
        if (pf) result.person_found = cJSON_IsTrue(pf);

        auto* cx = cJSON_GetObjectItem(root, "center_x");
        if (cx && cJSON_IsNumber(cx)) result.center_x = (float)cx->valuedouble;

        auto* cy = cJSON_GetObjectItem(root, "center_y");
        if (cy && cJSON_IsNumber(cy)) result.center_y = (float)cy->valuedouble;

        auto* conf = cJSON_GetObjectItem(root, "confidence");
        if (conf && cJSON_IsNumber(conf)) result.confidence = (float)conf->valuedouble;

        cJSON_Delete(root);
        return result;
    }

    /** 根据人物位置调整云台 */
    void AdjustPanTilt(const TrackResult& result) {
        float dx = result.center_x - 0.5f;
        float dy = result.center_y - 0.5f;

        if (std::abs(dx) < deadzone_) dx = 0;
        if (std::abs(dy) < deadzone_) dy = 0;

        float pan_delta = dx * pan_gain_;
        float tilt_delta = -dy * tilt_gain_;

        pan_tilt_->PanBy(pan_delta);
        pan_tilt_->TiltBy(tilt_delta);
    }

    /** 根据人物位置+激光测距驱动底盘跟随 */
    void AdjustChassis(const TrackResult& result) {
        float dx = result.center_x - 0.5f;  // 水平偏移

        float left_speed = 0;
        float right_speed = 0;

        // ======= 距离控制（优先使用 VL53L0X 激光测距） =======
        float base_speed = follow_speed_;  // 默认速度

        if (result.distance_mm > 0) {
            // 有激光测距数据 — 精确距离控制
            if (result.distance_mm < TRACK_TOO_CLOSE_MM) {
                // 太近！停止或后退
                base_speed = 0.0f;
                ESP_LOGD(TAG, "Too close: %dmm, stopping", result.distance_mm);
            } else if (result.distance_mm < TRACK_FOLLOW_DIST_MM - TRACK_DIST_TOLERANCE_MM) {
                // 偏近，慢速跟随
                base_speed = follow_speed_ * 0.3f;
            } else if (result.distance_mm > TRACK_TOO_FAR_MM) {
                // 太远，加速追
                base_speed = follow_speed_ * 1.3f;
                ESP_LOGD(TAG, "Too far: %dmm, speeding up", result.distance_mm);
            } else if (result.distance_mm > TRACK_FOLLOW_DIST_MM + TRACK_DIST_TOLERANCE_MM) {
                // 偏远，正常速度
                base_speed = follow_speed_;
            } else {
                // 距离合适，低速维持
                base_speed = follow_speed_ * 0.15f;
            }
        } else {
            // 无激光测距 — 回退到视觉估算（兼容无 VL53L0X 的情况）
            float dy = result.center_y - 0.5f;
            if (std::abs(dx) < 0.15f && dy > 0.2f) {
                base_speed = follow_speed_ * 0.3f;  // 人在画面下方≈近
            }
            if (std::abs(dx) < 0.15f && dy < -0.15f) {
                base_speed = follow_speed_ * 1.3f;  // 人在画面上方≈远
            }
        }

        // ======= 差速转向控制 =======
        float turn_factor = dx * 1.5f;  // 转向系数

        left_speed = base_speed - turn_factor * follow_speed_;
        right_speed = base_speed + turn_factor * follow_speed_;

        // 限制范围
        left_speed = std::clamp(left_speed, -1.0f, 1.0f);
        right_speed = std::clamp(right_speed, -1.0f, 1.0f);

        motor_->SetMotorA(pca_, left_speed);
        motor_->SetMotorB(pca_, right_speed);

        ESP_LOGD(TAG, "Chassis: L=%.2f, R=%.2f, dist=%dmm",
                 left_speed, right_speed, result.distance_mm);
    }

    PanTilt* pan_tilt_;
    Camera* camera_;
    TB6612* motor_;
    PCA9685* pca_;
    VL53L0X* tof_;
    bool tracking_ = false;
    int tracking_interval_ms_ = 2000;
    int no_person_count_ = 0;
    float deadzone_ = 0.1f;
    float pan_gain_ = 60.0f;
    float tilt_gain_ = 40.0f;
    float follow_speed_ = 0.4f;
    TaskHandle_t track_task_ = nullptr;
    TrackResult last_result_;
};

// ============ 板型定义 ============
class atk_dnesp32s3 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_;
    XL9555* xl9555_;
    EspVideo* camera_;
    PCA9685* pca9685_;
    TB6612* motor_driver_;
    PanTilt* pan_tilt_;
    WalleExpression* expression_;
    WalleEyes* eyes_;
    VL53L0X* tof_;
    PersonTracker* tracker_;

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize XL9555（I2C 地址 0x20）
        xl9555_ = new XL9555(i2c_bus_, 0x20);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = LCD_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = LCD_SCLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = LCD_CS_PIN;
        io_config.dc_gpio_num = LCD_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 7;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io);

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.data_endian = LCD_RGB_DATA_ENDIAN_BIG;
        esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel);

        esp_lcd_panel_reset(panel);
        xl9555_->SetOutputState(8, 1);   // LCD背光开
        xl9555_->SetOutputState(2, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                    DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                    DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeCamera() {
        xl9555_->SetOutputState(OV_PWDN_IO, 0);
        xl9555_->SetOutputState(OV_RESET_IO, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        xl9555_->SetOutputState(OV_RESET_IO, 1);
        vTaskDelay(pdMS_TO_TICKS(50));

        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAM_PIN_D0,
                [1] = CAM_PIN_D1,
                [2] = CAM_PIN_D2,
                [3] = CAM_PIN_D3,
                [4] = CAM_PIN_D4,
                [5] = CAM_PIN_D5,
                [6] = CAM_PIN_D6,
                [7] = CAM_PIN_D7,
            },
            .vsync_io = CAM_PIN_VSYNC,
            .de_io = CAM_PIN_HREF,
            .pclk_io = CAM_PIN_PCLK,
            .xclk_io = CAM_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = 1,
                .scl_pin = CAM_PIN_SIOC,
                .sda_pin = CAM_PIN_SIOD,
            },
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAM_PIN_RESET,
            .pwdn_pin = CAM_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = 20000000,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new EspVideo(video_config);
    }

    /** 初始化 PCA9685 PWM 驱动板 */
    void InitializePCA9685() {
        // PCA9685 挂在主 I2C 总线上，与 ES8388/XL9555 共享
        // 地址 0x40（默认），50Hz 舵机频率
        pca9685_ = new PCA9685(i2c_bus_, PCA9685_I2C_ADDR, PCA9685_SERVO_FREQ_HZ);

        // 所有通道初始化为0
        for (int i = 0; i < 16; i++) {
            pca9685_->SetDutyCycle(i, 0);
        }

        ESP_LOGI(TAG, "PCA9685 initialized on I2C bus");
    }

    /** 初始化 TB6612 电机驱动 */
    void InitializeMotor() {
        TB6612::MotorPins motor_a = {
            .in1 = TB6612_AIN1_GPIO,
            .in2 = TB6612_AIN2_GPIO,
            .pca_channel = MOTOR_LEFT_PWM_CH,
        };
        TB6612::MotorPins motor_b = {
            .in1 = TB6612_BIN1_GPIO,
            .in2 = TB6612_BIN2_GPIO,
            .pca_channel = MOTOR_RIGHT_PWM_CH,
        };

        motor_driver_ = new TB6612(motor_a, motor_b);
        ESP_LOGI(TAG, "TB6612 motor driver initialized");
    }

    /** 初始化云台（PCA9685 模式） */
    void InitializePanTilt() {
        PanTilt::Config cfg;
        cfg.mode = PanTilt::DriverMode::PCA9685;
        cfg.min_angle = HEAD_PAN_MIN;
        cfg.max_angle = HEAD_PAN_MAX;
        cfg.center_angle = HEAD_PAN_CENTER;
        cfg.pca.pan_channel = SERVO_HEAD_PAN_CH;
        cfg.pca.tilt_channel = SERVO_NECK_TILT_CH;

        pan_tilt_ = new PanTilt(cfg);
        pan_tilt_->SetPcaDriver(pca9685_);
        ESP_LOGI(TAG, "PanTilt(PCA9685) initialized: head=CH%d, neck=CH%d",
                 SERVO_HEAD_PAN_CH, SERVO_NECK_TILT_CH);
    }

    /** 初始化 WALL-E 表情控制 */
    void InitializeExpression() {
        // 先初始化眼睛（Expression 需要联动眼睛）
        InitializeEyes();
        expression_ = new WalleExpression(pca9685_, eyes_);
        ESP_LOGI(TAG, "WALLE-Expression initialized");
    }

    /** 初始化 WALL-E 眼睛灯（WS2812 RGB） */
    void InitializeEyes() {
        eyes_ = new WalleEyes(WS2812_GPIO);
        eyes_->SetMode(WalleEyes::kOn);  // 开机亮眼（暖黄色）
        ESP_LOGI(TAG, "WALLE-Eyes initialized (WS2812 on GPIO%d, %d LEDs)", WS2812_GPIO, WS2812_LED_COUNT);
    }

    /** 初始化 VL53L0X 激光测距传感器 */
    void InitializeToF() {
        tof_ = new VL53L0X(i2c_bus_, VL53L0X_I2C_ADDR);
        bool ok = tof_->Init();
        if (!ok) {
            ESP_LOGW(TAG, "VL53L0X init failed — distance tracking disabled, falling back to visual-only");
        }
    }

    /** 初始化人物追踪器 */
    void InitializeTracker() {
        tracker_ = new PersonTracker(pan_tilt_, camera_, motor_driver_, pca9685_, tof_);
        ESP_LOGI(TAG, "PersonTracker initialized (ToF=%s)", tof_->IsInitialized() ? "YES" : "NO");
    }

    /** 注册 MCP 工具，支持语音控制 */
    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        // ============ 底盘控制（4个） ============

        mcp.AddTool("self.chassis.go_forward",
            "WALL-E前进",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                motor_driver_->SetMotorA(pca9685_, 0.5f);
                motor_driver_->SetMotorB(pca9685_, 0.5f);
                return true;
            });

        mcp.AddTool("self.chassis.go_back",
            "WALL-E后退",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                motor_driver_->SetMotorA(pca9685_, -0.5f);
                motor_driver_->SetMotorB(pca9685_, -0.5f);
                return true;
            });

        mcp.AddTool("self.chassis.turn_left",
            "WALL-E左转",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                motor_driver_->SetMotorA(pca9685_, -0.4f);
                motor_driver_->SetMotorB(pca9685_, 0.4f);
                return true;
            });

        mcp.AddTool("self.chassis.turn_right",
            "WALL-E右转",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                motor_driver_->SetMotorA(pca9685_, 0.4f);
                motor_driver_->SetMotorB(pca9685_, -0.4f);
                return true;
            });

        mcp.AddTool("self.chassis.stop",
            "WALL-E停止移动",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                motor_driver_->StopAll();
                pca9685_->SetDutyCycle(MOTOR_LEFT_PWM_CH, 0);
                pca9685_->SetDutyCycle(MOTOR_RIGHT_PWM_CH, 0);
                return true;
            });

        // ============ 追踪控制（3个） ============

        mcp.AddTool("self.tracker.start",
            "开始追踪人物。WALL-E会跟着人走，找不到人时自动扫描搜索。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                tracker_->StartTracking(2000);
                expression_->SetExpression(WalleExpression::kCurious);
                return true;
            });

        mcp.AddTool("self.tracker.stop",
            "停止追踪人物。云台回到居中位置，底盘停止。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                tracker_->StopTracking();
                pan_tilt_->Home();
                expression_->SetExpression(WalleExpression::kNeutral);
                motor_driver_->StopAll();
                pca9685_->SetDutyCycle(MOTOR_LEFT_PWM_CH, 0);
                pca9685_->SetDutyCycle(MOTOR_RIGHT_PWM_CH, 0);
                return true;
            });

        mcp.AddTool("self.tracker.check",
            "拍照检查当前画面中是否有人，返回人物位置和距离信息。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                bool found = tracker_->TrackOnce();
                auto& r = tracker_->last_result();
                cJSON* json = cJSON_CreateObject();
                cJSON_AddBoolToObject(json, "person_found", found);
                cJSON_AddNumberToObject(json, "center_x", r.center_x);
                cJSON_AddNumberToObject(json, "center_y", r.center_y);
                cJSON_AddNumberToObject(json, "confidence", r.confidence);
                cJSON_AddNumberToObject(json, "distance_mm", r.distance_mm);
                return json;
            });

        // ============ 云台控制（3个） ============

        mcp.AddTool("self.head.look_at",
            "控制WALL-E头部看向指定方向。pan控制左右（0=最左，90=正中，180=最右），tilt控制上下（45=最低，90=正中，135=最高）。",
            PropertyList({
                Property("pan", kPropertyTypeInteger, 90, (int)HEAD_PAN_MIN, (int)HEAD_PAN_MAX),
                Property("tilt", kPropertyTypeInteger, 90, (int)NECK_TILT_MIN, (int)NECK_TILT_MAX)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                float pan = (float)props["pan"].value<int>();
                float tilt = (float)props["tilt"].value<int>();
                pan_tilt_->SmoothMoveTo(pan, tilt);
                return true;
            });

        mcp.AddTool("self.head.sweep",
            "WALL-E头部左右扫描搜索。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                pan_tilt_->StartSweep(HEAD_PAN_MIN, HEAD_PAN_MAX, NECK_TILT_CENTER);
                return true;
            });

        mcp.AddTool("self.head.home",
            "WALL-E头部回到正前方。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                pan_tilt_->Home();
                return true;
            });

        // ============ 表情/手臂控制（2个） ============

        mcp.AddTool("self.expression.set",
            "设置WALL-E的表情。0=中性，1=开心（双臂举起），2=难过（双臂下垂），3=好奇（左臂微抬），4=害怕（双臂高举），5=挥手。",
            PropertyList({
                Property("expression", kPropertyTypeInteger, 0, 0, 5)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                int expr = props["expression"].value<int>();
                expression_->SetExpression(static_cast<WalleExpression::Expression>(expr));
                return true;
            });

        mcp.AddTool("self.expression.wave",
            "WALL-E挥手打招呼。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                expression_->StartWave();
                return true;
            });

        // ============ 眼睛灯控制（4个） ============

        mcp.AddTool("self.eyes.set_mode",
            "设置WALL-E眼睛灯模式。0=关闭，1=常亮，2=呼吸灯，3=闪烁，4=生气(红色快速闪烁)，5=困倦(慢呼吸)。",
            PropertyList({
                Property("mode", kPropertyTypeInteger, 1, 0, 5)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                int mode = props["mode"].value<int>();
                eyes_->SetMode(static_cast<WalleEyes::EyeMode>(mode));
                return true;
            });

        mcp.AddTool("self.eyes.set_brightness",
            "设置WALL-E眼睛亮度。0=最暗，100=最亮。",
            PropertyList({
                Property("brightness", kPropertyTypeInteger, 60, 0, 100)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                int val = props["brightness"].value<int>();
                eyes_->SetBrightness(val / 100.0f);
                return true;
            });

        mcp.AddTool("self.eyes.set_color",
            "设置WALL-E眼睛颜色。RGB格式，每个通道0-255。默认暖黄色(255,200,80)。",
            PropertyList({
                Property("r", kPropertyTypeInteger, 255, 0, 255),
                Property("g", kPropertyTypeInteger, 200, 0, 255),
                Property("b", kPropertyTypeInteger, 80, 0, 255)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                int r = props["r"].value<int>();
                int g = props["g"].value<int>();
                int b = props["b"].value<int>();
                eyes_->SetColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
                return true;
            });

        mcp.AddTool("self.eyes.blink",
            "WALL-E眨眼。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                eyes_->BlinkOnceAsync();
                return true;
            });

        ESP_LOGI(TAG, "MCP tools registered (17 WALL-E tools)");
    }

public:
    atk_dnesp32s3()
        : boot_button_(BOOT_BUTTON_GPIO)
        , display_(nullptr)
        , xl9555_(nullptr)
        , camera_(nullptr)
        , pca9685_(nullptr)
        , motor_driver_(nullptr)
        , pan_tilt_(nullptr)
        , expression_(nullptr)
        , eyes_(nullptr)
        , tof_(nullptr)
        , tracker_(nullptr) {

        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeButtons();
        InitializeCamera();
        InitializePCA9685();
        InitializeMotor();
        InitializePanTilt();
        InitializeExpression();
        // InitializeEyes() 已在 InitializeExpression() 内部调用
        InitializeToF();
        InitializeTracker();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8388AudioCodec audio_codec(
            i2c_bus_,
            I2C_NUM_0,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            GPIO_NUM_NC,
            AUDIO_CODEC_ES8388_ADDR
        );
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    // ============ WALL-E 公共访问接口 ============
    WalleEyes* GetWalleEyes() { return eyes_; }
    PCA9685* GetPca9685() { return pca9685_; }
    PanTilt* GetPanTilt() { return pan_tilt_; }
    TB6612* GetMotorDriver() { return motor_driver_; }
    WalleExpression* GetWalleExpression() { return expression_; }
    VL53L0X* GetVl53l0x() { return tof_; }
    i2c_master_bus_handle_t GetI2cBus() { return i2c_bus_; }
};

// ============ WALL-E C 包装函数（供 walle_debug_server.cc 调用）============
#include "boards/common/board.h"

extern "C" {
    // 眼睛灯控制
    void walle_eyes_turnOff() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* eyes = board->GetWalleEyes();
        if (eyes) eyes->turnOff();
    }
    void walle_eyes_setColor(uint8_t r, uint8_t g, uint8_t b) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* eyes = board->GetWalleEyes();
        if (eyes) eyes->setColor(r, g, b);
    }
    void walle_eyes_setBreath(uint8_t r, uint8_t g, uint8_t b) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* eyes = board->GetWalleEyes();
        if (eyes) eyes->setBreath(r, g, b);
    }
    void walle_eyes_setBlink(uint8_t r, uint8_t g, uint8_t b) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* eyes = board->GetWalleEyes();
        if (eyes) eyes->setBlink(r, g, b);
    }
    void walle_eyes_setRainbow() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* eyes = board->GetWalleEyes();
        if (eyes) eyes->setRainbow();
    }
    
    // 表情控制（直接调用 PCA9685）
    void walle_expression_playHappy() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* pca = board->GetPca9685();
        if (pca) { pca->SetServoAngle(2, 45); pca->SetServoAngle(3, 135); }
    }
    void walle_expression_playSad() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* pca = board->GetPca9685();
        if (pca) { pca->SetServoAngle(2, 135); pca->SetServoAngle(3, 45); }
    }
    void walle_expression_playSurprised() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* pca = board->GetPca9685();
        if (pca) { pca->SetServoAngle(2, 90); pca->SetServoAngle(3, 90); }
    }
    void walle_expression_playAngry() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* pca = board->GetPca9685();
        if (pca) { pca->SetServoAngle(2, 60); pca->SetServoAngle(3, 120); }
    }
    void walle_expression_playSleepy() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* pca = board->GetPca9685();
        if (pca) { pca->SetServoAngle(2, 75); pca->SetServoAngle(3, 105); }
    }
    void walle_expression_playWave() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* pca = board->GetPca9685();
        if (pca) { pca->SetServoAngle(4, 180); vTaskDelay(pdMS_TO_TICKS(500)); pca->SetServoAngle(4, 0); }
    }
    
    // 舵机控制（5个舵机：CH0-CH4）
    void walle_servo_setAngle(int channel, int angle) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* pca = board->GetPca9685();
        if (channel == 0) { auto* pt = board->GetPanTilt(); if (pt) pt->SetPanAngle(angle); }
        else if (channel == 1) { auto* pt = board->GetPanTilt(); if (pt) pt->SetTiltAngle(angle); }
        else if (pca) pca->SetServoAngle(channel, angle);
    }
    
    // 电机控制（速度 -255~255）
    void walle_motor_setSpeed(int left, int right) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* motor = board->GetMotorDriver();
        auto* pca = board->GetPca9685();
        if (motor && pca) {
            motor->SetMotorA(pca, left / 255.0f);
            motor->SetMotorB(pca, right / 255.0f);
        }
    }
    
    // 急停
    void walle_emergency_stop() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* motor = board->GetMotorDriver();
        auto* pca = board->GetPca9685();
        if (motor && pca) { motor->SetMotorA(pca, 0); motor->SetMotorB(pca, 0); }
    }
    
    // 读取距离（毫米）
    uint16_t walle_vl53l0x_readRange() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* tof = board->GetVl53l0x();
        if (tof) return (uint16_t)tof->ReadDistanceMm();
        return 0;
    }
    
    // 系统重启
    void walle_system_restart() {
        esp_restart();
    }
    
    // I²C 总线扫描（通过临时设备探测，返回 JSON 设备地址列表）
    char* walle_i2c_scan_json() {
        static char result[2048];
        result[0] = '\0';
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* bus = board->GetI2cBus();
        if (!bus) { snprintf(result, sizeof(result), "{\"error\":\"no i2c bus\"}"); return result; }
        
        char* p = result;
        p += snprintf(p, sizeof(result) - (p - result), "{\"devices\":[");
        bool first = true;
        for (int addr = 1; addr < 127; addr++) {
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = (uint8_t)addr,
                .scl_speed_hz = 100000,
            };
            i2c_master_dev_handle_t dev = nullptr;
            esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
            if (ret == ESP_OK && dev) {
                uint8_t dummy[1] = {0};
                ret = i2c_master_transmit_receive(dev, dummy, 0, dummy, 0, 10);
                i2c_master_bus_rm_device(dev);
                if (ret == ESP_OK) {
                    if (!first) *p++ = ',';
                    p += snprintf(p, sizeof(result) - (p - result), "\"0x%02X\"", addr);
                    first = false;
                }
            }
        }
        p += snprintf(p, sizeof(result) - (p - result), "]}");
        return result;
    }
}

DECLARE_BOARD(atk_dnesp32s3);

