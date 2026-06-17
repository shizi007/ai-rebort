/*
 * 正点原子 ATK-DNESP32S3  瓦利机器人版 v10 — 纯 P1 排针方案
 *
 * [v10] 架构变更：完全不使用板载走线，所有外设只从 P1 排针引出
 *   - 删除 XL9555 IO 扩展、ES8388 板载音频、板载 ST7789 LCD
 *   - 改用 ST7735S 1.3" 240×240 外接 LCD (P1 排针直连)
 *   - ExternalAudio(PCM5102+INMP441) 替代 ES8388，实现 AudioCodec 接口
 *   - GC9A01 眼屏 CS 改 GPIO 直控(GPIO19/20)，不再走 XL9555
 *   - LCD 背光改 GPIO 直控(GPIO48)，不再走 XL9555
 *
 * 功能:
 *   - PCA9685 I2C 16路PWM驱动板(9个舵机 + 2路电机PWM)
 *   - TB6612 双路直流电机驱动(2个JGB37-520减速电机,12V/200RPM,履带行走)
 *   - PanTilt 云台(脖子左右 + 头部上下 + 脖子伸缩,PCA9685 模式)
 *   - 双臂舵机控制(左臂 + 右臂)
 *   - 双眼 GC9A01 240x240 圆形TFT屏(共享主屏SPI总线,像素级表情渲染)
 *   - ST7735S 1.3" 240×240 主屏(P1 排针 SPI 直连)
 *   - VL53L0X 激光测距(I2C,精确距离控制,替代视觉估算)
 *   - PersonTracker 人物追踪器(拍照→AI分析→云台跟踪+底盘跟随+激光测距)
 *   - 外接喇叭(PCM5102→PAM8406功放→喇叭)
 *   - 外接麦克风(INMP441)
 *   - 17 个 MCP 工具,支持语音控制
 *
 * 硬件:ATK-DNESP32S3 + ESP32-CAM子板(UART2) + PCA9685 + TB6612 + VL53L0X
 *        + PAM8406 + PCM5102 + INMP441 + ST7735S + SG90×9 + JGB37-520×2 + 喇叭 + GC9A01×2
 *
 * P1 排针 GPIO 分配 (v10):
 *   GPIO3  → LCD CS (ST7735S)
 *   GPIO4  → TB6612 AIN1
 *   GPIO5  → TB6612 AIN2
 *   GPIO6  → TB6612 BIN1
 *   GPIO7  → TB6612 BIN2
 *   GPIO9  → SPI MOSI (LCD + 眼屏)
 *   GPIO10 → SPI SCLK (LCD + 眼屏)
 *   GPIO14 → SPI DC (LCD + 眼屏)
 *   GPIO15 → I2S1 BCLK
 *   GPIO16 → UART2 TX
 *   GPIO17 → UART2 RX
 *   GPIO18 → I2S1 WS
 *   GPIO19 → 眼屏 CS_L (GC9A01)
 *   GPIO20 → 眼屏 CS_R (GC9A01)
 *   GPIO38 → I2C1 SCL
 *   GPIO39 → I2C1 SDA
 *   GPIO45 → I2S1 DOUT (→ PCM5102 DIN)
 *   GPIO47 → I2S1 DIN  (← INMP441 SD)
 *   GPIO48 → LCD 背光
 */

#include "wifi_board.h"
// [v10] 已删除：#include "codecs/es8388_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "led/single_led.h"
#include "uart_camera.h"
#include "pan_tilt.h"
#include "pca9685.h"
#include "tb6612.h"
#include "mcp_server.h"
#include "walle_debug_server.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_st7789.h>
#include <esp_lcd_gc9a01.h>
#include <driver/i2c_master.h>
#include <driver/i2s_std.h>
#include <driver/i2s_common.h>
#include <driver/spi_common.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <cmath>
#include <vector>

#define TAG "atk_dnesp32s3"

// [v10] 已删除：XL9555 IO 扩展芯片类 — 不再使用板载走线，所有 CS/BL 改 GPIO 直控

// ============ GC9A01 圆形屏眼睛(SPI 独立总线) ============
// ============ GC9A01 圆形屏眼睛(SPI 独立总线) ============
class Gc9a01Eyes {
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
        static Color WarmYellow() { return Color(255,200,80); }
        static Color Red()        { return Color(255,0,0); }
        static Color Blue()       { return Color(50,100,255); }
        static Color Green()      { return Color(0,255,100); }
        static Color White()      { return Color(255,255,255); }
        static Color Black()      { return Color(0,0,0); }
    };

    Gc9a01Eyes(spi_host_device_t host, gpio_num_t dc, gpio_num_t cs_left, gpio_num_t cs_right)
        : host_(host), dc_(dc), cs_left_(cs_left), cs_right_(cs_right)
    {
        InitPanels();
        AllocFramebuffers();
        if (fb_left_)  ClearFb(fb_left_);
        if (fb_right_) ClearFb(fb_right_);
        color_ = Color::WarmYellow();
        brightness_ = 0.6f;
        ESP_LOGI(TAG, "GC9A01 eyes: SPI%d DC=%d CS_L=%d CS_R=%d (shared LCD bus)",
                 host_+1, dc_, cs_left_, cs_right_);
    }

    ~Gc9a01Eyes() {
        StopAnimation();
        vTaskDelay(pdMS_TO_TICKS(150));
        if (panel_left_)  esp_lcd_panel_del(panel_left_);
        if (panel_right_) esp_lcd_panel_del(panel_right_);
        if (io_left_)     esp_lcd_panel_io_del(io_left_);
        if (io_right_)    esp_lcd_panel_io_del(io_right_);
        // 注意:不释放 SPI 总线,因为与主屏共享
        if (fb_left_)  heap_caps_free(fb_left_);
        if (fb_right_) heap_caps_free(fb_right_);
    }

    void SetMode(EyeMode mode) {
        mode_ = mode; StopAnimation();
        switch (mode) {
            case kOff:
                ClearFb(fb_left_); ClearFb(fb_right_);
                PushBoth();
                ESP_LOGI(TAG, "Eyes: OFF");
                break;
            case kOn:
                RenderBothOpen(color_); PushBoth();
                ESP_LOGI(TAG, "Eyes: ON");
                break;
            case kBreathe:
                StartAnimation(2000);
                break;
            case kBlink:
                StartAnimation(800);
                break;
            case kAngry:
                color_ = Color::Red();
                StartAnimation(200);
                break;
            case kSleepy:
                StartAnimation(4000);
                break;
        }
    }

    void SetBrightness(float level) { brightness_ = std::clamp(level, 0.0f, 1.0f); }

    void SetColor(uint8_t r, uint8_t g, uint8_t b) {
        color_ = Color(r, g, b);
        if (mode_ == kOn) { RenderBothOpen(color_); PushBoth(); }
    }
    void SetColor(Color c) {
        color_ = c;
        if (mode_ == kOn) { RenderBothOpen(color_); PushBoth(); }
    }

    // ---- walle_debug_server 兼容方法 ----
    void turnOff() { SetMode(kOff); }
    void setColor(uint8_t r, uint8_t g, uint8_t b) { SetColor(r, g, b); }
    void setBreath(uint8_t r, uint8_t g, uint8_t b) { color_ = Color(r, g, b); SetMode(kBreathe); }
    void setBlink(uint8_t r, uint8_t g, uint8_t b) { color_ = Color(r, g, b); SetMode(kBlink); }
    void setRainbow() { SetColor(255, 0, 255); SetMode(kBreathe); }

    void BlinkOnce() {
        Color saved = color_;
        ClearFb(fb_left_); ClearFb(fb_right_); PushBoth();
        vTaskDelay(pdMS_TO_TICKS(100));
        RenderBothOpen(saved); PushBoth();
    }
    void BlinkOnceAsync() {
        xTaskCreate([](void* a) { static_cast<Gc9a01Eyes*>(a)->BlinkOnce(); vTaskDelete(NULL); },
                    "blink", 2048, this, 3, nullptr);
    }

    EyeMode mode() const { return mode_; }
    float brightness() const { return brightness_; }
    Color color() const { return color_; }

private:
    // ======== RGB565 工具 ========
    static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
    }
    static void ClearFb(uint16_t* fb) {
        memset(fb, 0, EYE_RESOLUTION * EYE_RESOLUTION * 2);
    }

    // ======== 像素绘制 ========
    static void FillRect(uint16_t* fb, int x, int y, int w, int h, uint16_t color) {
        if (x < 0) { w += x; x = 0; }
        if (y < 0) { h += y; y = 0; }
        if (x + w > EYE_RESOLUTION) w = EYE_RESOLUTION - x;
        if (y + h > EYE_RESOLUTION) h = EYE_RESOLUTION - y;
        if (w <= 0 || h <= 0) return;
        for (int row = 0; row < h; row++) {
            uint16_t* line = fb + (y + row) * EYE_RESOLUTION + x;
            for (int col = 0; col < w; col++) line[col] = color;
        }
    }
    static void FillCircle(uint16_t* fb, int cx, int cy, int r, uint16_t color) {
        // 整数 Bresenham 圆算法，避免 sqrtf
        int x = 0, y = r;
        int d = 3 - 2 * r;
        auto fillHLine = [&](int x0, int x1, int py) {
            if (py < 0 || py >= EYE_RESOLUTION) return;
            if (x0 < 0) x0 = 0;
            if (x1 >= EYE_RESOLUTION) x1 = EYE_RESOLUTION - 1;
            if (x0 > x1) return;
            uint16_t* line = fb + py * EYE_RESOLUTION;
            for (int px = x0; px <= x1; px++) line[px] = color;
        };
        while (x <= y) {
            fillHLine(cx - x, cx + x, cy + y);
            fillHLine(cx - x, cx + x, cy - y);
            fillHLine(cx - y, cx + y, cy + x);
            fillHLine(cx - y, cx + y, cy - x);
            if (d < 0) {
                d += 4 * x + 6;
            } else {
                d += 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    }

    // ======== 画眼睛 ========
    // 中心点:(cx, cy),眼白半径 r,虹膜颜色 iris
    void RenderEye(uint16_t* fb, int cx, int cy, int r, Color iris, float lid_pct) {
        ClearFb(fb);
        // 1) 眼白
        FillCircle(fb, cx, cy, r, RGB565(245, 245, 255));
        // 2) 虹膜
        int iris_r = EYE_IRIS_RADIUS;
        FillCircle(fb, cx, cy - 2, iris_r, RGB565(iris.r, iris.g, iris.b));
        // 3) 瞳孔
        int pupil_r = EYE_PUPIL_RADIUS;
        FillCircle(fb, cx, cy - 2, pupil_r, RGB565(10, 10, 15));
        // 4) 高光
        FillCircle(fb, cx + 12, cy - 14, EYE_HIGHLIGHT_RADIUS, RGB565(255, 255, 255));
        FillCircle(fb, cx + 8,  cy - 10, 4, RGB565(255, 255, 255));
        // 5) 上眼睑(从顶部覆盖)
        if (lid_pct > 0.001f) {
            int lid_h = (int)((float)(r * 2) * lid_pct);
            uint16_t lid_c = RGB565(EYE_LID_COLOR_R, EYE_LID_COLOR_G, EYE_LID_COLOR_B);
            FillRect(fb, cx - r, cy - r, r * 2, lid_h, lid_c);
        }
    }

    void RenderBothOpen(Color iris) {
        RenderEye(fb_left_,  120, 120, 115, iris, 0.0f);
        RenderEye(fb_right_, 120, 120, 115, iris, 0.0f);
    }

    void RenderBothWithLid(float lid_pct, Color iris) {
        RenderEye(fb_left_,  120, 120, 115, iris, lid_pct);
        RenderEye(fb_right_, 120, 120, 115, iris, lid_pct);
    }

    void RenderAngryEyes() {
        Color r = Color::Red();
        // 生气:瞳孔缩小+上移,上眼睑下压成 V 形
        int iris_r = EYE_IRIS_RADIUS - 5;
        for (auto* fb : {fb_left_, fb_right_}) {
            ClearFb(fb);
            int cx = 120, cy = 120;
            FillCircle(fb, cx, cy, 115, RGB565(245, 245, 255));
            FillCircle(fb, cx, cy - 10, iris_r, RGB565(r.r, r.g, r.b));
            FillCircle(fb, cx, cy - 10, EYE_PUPIL_RADIUS - 3, RGB565(10, 10, 15));
            // 上眼睑下压 + 内斜
            uint16_t lid = RGB565(EYE_LID_COLOR_R, EYE_LID_COLOR_G, EYE_LID_COLOR_B);
            for (int y = cy - 115; y < cy; y++) {
                int ang = (y - (cy - 115)) * 2;
                int margin = std::min(30, ang / 4);
                int x0 = cx - 115 + margin;
                int x1 = cx + 115 - margin;
                FillRect(fb, x0, y, x1 - x0, 1, lid);
            }
        }
    }

    // ======== SPI 面板初始化 ========
    // 返回 true=成功, false=失败(非致命,屏没接也能跑)
    bool InitPanel(gpio_num_t cs, esp_lcd_panel_io_handle_t& io, esp_lcd_panel_handle_t& panel) {
        esp_lcd_panel_io_spi_config_t io_cfg = {};
        io_cfg.cs_gpio_num = cs;
        io_cfg.dc_gpio_num = dc_;
        io_cfg.spi_mode = 0;
        io_cfg.pclk_hz = EYE_PCLK_HZ;
        io_cfg.trans_queue_depth = 10;
        io_cfg.lcd_cmd_bits = 8;
        io_cfg.lcd_param_bits = 8;
        io_cfg.on_color_trans_done = nullptr;
        io_cfg.user_ctx = nullptr;
        esp_err_t ret = esp_lcd_new_panel_io_spi(host_, &io_cfg, &io);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Eye CS=%d: panel IO failed (err=0x%x), skipping", cs, ret);
            return false;
        }

        gc9a01_vendor_config_t vendor = {};
        vendor.init_cmds = nullptr;
        vendor.init_cmds_size = 0;

        esp_lcd_panel_dev_config_t panel_cfg = {};
        panel_cfg.reset_gpio_num = GPIO_NUM_NC;
        panel_cfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_cfg.bits_per_pixel = 16;
        panel_cfg.vendor_config = &vendor;
        ret = esp_lcd_new_panel_gc9a01(io, &panel_cfg, &panel);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Eye CS=%d: panel create failed (err=0x%x), freeing IO", cs, ret);
            esp_lcd_panel_io_del(io);
            io = nullptr;
            return false;
        }

        ret = esp_lcd_panel_reset(panel);
        if (ret != ESP_OK) { ESP_LOGW(TAG, "Eye CS=%d: panel reset warn (err=0x%x)", cs, ret); }

        ret = esp_lcd_panel_init(panel);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Eye CS=%d: panel init failed (err=0x%x) - display not connected?", cs, ret);
            esp_lcd_panel_del(panel);
            esp_lcd_panel_io_del(io);
            panel = nullptr; io = nullptr;
            return false;
        }

        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_mirror(panel, false, false);
        esp_lcd_panel_disp_on_off(panel, true);
        ESP_LOGI(TAG, "Eye CS=%d: GC9A01 OK", cs);
        return true;
    }

    void InitPanels() {
        bool left_ok  = InitPanel(cs_left_,  io_left_,  panel_left_);
        bool right_ok = InitPanel(cs_right_, io_right_, panel_right_);
        // AllocFramebuffers 前置的条件检查
        if (!left_ok && !right_ok) {
            ESP_LOGW(TAG, "No GC9A01 eyes detected - running headless");
            return;
        }
        if (!left_ok)  { ESP_LOGW(TAG, "Left eye missing, right eye only"); }
        if (!right_ok) { ESP_LOGW(TAG, "Right eye missing, left eye only"); }
    }

    void AllocFramebuffers() {
        size_t sz = EYE_RESOLUTION * EYE_RESOLUTION * 2;
        fb_left_  = (uint16_t*)heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        fb_right_ = (uint16_t*)heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        assert(fb_left_ && fb_right_);
    }

    void PushPanel(esp_lcd_panel_handle_t panel, uint16_t* fb) {
        if (!panel || !fb) return;
        esp_lcd_panel_draw_bitmap(panel, 0, 0, EYE_RESOLUTION, EYE_RESOLUTION, fb);
    }
    void PushBoth() {
        PushPanel(panel_left_, fb_left_);
        PushPanel(panel_right_, fb_right_);
    }

    // ======== 动画 ========
    void StartAnimation(int ms) {
        if (animating_) return;
        animating_ = true;
        anim_period_ms_ = ms;
        xTaskCreate(AnimTaskFunc, "eye_anim", 3072, this, 3, &anim_task_);
    }
    void StopAnimation() {
        animating_ = false;
        if (anim_task_) {
            // 等待动画任务自行退出（最多 500ms）
            for (int i = 0; i < 5 && anim_task_; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            if (anim_task_) {
                vTaskDelete(anim_task_);
                ESP_LOGW(TAG, "Animation task force-killed");
            }
            anim_task_ = nullptr;
        }
    }
    static void AnimTaskFunc(void* a) { static_cast<Gc9a01Eyes*>(a)->AnimTask(); }

    void AnimTask() {
        int step = 0, total_steps = anim_period_ms_ / 20;
        if (total_steps < 10) total_steps = 10;
        Color iris = color_;
        while (animating_) {
            switch (mode_) {
                case kBreathe: {
                    float t = (float)(step % total_steps) / (float)total_steps;
                    float scale = 0.7f + 0.3f * sinf(t * 2.0f * 3.14159f);
                    int ir = (int)(EYE_IRIS_RADIUS * scale);
                    int pr = (int)(EYE_PUPIL_RADIUS * scale);
                    RenderBreathFrame(iris, ir, pr);
                    break;
                }
                case kSleepy: {
                    float t = (float)(step % total_steps) / (float)total_steps;
                    float lid = 0.3f + 0.3f * (0.5f + 0.5f * sinf(t * 2.0f * 3.14159f));
                    RenderBothWithLid(lid, Color::Blue());
                    PushBoth();
                    break;
                }
                case kBlink: {
                    int ss = step % 30;
                    float lid = 0.0f;
                    if (ss < 5)       lid = (float)ss / 5.0f;       // 闭眼
                    else if (ss < 10) lid = 1.0f;                     // 全闭
                    else if (ss < 15) lid = 1.0f - (float)(ss-10)/5.0f; // 睁眼
                    else              lid = 0.0f;                     // 全开
                    RenderBothWithLid(lid, iris);
                    PushBoth();
                    break;
                }
                case kAngry: {
                    RenderAngryEyes(); PushBoth();
                    break;
                }
                default:
                    animating_ = false;
                    break;
            }
            step = (step + 1) % total_steps;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        anim_task_ = nullptr;
        vTaskDelete(NULL);
    }

    void RenderBreathFrame(Color iris, int iris_r, int pupil_r) {
        for (auto* fb : {fb_left_, fb_right_}) {
            ClearFb(fb);
            FillCircle(fb, 120, 120, 115, RGB565(245, 245, 255));
            FillCircle(fb, 120, 118, iris_r, RGB565(iris.r, iris.g, iris.b));
            FillCircle(fb, 120, 118, pupil_r, RGB565(10, 10, 15));
            FillCircle(fb, 132, 106, EYE_HIGHLIGHT_RADIUS, RGB565(255, 255, 255));
        }
        PushBoth();
    }

    spi_host_device_t host_;
    gpio_num_t dc_, cs_left_, cs_right_;
    esp_lcd_panel_io_handle_t io_left_ = nullptr, io_right_ = nullptr;
    esp_lcd_panel_handle_t panel_left_ = nullptr, panel_right_ = nullptr;
    uint16_t* fb_left_ = nullptr;
    uint16_t* fb_right_ = nullptr;
    EyeMode mode_ = kOn;
    float brightness_ = 0.6f;
    Color color_;
    bool animating_ = false;
    int anim_period_ms_ = 2000;
    TaskHandle_t anim_task_ = nullptr;
};

// 保留别名方便过渡
using WalleEyes = Gc9a01Eyes;

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
        // 初始化所有关节到默认姿态
        SetArmLeft(ARM_LEFT_CENTER);
        SetArmRight(ARM_RIGHT_CENTER);
        SetHeadUD(HEAD_UD_CENTER);
        SetNeckLR(NECK_LR_CENTER);
        SetNeckOI(NECK_OI_CENTER);
        SetEyeLeft(EYE_LEFT_CENTER);
        SetEyeRight(EYE_RIGHT_CENTER);
        SetBrowLeft(BROW_LEFT_CENTER);
        SetBrowRight(BROW_RIGHT_CENTER);
    }

    void SetExpression(Expression expr) {
        current_expr_ = expr;
        // 眼睛联动(颜色+模式)
        if (eyes_) {
            switch (expr) {
                case kHappy:   eyes_->SetColor(WalleEyes::Color::WarmYellow()); eyes_->SetMode(WalleEyes::kOn); eyes_->SetBrightness(1.0f); break;
                case kSad:     eyes_->SetColor(WalleEyes::Color::Blue()); eyes_->SetMode(WalleEyes::kSleepy); eyes_->SetBrightness(0.4f); break;
                case kCurious: eyes_->SetColor(WalleEyes::Color::Green()); eyes_->BlinkOnceAsync(); eyes_->SetMode(WalleEyes::kOn); eyes_->SetBrightness(0.8f); break;
                case kScared:  eyes_->SetColor(WalleEyes::Color::White()); eyes_->SetMode(WalleEyes::kBlink); eyes_->SetBrightness(1.0f); break;
                case kWave:    eyes_->SetColor(WalleEyes::Color::WarmYellow()); eyes_->SetMode(WalleEyes::kOn); eyes_->SetBrightness(0.8f); break;
                case kNeutral:
                default:       eyes_->SetColor(WalleEyes::Color::WarmYellow()); eyes_->SetMode(WalleEyes::kOn); eyes_->SetBrightness(0.6f); break;
            }
        }
        // 全关节联动(9舵机)
        switch (expr) {
            case kHappy:
                SetArmLeft(40.0f);
                SetArmRight(140.0f);
                SetHeadUD(110.0f);       // 头微抬
                SetNeckLR(NECK_LR_CENTER);
                SetNeckOI(NECK_OI_CENTER);
                SetBrowLeft(110.0f);     // 眉毛上挑
                SetBrowRight(70.0f);
                break;
            case kSad:
                SetArmLeft(80.0f);
                SetArmRight(100.0f);
                SetHeadUD(60.0f);        // 低头
                SetNeckLR(NECK_LR_CENTER);
                SetNeckOI(60.0f);        // 脖子后缩
                SetBrowLeft(70.0f);      // 眉毛下垂
                SetBrowRight(110.0f);
                break;
            case kCurious:
                SetArmLeft(50.0f);
                SetArmRight(110.0f);
                SetHeadUD(100.0f);
                SetNeckLR(70.0f);        // 头微偏
                SetNeckOI(110.0f);       // 脖子前伸
                SetBrowLeft(100.0f);
                SetBrowRight(80.0f);
                break;
            case kScared:
                SetArmLeft(30.0f);
                SetArmRight(150.0f);
                SetHeadUD(50.0f);        // 猛低头
                SetNeckLR(NECK_LR_CENTER);
                SetNeckOI(50.0f);        // 猛后缩
                SetBrowLeft(70.0f);
                SetBrowRight(110.0f);
                break;
            case kWave:
                SetHeadUD(HEAD_UD_CENTER);
                SetNeckLR(NECK_LR_CENTER);
                SetNeckOI(NECK_OI_CENTER);
                SetBrowLeft(BROW_LEFT_CENTER);
                SetBrowRight(BROW_RIGHT_CENTER);
                StartWave();
                return;
            case kNeutral:
            default:
                SetArmLeft(ARM_LEFT_CENTER);
                SetArmRight(ARM_RIGHT_CENTER);
                SetHeadUD(HEAD_UD_CENTER);
                SetNeckLR(NECK_LR_CENTER);
                SetNeckOI(NECK_OI_CENTER);
                SetBrowLeft(BROW_LEFT_CENTER);
                SetBrowRight(BROW_RIGHT_CENTER);
                break;
        }
        StopWave();
    }

    Expression current_expression() const { return current_expr_; }

    void SetArmLeft(float angle) {
        left_arm_angle_ = std::clamp(angle, ARM_MIN, ARM_MAX);
        pca_->SetServoAngle(SERVO_L_ARM_CH, left_arm_angle_);
    }

    void SetArmRight(float angle) {
        right_arm_angle_ = std::clamp(angle, ARM_MIN, ARM_MAX);
        pca_->SetServoAngle(SERVO_R_ARM_CH, right_arm_angle_);
    }

    void SetHeadUD(float angle) {
        head_ud_angle_ = std::clamp(angle, HEAD_UD_MIN, HEAD_UD_MAX);
        pca_->SetServoAngle(SERVO_HEAD_UD_CH, head_ud_angle_);
    }

    void SetNeckLR(float angle) {
        neck_lr_angle_ = std::clamp(angle, NECK_LR_MIN, NECK_LR_MAX);
        pca_->SetServoAngle(SERVO_NECK_LR_CH, neck_lr_angle_);
    }

    void SetNeckOI(float angle) {
        neck_oi_angle_ = std::clamp(angle, NECK_OI_MIN, NECK_OI_MAX);
        pca_->SetServoAngle(SERVO_NECK_OI_CH, neck_oi_angle_);
    }

    void SetEyeLeft(float angle) {
        eye_left_angle_ = std::clamp(angle, EYE_MIN, EYE_MAX);
        pca_->SetServoAngle(SERVO_L_EYE_CH, eye_left_angle_);
    }

    void SetEyeRight(float angle) {
        eye_right_angle_ = std::clamp(angle, EYE_MIN, EYE_MAX);
        pca_->SetServoAngle(SERVO_R_EYE_CH, eye_right_angle_);
    }

    void SetBrowLeft(float angle) {
        brow_left_angle_ = std::clamp(angle, BROW_MIN, BROW_MAX);
        pca_->SetServoAngle(SERVO_L_BROW_CH, brow_left_angle_);
    }

    void SetBrowRight(float angle) {
        brow_right_angle_ = std::clamp(angle, BROW_MIN, BROW_MAX);
        pca_->SetServoAngle(SERVO_R_BROW_CH, brow_right_angle_);
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
            // 等待任务自行退出（最多 1 秒）
            for (int i = 0; i < 10 && wave_task_; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            if (wave_task_) {
                // 任务卡死，强制删除
                vTaskDelete(wave_task_);
                ESP_LOGW(TAG, "Wave task force-killed");
            }
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
    float head_ud_angle_ = HEAD_UD_CENTER;
    float neck_lr_angle_ = NECK_LR_CENTER;
    float neck_oi_angle_ = NECK_OI_CENTER;
    float eye_left_angle_ = EYE_LEFT_CENTER;
    float eye_right_angle_ = EYE_RIGHT_CENTER;
    float brow_left_angle_ = BROW_LEFT_CENTER;
    float brow_right_angle_ = BROW_RIGHT_CENTER;
    bool waving_ = false;
    TaskHandle_t wave_task_ = nullptr;
};


// ============ VL53L0X 激光测距传感器 ============
class VL53L0X {
public:
    VL53L0X(i2c_master_bus_handle_t i2c_bus, uint8_t addr = VL53L0X_I2C_ADDR)
        : i2c_bus_(i2c_bus), addr_(addr), last_distance_mm_(0), initialized_(false) {}

    bool Init() {
        // 检测设备是否在线:读取 VL53L0X 标识寄存器 0x00C0
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

        // 初始化序列:设置信号速率、VCSEL 脉冲周期、测量预算
        // 简化初始化 - 写入必要寄存器使传感器进入测距模式
        WriteReg16(0x0080, 0x0001);  // SYSRANGE_START bit0=1 (单次测距)
        WriteReg16(0x0083, 0x0004);  // SYSTEM_INTERRUPT_CONFIG_GPIO = 4 (new sample ready)

        initialized_ = true;
        ESP_LOGI(TAG, "VL53L0X initialized at I2C 0x%02X", addr_);
        return true;
    }

    /** 读取距离(毫米),失败返回 -1 */
    int ReadDistanceMm() {
        if (!initialized_) return -1;

        // 启动单次测距
        WriteReg16(0x0018, 0x0001);

        // 等待测量完成(轮询方式,最大 30ms)
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

    /** 带中值滤波的读取(3次采样取中值,抗干扰) */
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

// ============ 人物追踪器(WALL-E 增强:云台+底盘跟随+激光测距) ============
class PersonTracker {
public:
    struct TrackResult {
        bool person_found = false;
        float center_x = 0.5f;
        float center_y = 0.5f;
        float confidence = 0.0f;
        int distance_mm = -1;  // VL53L0X 测距结果,-1=不可用
    };

    PersonTracker(PanTilt* pan_tilt, Camera* camera, TB6612* motor, PCA9685* pca, VL53L0X* tof)
        : pan_tilt_(pan_tilt), camera_(camera), motor_(motor), pca_(pca), tof_(tof) {}

    ~PersonTracker() {
        StopTracking();
    }

    /** 单次追踪:拍照 → AI分析 → 测距 → 驱动云台+底盘 */
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

        // 读取 VL53L0X 测距(如果可用)
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

        // ======= 距离控制(优先使用 VL53L0X 激光测距) =======
        float base_speed = follow_speed_;  // 默认速度

        if (result.distance_mm > 0) {
            // 有激光测距数据 - 精确距离控制
            if (result.distance_mm < TRACK_TOO_CLOSE_MM) {
                // 太近!停止或后退
                base_speed = 0.0f;
                ESP_LOGD(TAG, "Too close: %dmm, stopping", result.distance_mm);
            } else if (result.distance_mm < TRACK_FOLLOW_DIST_MM - TRACK_DIST_TOLERANCE_MM) {
                // 偏近,慢速跟随
                base_speed = follow_speed_ * 0.3f;
            } else if (result.distance_mm > TRACK_TOO_FAR_MM) {
                // 太远,加速追
                base_speed = follow_speed_ * 1.3f;
                ESP_LOGD(TAG, "Too far: %dmm, speeding up", result.distance_mm);
            } else if (result.distance_mm > TRACK_FOLLOW_DIST_MM + TRACK_DIST_TOLERANCE_MM) {
                // 偏远,正常速度
                base_speed = follow_speed_;
            } else {
                // 距离合适,低速维持
                base_speed = follow_speed_ * 0.15f;
            }
        } else {
            // 无激光测距 - 回退到视觉估算(兼容无 VL53L0X 的情况)
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

#include "external_audio.h"

// [v10] ExternalAudio 现在实现 AudioCodec 接口，替代板载 ES8388
// 使用 I2S1 全双工: PCM5102 DAC (输出) + INMP441 麦克风 (输入)
// GPIO15(BCLK), GPIO18(WS), GPIO45(DOUT), GPIO47(DIN)

// ============ 板型定义 ============
class atk_dnesp32s3 : public WifiBoard {
private:
    // [v10] 已删除：i2c_bus_ (I2C0 内部总线) — 不再使用板载 XL9555/ES8388
    i2c_master_bus_handle_t i2c_bus_ext_;  // I2C1: 外部总线 (PCA9685 + VL53L0X)
    Button boot_button_;
    LcdDisplay* display_;
    // [v10] 已删除：XL9555* xl9555_; — 不再使用板载 IO 扩展
    UartCamera* camera_;
    PCA9685* pca9685_;
    TB6612* motor_driver_;
    PanTilt* pan_tilt_;
    WalleExpression* expression_;
    WalleEyes* eyes_;
    VL53L0X* tof_;
    PersonTracker* tracker_;
    ExternalAudio* ext_audio_;    // I2S1: PCM5102 + INMP441 (实现 AudioCodec 接口)

    void InitializeI2c() {
        // [v10] 已删除：I2C0 内部总线 (GPIO41/42) — XL9555/ES8388 不再使用

        // I2C1 - 外部总线 (GPIO38/39, P1排针): PCA9685 (0x40) + VL53L0X (0x29)
        // 摄像头已外置 ESP32-CAM 子板，I2C1 不再与 SCCB 共享
        i2c_master_bus_config_t i2c_ext_cfg = {
            .i2c_port = (i2c_port_t)EXTERNAL_I2C_PORT,
            .sda_io_num = EXTERNAL_I2C_SDA_PIN,
            .scl_io_num = EXTERNAL_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_ext_cfg, &i2c_bus_ext_));
        ESP_LOGI(TAG, "I2C1 external bus initialized: SDA=%d, SCL=%d (PCA9685 + VL53L0X)",
                 EXTERNAL_I2C_SDA_PIN, EXTERNAL_I2C_SCL_PIN);
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

    void InitializeSt7735Display() {
        // [v10] 改为 ST7735S 1.3" 240×240 外接 LCD, SPI 引脚从 P1 排针引出
        // LCD_BL_PIN (GPIO48) GPIO 直控背光，不再走 XL9555

        // 初始化 LCD 背光 GPIO
        gpio_config_t bl_cfg = {};
        bl_cfg.pin_bit_mask = (1ULL << LCD_BL_PIN);
        bl_cfg.mode = GPIO_MODE_OUTPUT;
        bl_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        bl_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        bl_cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&bl_cfg);

        // 初始化眼屏 CS GPIO (GPIO19/20)
        gpio_config_t eye_cs_cfg = {};
        eye_cs_cfg.pin_bit_mask = (1ULL << EYE_CS_LEFT_PIN) | (1ULL << EYE_CS_RIGHT_PIN);
        eye_cs_cfg.mode = GPIO_MODE_OUTPUT;
        eye_cs_cfg.pull_up_en = GPIO_PULLUP_ENABLE;  // CS 默认高电平(未选中)
        eye_cs_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        eye_cs_cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&eye_cs_cfg);
        // 默认拉高 CS (未选中)
        gpio_set_level(EYE_CS_LEFT_PIN, 1);
        gpio_set_level(EYE_CS_RIGHT_PIN, 1);

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
        // ST7735S 使用 ST7789 驱动 + 正确的 offset/mirror 参数
        // ST7735S 240x240 的初始化序列与 ST7789 兼容性较高
        esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel);

        esp_lcd_panel_reset(panel);
        gpio_set_level(LCD_BL_PIN, 1);   // [v10] GPIO 直控背光开
        // [v10] 已删除：xl9555_->SetOutputState(8, 1) / xl9555_->SetOutputState(2, 0)

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, false);  // [v10] ST7735S 通常不反转
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                    DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                    DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeCamera() {
        // ESP32-CAM 子板通过 UART2 连接，不再使用 DVP 接口
        UartCamera::Config cam_cfg;
        cam_cfg.uart_port = UART_CAM_PORT;
        cam_cfg.tx_pin = UART_CAM_TX_PIN;    // GPIO16
        cam_cfg.rx_pin = UART_CAM_RX_PIN;    // GPIO17
        cam_cfg.baud_rate = UART_CAM_BAUD;   // 921600
        cam_cfg.capture_timeout_ms = 5000;

        camera_ = new UartCamera(cam_cfg);
        if (!camera_) {
            ESP_LOGE(TAG, "UartCamera allocation failed");
            return;
        }

        // 测试连接：发送一个镜像设置命令
        camera_->SetHMirror(false);
        ESP_LOGI(TAG, "UartCamera initialized: UART%d TX=%d RX=%d @ %d baud",
                 UART_CAM_PORT, UART_CAM_TX_PIN, UART_CAM_RX_PIN, UART_CAM_BAUD);
    }

    /** 初始化 PCA9685 PWM 驱动板(fail-soft:未连接不崩溃)
     *  挂在外部 I2C1 总线 (GPIO38/39) */
    void InitializePCA9685() {
        // I2C probe: 确认 PCA9685 物理连接
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = PCA9685_I2C_ADDR,
            .scl_speed_hz = 100000,
        };
        i2c_master_dev_handle_t dev;
        esp_err_t ret = i2c_master_bus_add_device(i2c_bus_ext_, &dev_cfg, &dev);
        if (ret == ESP_OK) {
            // 软件复位：写 0x06 到 MODE1 寄存器，然后读回确认
            uint8_t reset_cmd[2] = {0x00, 0x06};  // MODE1 = SWRST
            ret = i2c_master_transmit(dev, reset_cmd, 2, 50);
            if (ret == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(10));  // 等待复位完成
                uint8_t reg = 0x00, val = 0;
                ret = i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 50);
                if (ret == ESP_OK && val == 0x00) {
                    ESP_LOGI(TAG, "PCA9685 probe OK at 0x%02X (MODE1=0x%02X after reset)", PCA9685_I2C_ADDR, val);
                } else {
                    ESP_LOGW(TAG, "PCA9685 at 0x%02X: reset ack but MODE1=0x%02X (expected 0x00)", PCA9685_I2C_ADDR, val);
                }
            }
            i2c_master_bus_rm_device(dev);
        }
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "PCA9685 not found at 0x%02X - servos/motors disabled", PCA9685_I2C_ADDR);
            pca9685_ = nullptr;
            return;
        }

        pca9685_ = new PCA9685(i2c_bus_ext_, PCA9685_I2C_ADDR, PCA9685_SERVO_FREQ_HZ);
        for (int i = 0; i < 16; i++) {
            pca9685_->SetDutyCycle(i, 0);
        }
        ESP_LOGI(TAG, "PCA9685 OK at 0x%02X", PCA9685_I2C_ADDR);
    }

    /** 初始化 TB6612 电机驱动 */
    void InitializeMotor() {
        // TB6612 方向引脚直连 P1 排针 GPIO4/5/6/7
        TB6612::MotorPins motor_a = {
            .in1 = TB6612_AIN1_GPIO,   // GPIO4
            .in2 = TB6612_AIN2_GPIO,   // GPIO5
            .pca_channel = MOTOR_LEFT_PWM_CH,
            // [v10] 已删除：xl9555_in1_bit / xl9555_in2_bit
        };
        TB6612::MotorPins motor_b = {
            .in1 = TB6612_BIN1_GPIO,   // GPIO6
            .in2 = TB6612_BIN2_GPIO,   // GPIO7
            .pca_channel = MOTOR_RIGHT_PWM_CH,
            // [v10] 已删除：xl9555_in1_bit / xl9555_in2_bit
        };

        motor_driver_ = new TB6612(motor_a, motor_b);
        // GPIO 直连模式，无需 XL9555 回调
        ESP_LOGI(TAG, "TB6612 motor driver initialized (GPIO%d/%d/%d/%d direct)",
                 TB6612_AIN1_GPIO, TB6612_AIN2_GPIO, TB6612_BIN1_GPIO, TB6612_BIN2_GPIO);
    }

    /** 初始化云台(PCA9685 模式) */
    void InitializePanTilt() {
        if (!pca9685_) {
            ESP_LOGW(TAG, "PanTilt: PCA9685 not available, skipping");
            pan_tilt_ = nullptr;
            return;
        }
        PanTilt::Config cfg;
        cfg.mode = PanTilt::DriverMode::PCA9685;
        cfg.min_angle = NECK_LR_MIN;
        cfg.max_angle = NECK_LR_MAX;
        cfg.center_angle = NECK_LR_CENTER;
        cfg.pca.pan_channel = SERVO_NECK_LR_CH;
        cfg.pca.tilt_channel = SERVO_HEAD_UD_CH;

        pan_tilt_ = new PanTilt(cfg);
        pan_tilt_->SetPcaDriver(pca9685_);
        ESP_LOGI(TAG, "PanTilt(PCA9685) initialized: neck_lr=CH%d, head_ud=CH%d",
                 SERVO_NECK_LR_CH, SERVO_HEAD_UD_CH);
    }

    /** 初始化 WALL-E 表情控制 */
    void InitializeExpression() {
        InitializeEyes();  // 已 fail-soft
        if (!pca9685_) {
            ESP_LOGW(TAG, "WalleExpression: PCA9685 not available, skipping");
            expression_ = nullptr;
            return;
        }
        expression_ = new WalleExpression(pca9685_, eyes_);
        ESP_LOGI(TAG, "WALLE-Expression initialized");
    }

    /** 初始化 WALL-E 眼睛 - 延迟到硬件就绪(阶段5),避免启动时消耗内存 */
    void InitializeEyes() {
        // GC9A01 双屏帧缓冲 230KB PSRAM + animation task 3KB 栈,
        // 未连接时完全不分配,保证 WiFi / 语音等核心功能有足够内存
        ESP_LOGI(TAG, "WALLE-Eyes deferred (GC9A01 Stage5) - call walle_eyes_start after connecting screens");
        eyes_ = nullptr;
    }

    // StartEyes() moved to public section - see below

    /** 初始化 VL53L0X 激光测距传感器(挂在外部 I2C1 总线) */
    void InitializeToF() {
        tof_ = new VL53L0X(i2c_bus_ext_, VL53L0X_I2C_ADDR);
        bool ok = tof_->Init();
        if (!ok) {
            ESP_LOGW(TAG, "VL53L0X init failed - distance tracking disabled, falling back to visual-only");
        }
    }

    /** 初始化人物追踪器 */
    void InitializeTracker() {
        bool tof_ok = (tof_ != nullptr) && tof_->IsInitialized();
        if (!pan_tilt_ && !camera_) {
            ESP_LOGW(TAG, "PersonTracker: no pan/tilt or camera, skipping");
            tracker_ = nullptr;
            return;
        }
        tracker_ = new PersonTracker(pan_tilt_, camera_, motor_driver_, pca9685_, tof_);
        ESP_LOGI(TAG, "PersonTracker initialized (ToF=%s)", tof_ok ? "YES" : "NO");
    }

    void InitializeExternalAudio() {
        // I2S1 全双工: PCM5102 DAC (输出) + INMP441 麦克风 (输入)
        // [v10] ExternalAudio 实现框架 AudioCodec 接口，替代 ES8388
        ext_audio_ = new ExternalAudio();
        ExternalAudio::Config cfg;
        cfg.bclk_pin = EXTERNAL_AUDIO_BCLK_PIN;     // GPIO15
        cfg.ws_pin   = EXTERNAL_AUDIO_WS_PIN;       // GPIO18
        cfg.dout_pin = EXTERNAL_AUDIO_DOUT_PIN;     // GPIO45 → PCM5102 DIN
        cfg.din_pin  = EXTERNAL_AUDIO_DIN_PIN;      // GPIO47 ← INMP441 SD
        cfg.sample_rate     = EXTERNAL_AUDIO_SAMPLE_RATE;     // 24000
        cfg.output_channels = EXTERNAL_AUDIO_OUT_CHANNELS;    // 2 (立体声)
        cfg.input_channels  = EXTERNAL_AUDIO_IN_CHANNELS;     // 1 (单声道)
        esp_err_t ret = ext_audio_->Initialize(cfg);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ExternalAudio initialized: I2S1 (PCM5102+INMP441)");
        } else {
            ESP_LOGE(TAG, "ExternalAudio init failed: %s", esp_err_to_name(ret));
            delete ext_audio_;
            ext_audio_ = nullptr;
        }
    }

    /** 注册 MCP 工具,支持语音控制 */
    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        // ============ 底盘控制(4个) ============

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

        // ============ 追踪控制(3个) ============

        mcp.AddTool("self.tracker.start",
            "开始追踪人物。WALL-E会跟着人走,找不到人时自动扫描搜索。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                tracker_->StartTracking(2000);
                expression_->SetExpression(WalleExpression::kCurious);
                return true;
            });

        mcp.AddTool("self.tracker.stop",
            "停止追踪人物。云台回到居中位置,底盘停止。",
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
            "拍照检查当前画面中是否有人,返回人物位置和距离信息。",
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

        // ============ 云台控制(3个) ============

        mcp.AddTool("self.head.look_at",
            "控制WALL-E头部看向指定方向。pan控制脖子左右(0=最左,90=正中,180=最右),tilt控制头上下(45=最低,90=正中,135=最高)。",
            PropertyList({
                Property("pan", kPropertyTypeInteger, 90, (int)NECK_LR_MIN, (int)NECK_LR_MAX),
                Property("tilt", kPropertyTypeInteger, 90, (int)HEAD_UD_MIN, (int)HEAD_UD_MAX)
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
                pan_tilt_->StartSweep(NECK_LR_MIN, NECK_LR_MAX, HEAD_UD_CENTER);
                return true;
            });

        mcp.AddTool("self.head.home",
            "WALL-E头部回到正前方。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                pan_tilt_->Home();
                return true;
            });

        // ============ 表情/手臂控制(2个) ============

        mcp.AddTool("self.expression.set",
            "设置WALL-E的表情。0=中性,1=开心(双臂举起),2=难过(双臂下垂),3=好奇(左臂微抬),4=害怕(双臂高举),5=挥手。",
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

        // ============ 眼睛屏控制(5个) ============

        mcp.AddTool("self.eyes.start",
            "启动WALL-E眼睛屏(GC9A01 240x240 TFT)。仅在屏幕已物理连接后调用,启动后会分配帧缓冲并开始渲染瞳孔动画。5阶段调试:阶段5使用。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                if (eyes_) { ESP_LOGW(TAG, "Eyes already started"); return true; }
                // [v10] DC=GPIO14(P1排针), CS_L=GPIO19, CS_R=GPIO20
                eyes_ = new Gc9a01Eyes(EYE_SPI_HOST, LCD_DC_PIN, EYE_CS_LEFT_PIN, EYE_CS_RIGHT_PIN);
                eyes_->SetMode(Gc9a01Eyes::kOn);
                ESP_LOGI(TAG, "MCP: eyes started");
                return true;
            });

        mcp.AddTool("self.eyes.set_mode",
            "设置WALL-E眼睛模式。0=关闭,1=常亮,2=呼吸,3=眨眼,4=生气,5=困倦。需要先调用 self.eyes.start 启动眼睛屏。",
            PropertyList({
                Property("mode", kPropertyTypeInteger, 1, 0, 5)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                if (!eyes_) { ESP_LOGW(TAG, "Eyes not started"); return false; }
                int mode = props["mode"].value<int>();
                eyes_->SetMode(static_cast<WalleEyes::EyeMode>(mode));
                return true;
            });

        mcp.AddTool("self.eyes.set_brightness",
            "设置WALL-E眼睛亮度。0=最暗,100=最亮。需要先调用 self.eyes.start。",
            PropertyList({
                Property("brightness", kPropertyTypeInteger, 60, 0, 100)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                if (!eyes_) { ESP_LOGW(TAG, "Eyes not started"); return false; }
                int val = props["brightness"].value<int>();
                eyes_->SetBrightness(val / 100.0f);
                return true;
            });

        mcp.AddTool("self.eyes.set_color",
            "设置WALL-E眼睛颜色。RGB格式,每个通道0-255。默认暖黄色(255,200,80)。需要先调用 self.eyes.start。",
            PropertyList({
                Property("r", kPropertyTypeInteger, 255, 0, 255),
                Property("g", kPropertyTypeInteger, 200, 0, 255),
                Property("b", kPropertyTypeInteger, 80, 0, 255)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                if (!eyes_) { ESP_LOGW(TAG, "Eyes not started"); return false; }
                int r = props["r"].value<int>();
                int g = props["g"].value<int>();
                int b = props["b"].value<int>();
                eyes_->SetColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
                return true;
            });

        mcp.AddTool("self.eyes.blink",
            "WALL-E眨眼。需要先调用 self.eyes.start。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                if (!eyes_) { ESP_LOGW(TAG, "Eyes not started"); return false; }
                eyes_->BlinkOnceAsync();
                return true;
            });

        // ============ 外接音频控制(2个) ============

        mcp.AddTool("self.audio.play_effect",
            "播放音效到外接喇叭(PCM5102+PAM8406)。传入16-bit PCM数据或内置音效名称。",
            PropertyList({
                Property("effect", kPropertyTypeString, std::string("beep"))
            }),
            [this](const PropertyList& props) -> ReturnValue {
                if (!ext_audio_) { ESP_LOGW(TAG, "External audio not initialized"); return false; }
                std::string effect = props["effect"].value<std::string>();
                // 预计算 1kHz 正弦波 @ 24kHz 采样 (1个完整周期=24采样点)
                static const int16_t kSine24[] = {
                    0,8480,16383,23169,28377,31650,32767,31650,28377,23169,
                    16383,8480,0,-8480,-16383,-23169,-28377,-31650,-32767,
                    -31650,-28377,-23169,-16383,-8480
                };
                if (effect == "beep") {
                    static int16_t beep[480];  // 20ms @ 24kHz = 20 cycles
                    static bool beep_init = false;
                    if (!beep_init) {
                        for (int i = 0; i < 480; i++) beep[i] = kSine24[i % 24];
                        beep_init = true;
                    }
                    ext_audio_->PlayPcmAsync(beep, 480);
                } else if (effect == "double_beep") {
                    static int16_t dbl[960];  // 40ms = 20ms beep + 20ms silence
                    static bool dbl_init = false;
                    if (!dbl_init) {
                        for (int i = 0; i < 480; i++) dbl[i] = kSine24[i % 24];
                        for (int i = 480; i < 960; i++) dbl[i] = 0;
                        dbl_init = true;
                    }
                    ext_audio_->PlayPcmAsync(dbl, 960);
                } else {
                    ESP_LOGW(TAG, "Unknown effect: %s", effect.c_str());
                    return false;
                }
                return true;
            });

        mcp.AddTool("self.audio.set_volume",
            "设置外接喇叭音量(0-100)。",
            PropertyList({
                Property("volume", kPropertyTypeInteger, 80, 0, 100)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                if (!ext_audio_) { ESP_LOGW(TAG, "External audio not initialized"); return false; }
                int vol = props["volume"].value<int>();
                ext_audio_->SetOutputVolume(vol);
                return true;
            });

        ESP_LOGI(TAG, "MCP tools registered (20 WALL-E tools)");
    }

public:
    atk_dnesp32s3()
        : boot_button_(BOOT_BUTTON_GPIO)
        , display_(nullptr)
        // [v10] 已删除：xl9555_(nullptr)
        , camera_(nullptr)
        , pca9685_(nullptr)
        , motor_driver_(nullptr)
        , pan_tilt_(nullptr)
        , expression_(nullptr)
        , eyes_(nullptr)
        , tof_(nullptr)
        , tracker_(nullptr)
        , ext_audio_(nullptr) {

        InitializeI2c();
        InitializeSpi();
        InitializeSt7735Display();  // [v10] 改为 ST7735S
        InitializeButtons();
        InitializeCamera();
        InitializePCA9685();
        InitializeMotor();
        InitializePanTilt();
        InitializeExpression();
        // InitializeEyes() 已在 InitializeExpression() 内部调用
        InitializeToF();
        InitializeTracker();
        InitializeExternalAudio();
        InitializeTools();

    }

    virtual void OnNetworkEvent(NetworkEvent event, const std::string& data) override {
        // 先调用父类处理（WiFi 事件分发、application 回调等）
        WifiBoard::OnNetworkEvent(event, data);
        // WiFi 连接成功后启动 Web 调试服务器
        if (event == NetworkEvent::Connected) {
            ESP_LOGI("WALLE", "Network connected, starting debug server...");
            walle_debug_server_start();
        }
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        // [v10] 使用 ExternalAudio (PCM5102+INMP441) 替代板载 ES8388
        return ext_audio_;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    // ============ WALL-E 公共访问接口 ============
    WalleEyes* GetWalleEyes() { return eyes_; }

    /** 启动眼睛屏(5阶段调试:阶段5)- 仅在 GC9A01 物理连接后调用 */
    void StartEyes() {
        if (eyes_) {
            ESP_LOGW(TAG, "Eyes already started");
            return;
        }
        // [v10] CS 改 GPIO 直控: EYE_CS_LEFT_PIN=GPIO19, EYE_CS_RIGHT_PIN=GPIO20
        eyes_ = new Gc9a01Eyes(EYE_SPI_HOST, LCD_DC_PIN, EYE_CS_LEFT_PIN, EYE_CS_RIGHT_PIN);
        eyes_->SetMode(Gc9a01Eyes::kOn);
        ESP_LOGI(TAG, "WALLE-Eyes started (GC9A01 share LCD bus, CS_L=%d CS_R=%d)",
                 EYE_CS_LEFT_PIN, EYE_CS_RIGHT_PIN);
    }
    PCA9685* GetPca9685() { return pca9685_; }
    PanTilt* GetPanTilt() { return pan_tilt_; }
    TB6612* GetMotorDriver() { return motor_driver_; }
    WalleExpression* GetWalleExpression() { return expression_; }
    VL53L0X* GetVl53l0x() { return tof_; }
    ExternalAudio* GetExternalAudio() { return ext_audio_; }
    i2c_master_bus_handle_t GetI2cBus() { return i2c_bus_ext_; }  // [v10] 只返回 I2C1 外部总线
    i2c_master_bus_handle_t GetI2cBusExt() { return i2c_bus_ext_; }
};

// ============ WALL-E C 包装函数(供 walle_debug_server.cc 调用)============
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
    void walle_eyes_setBrightness(float level) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* eyes = board->GetWalleEyes();
        if (eyes) eyes->SetBrightness(level);
    }
    void walle_eyes_setMode(const char* mode_str) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* eyes = board->GetWalleEyes();
        if (!eyes) return;
        if (strcmp(mode_str, "on") == 0)      eyes->SetMode(Gc9a01Eyes::kOn);
        else if (strcmp(mode_str, "off") == 0)     eyes->SetMode(Gc9a01Eyes::kOff);
        else if (strcmp(mode_str, "breathe") == 0) eyes->SetMode(Gc9a01Eyes::kBreathe);
        else if (strcmp(mode_str, "blink") == 0)   eyes->SetMode(Gc9a01Eyes::kBlink);
        else if (strcmp(mode_str, "angry") == 0)   eyes->SetMode(Gc9a01Eyes::kAngry);
        else if (strcmp(mode_str, "sleepy") == 0)  eyes->SetMode(Gc9a01Eyes::kSleepy);
    }

    // 延迟启动眼睛屏(连接 GC9A01 后调用,通过 MCP/debug API)
    void walle_eyes_start() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        board->StartEyes();
    }
    // 表情控制(直接调用 PCA9685)
    void walle_expression_playHappy() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* expr = board->GetWalleExpression();
        if (expr) expr->SetExpression(WalleExpression::kHappy);
    }
    void walle_expression_playSad() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* expr = board->GetWalleExpression();
        if (expr) expr->SetExpression(WalleExpression::kSad);
    }
    void walle_expression_playSurprised() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* expr = board->GetWalleExpression();
        if (expr) expr->SetExpression(WalleExpression::kScared);
    }
    void walle_expression_playAngry() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* expr = board->GetWalleExpression();
        if (expr) {
            expr->SetExpression(WalleExpression::kScared);
            auto* eyes = board->GetWalleEyes();
            if (eyes) { eyes->SetColor(Gc9a01Eyes::Color::Red()); eyes->SetMode(Gc9a01Eyes::kAngry); }
        }
    }
    void walle_expression_playSleepy() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* expr = board->GetWalleExpression();
        if (expr) expr->SetExpression(WalleExpression::kSad);
    }
    void walle_expression_playWave() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* expr = board->GetWalleExpression();
        if (expr) expr->SetExpression(WalleExpression::kWave);
    }

    // 舵机控制(9个舵机:CH0-CH8)
    void walle_servo_setAngle(int channel, int angle) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* pca = board->GetPca9685();
        if (pca && channel >= 0 && channel < SERVO_COUNT) {
            pca->SetServoAngle(channel, angle);
        }
    }

    // 电机控制(速度 -255~255)
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

    // 读取距离(毫米)
    uint16_t walle_vl53l0x_readRange() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* tof = board->GetVl53l0x();
        if (tof) return (uint16_t)tof->ReadDistanceMm();
        return 0;
    }

    // 外接音频控制(PCM5102 + INMP441, I2S1)
    void walle_ext_audio_play_beep() {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* audio = board->GetExternalAudio();
        if (!audio) return;
        static const int16_t kSine24[] = {
            0,8480,16383,23169,28377,31650,32767,31650,28377,23169,
            16383,8480,0,-8480,-16383,-23169,-28377,-31650,-32767,
            -31650,-28377,-23169,-16383,-8480
        };
        static int16_t beep[480];
        static bool init = false;
        if (!init) {
            for (int i = 0; i < 480; i++) beep[i] = kSine24[i % 24];
            init = true;
        }
        audio->PlayPcmAsync(beep, 480);
    }

    void walle_ext_audio_set_volume(int volume) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* audio = board->GetExternalAudio();
        if (audio) audio->SetOutputVolume(volume);
    }

    int walle_ext_audio_read_mic(int16_t* buffer, int max_samples) {
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());
        auto* audio = board->GetExternalAudio();
        if (!audio || !buffer) return 0;
        return audio->ReadMic(buffer, static_cast<size_t>(max_samples), 100);
    }

    // 系统重启
    void walle_system_restart() {
        esp_restart();
    }

    // I2C 总线扫描(只扫描 I2C1 外部总线, [v10] 已删除 I2C0)
    // 注意：返回 static buffer，调用者应立即复制结果，不要跨线程持有指针
    char* walle_i2c_scan_json() {
        static char result[4096];
        static std::mutex scan_mutex;
        std::lock_guard<std::mutex> lock(scan_mutex);
        result[0] = '\0';
        auto* board = static_cast<atk_dnesp32s3*>(&Board::GetInstance());

        char* p = result;
        p += snprintf(p, sizeof(result) - (p - result), "{");

        // 只扫描 I2C1 (外部总线)
        auto* bus1 = board->GetI2cBusExt();
        if (bus1) {
            p += snprintf(p, sizeof(result) - (p - result), "\"i2c1\":[");
            bool first = true;
            for (int addr = 1; addr < 127; addr++) {
                i2c_device_config_t dev_cfg = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = (uint8_t)addr,
                    .scl_speed_hz = 100000,
                };
                i2c_master_dev_handle_t dev = nullptr;
                esp_err_t ret = i2c_master_bus_add_device(bus1, &dev_cfg, &dev);
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
            p += snprintf(p, sizeof(result) - (p - result), "]");
        }

        p += snprintf(p, sizeof(result) - (p - result), "}");
        return result;
    }
}

DECLARE_BOARD(atk_dnesp32s3);

