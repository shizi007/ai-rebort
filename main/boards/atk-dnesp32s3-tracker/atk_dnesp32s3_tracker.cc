/*
 * 正点原子 ATK-DNESP32S3 人物追踪版
 * 
 * 基于基础版 atk_dnesp32s3，新增：
 *   - PanTilt 二自由度云台舵机驱动
 *   - PersonTracker 人物追踪器（拍照→AI分析→云台跟踪）
 *   - 6 个 MCP 工具，支持语音控制
 * 
 * 硬件：ATK-DNESP32S3 + OV2640 + SG90双舵机云台
 * 舵机接线：Pan=GPIO2, Tilt=GPIO8（独立5V供电）
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
#include "mcp_server.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <cmath>

#define TAG "atk_dnesp32s3_tracker"

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

// ============ 人物追踪器 ============
class PersonTracker {
public:
    struct TrackResult {
        bool person_found = false;
        float center_x = 0.5f;   // 归一化 0.0~1.0
        float center_y = 0.5f;
        float confidence = 0.0f;
    };

    PersonTracker(PanTilt* pan_tilt, Camera* camera)
        : pan_tilt_(pan_tilt), camera_(camera) {}

    ~PersonTracker() {
        StopTracking();
    }

    /**
     * 单次追踪流程：拍照 → AI分析人物位置 → 驱动云台
     * @return 是否检测到人物
     */
    bool TrackOnce() {
        if (!camera_->Capture()) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }

        // 利用项目已有的 Explain 机制，向云端AI询问人物位置
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

        if (track_result.person_found && track_result.confidence > 0.3f) {
            AdjustPanTilt(track_result);
            ESP_LOGI(TAG, "Person found at (%.2f, %.2f), conf=%.2f, pan=%.1f°, tilt=%.1f°",
                     track_result.center_x, track_result.center_y,
                     track_result.confidence,
                     pan_tilt_->pan_angle(), pan_tilt_->tilt_angle());
        } else {
            ESP_LOGI(TAG, "No person detected");
        }

        last_result_ = track_result;
        return track_result.person_found;
    }

    /**
     * 启动持续追踪
     * @param interval_ms 追踪间隔（毫秒）
     */
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
                    ESP_LOGI(TAG, "Person found, sweep stopped");
                }
            } else {
                no_person_count_++;
                // 连续 N 次未检测到人，进入扫描模式
                if (no_person_count_ >= 3 && !pan_tilt_->IsSweeping()) {
                    ESP_LOGI(TAG, "Person lost, starting sweep");
                    pan_tilt_->StartSweep();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(tracking_interval_ms_));
        }
        vTaskDelete(NULL);
    }

    TrackResult ParseTrackResult(const std::string& response) {
        TrackResult result;

        // 从AI回复中提取JSON
        auto json_start = response.find('{');
        auto json_end = response.rfind('}');
        if (json_start == std::string::npos || json_end == std::string::npos) {
            ESP_LOGW(TAG, "No JSON found in AI response");
            return result;
        }

        std::string json_str = response.substr(json_start, json_end - json_start + 1);
        cJSON* root = cJSON_Parse(json_str.c_str());
        if (!root) {
            ESP_LOGW(TAG, "JSON parse failed: %s", json_str.c_str());
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

        ESP_LOGI(TAG, "Track result: found=%d, pos=(%.2f,%.2f), conf=%.2f",
                 result.person_found, result.center_x, result.center_y, result.confidence);
        return result;
    }

    /**
     * 根据人物在画面中的位置调整云台
     * 比例控制器：人物偏离画面中心越多，云台转动幅度越大
     */
    void AdjustPanTilt(const TrackResult& result) {
        // 计算人物偏离画面中心的归一化偏移
        float dx = result.center_x - 0.5f;  // 正值=人在右侧，需要向右转
        float dy = result.center_y - 0.5f;  // 正值=人在下方，需要向下转

        // 死区内不移动（防止抖动）
        if (std::abs(dx) < deadzone_) dx = 0;
        if (std::abs(dy) < deadzone_) dy = 0;

        // 比例控制：偏移量 × 增益 = 云台转动角度
        // dx=0.5 (人在画面最右) → pan_gain_=60 → 转动30°
        float pan_delta = dx * pan_gain_;
        float tilt_delta = -dy * tilt_gain_;  // Y轴取反：画面上方=抬头=角度增大

        pan_tilt_->PanBy(pan_delta);
        pan_tilt_->TiltBy(tilt_delta);
    }

    PanTilt* pan_tilt_;
    Camera* camera_;
    bool tracking_ = false;
    int tracking_interval_ms_ = 2000;
    int no_person_count_ = 0;
    float deadzone_ = 0.1f;      // 10% 死区
    float pan_gain_ = 60.0f;     // 水平比例增益
    float tilt_gain_ = 40.0f;    // 垂直比例增益
    TaskHandle_t track_task_ = nullptr;
    TrackResult last_result_;
};

// ============ 板型定义 ============
class atk_dnesp32s3_tracker : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_;
    XL9555* xl9555_;
    EspVideo* camera_;
    PanTilt* pan_tilt_;
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

        // Initialize XL9555
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

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = LCD_CS_PIN;
        io_config.dc_gpio_num = LCD_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 7;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io);

        ESP_LOGD(TAG, "Install LCD driver");
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
        xl9555_->SetOutputState(OV_PWDN_IO, 0);  // PWDN=低 (上电)
        xl9555_->SetOutputState(OV_RESET_IO, 0);  // 确保复位
        vTaskDelay(pdMS_TO_TICKS(50));
        xl9555_->SetOutputState(OV_RESET_IO, 1);  // 释放复位
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

    /** 初始化云台舵机 */
    void InitializePanTilt() {
        PanTilt::Config cfg = {
            .pan_gpio = PAN_SERVO_GPIO,             // GPIO_2
            .tilt_gpio = TILT_SERVO_GPIO,           // GPIO_8
            .pan_channel = PAN_SERVO_LEDC_CHANNEL,   // LEDC_CHANNEL_1
            .tilt_channel = TILT_SERVO_LEDC_CHANNEL,  // LEDC_CHANNEL_2
            .timer = SERVO_LEDC_TIMER,              // LEDC_TIMER_1
            .min_angle = SERVO_MIN_ANGLE,
            .max_angle = SERVO_MAX_ANGLE,
            .center_angle = SERVO_CENTER_ANGLE,
        };
        pan_tilt_ = new PanTilt(cfg);
        ESP_LOGI(TAG, "PanTilt initialized");
    }

    /** 初始化人物追踪器 */
    void InitializeTracker() {
        tracker_ = new PersonTracker(pan_tilt_, camera_);
        ESP_LOGI(TAG, "PersonTracker initialized");
    }

    /** 注册 MCP 工具，支持语音控制 */
    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        // ---- 追踪控制 ----

        // 语音："跟着我" / "追踪" / "跟踪"
        mcp.AddTool("self.tracker.start",
            "开始追踪人物。摄像头会自动跟随检测到的人物移动。当找不到人时，云台会自动左右扫描搜索。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                tracker_->StartTracking(2000);
                return true;
            });

        // 语音："停止追踪" / "别跟了"
        mcp.AddTool("self.tracker.stop",
            "停止追踪人物。云台回到居中位置。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                tracker_->StopTracking();
                pan_tilt_->Home();
                return true;
            });

        // 语音："你看到谁了？" / "看看有没有人"
        mcp.AddTool("self.tracker.check",
            "拍照并检查当前画面中是否有人，返回人物位置信息。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                bool found = tracker_->TrackOnce();
                auto& r = tracker_->last_result();
                cJSON* json = cJSON_CreateObject();
                cJSON_AddBoolToObject(json, "person_found", found);
                cJSON_AddNumberToObject(json, "center_x", r.center_x);
                cJSON_AddNumberToObject(json, "center_y", r.center_y);
                cJSON_AddNumberToObject(json, "confidence", r.confidence);
                cJSON_AddNumberToObject(json, "pan_angle", pan_tilt_->pan_angle());
                cJSON_AddNumberToObject(json, "tilt_angle", pan_tilt_->tilt_angle());
                return json;
            });

        // ---- 云台控制 ----

        // 语音："看左边" / "看右边" / "看上面" / "看下面"
        mcp.AddTool("self.tracker.look_at",
            "将摄像头指向指定方向。pan控制左右(0=最左,90=正中,180=最右)，tilt控制上下(0=最下,90=正中,180=最上)。",
            PropertyList({
                Property("pan", kPropertyTypeInteger, 90, 0, 180),
                Property("tilt", kPropertyTypeInteger, 90, 0, 180)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                float pan = (float)props["pan"].value<int>();
                float tilt = (float)props["tilt"].value<int>();
                pan_tilt_->SmoothMoveTo(pan, tilt);
                return true;
            });

        // 语音："扫描一下" / "找找人"
        mcp.AddTool("self.tracker.sweep",
            "云台左右扫描搜索人物。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                pan_tilt_->StartSweep();
                return true;
            });

        // 语音："回正" / "看前面"
        mcp.AddTool("self.tracker.home",
            "云台回到居中位置。",
            PropertyList(),
            [this](const PropertyList& props) -> ReturnValue {
                pan_tilt_->Home();
                return true;
            });

        ESP_LOGI(TAG, "MCP tools registered (6 tracker tools)");
    }

public:
    atk_dnesp32s3_tracker()
        : boot_button_(BOOT_BUTTON_GPIO)
        , display_(nullptr)
        , xl9555_(nullptr)
        , camera_(nullptr)
        , pan_tilt_(nullptr)
        , tracker_(nullptr) {

        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeButtons();
        InitializeCamera();
        InitializePanTilt();
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
};

DECLARE_BOARD(atk_dnesp32s3_tracker);
