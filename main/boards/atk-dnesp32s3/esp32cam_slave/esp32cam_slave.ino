/**
 * WALL-E ESP32-CAM 子板固件
 * 
 * 功能：通过 UART 接收主板拍照指令，使用板载 OV2640 拍照，JPEG 通过 UART 回传
 * 
 * 通信协议（与 UartCamera 配合）：
 *   帧头: 0xAA 0x55
 *   命令: 1 byte
 *   数据长度: 4 bytes (LE)
 *   数据: N bytes
 *   帧尾: 0x0D 0x0A
 * 
 * 接线：
 *   ESP32-CAM        →  主板
 *   TX (GPIO1)       →  RX (GPIO17, 主板 UART2 RX)
 *   RX (GPIO3)       →  TX (GPIO16, 主板 UART2 TX)
 *   GND              →  GND
 *   5V               →  5V (来自 DC-DC 降压模块)
 * 
 * 烧录方式：FTDI/USB-TTL 连接 ESP32-CAM 的 U0TXD/U0RXD，按 IO0 后上电进入下载模式
 * 
 * 依赖库：esp_camera (ESP32 Arduino Core 自带)
 */

#include <Arduino.h>
#include "esp_camera.h"

// ============ 引脚定义 (AI-Thinker ESP32-CAM) ============
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============ UART 配置 ============
#define UART_BAUD       921600
#define SERIAL_UART     Serial1  // 使用 UART1，不影响下载串口

// ============ 协议常量 ============
#define FRAME_HEADER_0  0xAA
#define FRAME_HEADER_1  0x55
#define FRAME_FOOTER_0  0x0D
#define FRAME_FOOTER_1  0x0A

#define CMD_CAPTURE_REQUEST  0x01
#define CMD_JPEG_DATA        0x02
#define CMD_ACK              0x03
#define CMD_NACK             0x04
#define CMD_SET_MIRROR       0x10
#define CMD_SET_VFLIP        0x11

// ============ 全局变量 ============
bool h_mirror = false;
bool v_flip = false;

// ============ 摄像头配置 ============
camera_config_t camera_config = {
    .pin_pwdn     = PWDN_GPIO_NUM,
    .pin_reset    = RESET_GPIO_NUM,
    .pin_xclk     = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,
    .pin_d7       = Y9_GPIO_NUM,
    .pin_d6       = Y8_GPIO_NUM,
    .pin_d5       = Y7_GPIO_NUM,
    .pin_d4       = Y6_GPIO_NUM,
    .pin_d3       = Y5_GPIO_NUM,
    .pin_d2       = Y4_GPIO_NUM,
    .pin_d1       = Y3_GPIO_NUM,
    .pin_d0       = Y2_GPIO_NUM,
    .pin_vsync    = VSYNC_GPIO_NUM,
    .pin_href     = HREF_GPIO_NUM,
    .pin_pclk     = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size   = FRAMESIZE_QVGA,   // 320x240 (足够AI分析，传输快)
    .jpeg_quality = 63,                // 0-63, 越低质量越高(文件越大)
    .fb_count     = 2,                 // 双缓冲
    .grab_mode    = CAMERA_GRAB_LATEST // 始终取最新帧
};

// ============ 协议函数 ============

void sendFrame(uint8_t cmd, const uint8_t* data = nullptr, uint32_t data_len = 0) {
    uint8_t header[8];
    header[0] = FRAME_HEADER_0;
    header[1] = FRAME_HEADER_1;
    header[2] = cmd;
    header[3] = (data_len >> 0) & 0xFF;
    header[4] = (data_len >> 8) & 0xFF;
    header[5] = (data_len >> 16) & 0xFF;
    header[6] = (data_len >> 24) & 0xFF;
    header[7] = 0;

    SERIAL_UART.write(header, 8);
    if (data_len > 0 && data != nullptr) {
        SERIAL_UART.write(data, data_len);
    }
    uint8_t footer[2] = {FRAME_FOOTER_0, FRAME_FOOTER_1};
    SERIAL_UART.write(footer, 2);
    SERIAL_UART.flush();
}

bool receiveCommand(uint8_t& cmd, uint8_t*& data, uint32_t& data_len, uint32_t timeout_ms) {
    cmd = 0;
    data = nullptr;
    data_len = 0;

    // 同步帧头
    uint8_t b = 0;
    int phase = 0;
    unsigned long start = millis();

    while (millis() - start < timeout_ms) {
        if (SERIAL_UART.available()) {
            b = SERIAL_UART.read();
            if (phase == 0 && b == FRAME_HEADER_0) {
                phase = 1;
            } else if (phase == 1 && b == FRAME_HEADER_1) {
                break;
            } else {
                phase = (b == FRAME_HEADER_0) ? 1 : 0;
            }
        }
    }
    if (phase != 1) return false;

    // 命令字节
    if (!SERIAL_UART.available()) return false;
    cmd = SERIAL_UART.read();

    // 数据长度 4 bytes
    uint8_t len_bytes[4];
    for (int i = 0; i < 4; i++) {
        unsigned long t0 = millis();
        while (!SERIAL_UART.available() && millis() - t0 < 1000);
        if (!SERIAL_UART.available()) return false;
        len_bytes[i] = SERIAL_UART.read();
    }
    data_len = len_bytes[0] | (len_bytes[1] << 8) | (len_bytes[2] << 16) | (len_bytes[3] << 24);

    // 读取数据
    if (data_len > 0 && data_len < 1024) {
        data = new uint8_t[data_len];
        uint32_t received = 0;
        while (received < data_len) {
            if (SERIAL_UART.available()) {
                data[received++] = SERIAL_UART.read();
            }
        }
    }

    // 帧尾
    uint8_t ftr[2];
    for (int i = 0; i < 2; i++) {
        unsigned long t0 = millis();
        while (!SERIAL_UART.available() && millis() - t0 < 1000);
        if (!SERIAL_UART.available()) return false;
        ftr[i] = SERIAL_UART.read();
    }
    if (ftr[0] != FRAME_FOOTER_0 || ftr[1] != FRAME_FOOTER_1) {
        if (data) { delete[] data; data = nullptr; }
        return false;
    }

    return true;
}

void handleCapture() {
    // 获取一帧
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE("CAM", "Capture failed");
        sendFrame(CMD_NACK);
        return;
    }

    // 发送 JPEG 数据
    sendFrame(CMD_JPEG_DATA, fb->buf, fb->len);
    esp_camera_fb_return(fb);

    // 等待 ACK
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
        if (SERIAL_UART.available() >= 10) {  // 至少一个完整最小帧
            uint8_t ack_cmd = 0;
            uint8_t* ack_data = nullptr;
            uint32_t ack_len = 0;
            if (receiveCommand(ack_cmd, ack_data, ack_len, 100)) {
                if (ack_data) delete[] ack_data;
                break;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);  // 调试串口
    SERIAL_UART.begin(UART_BAUD, SERIAL_8N1, 3, 1);  // RX=GPIO3, TX=GPIO1

    // 初始化摄像头
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        // 闪 LED 指示错误
        pinMode(33, OUTPUT);
        while (1) {
            digitalWrite(33, !digitalRead(33));
            delay(200);
        }
    }

    // 丢弃前几帧（自动曝光校准）
    for (int i = 0; i < 5; i++) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
    }

    Serial.println("WALL-E ESP32-CAM slave ready");
}

void loop() {
    if (SERIAL_UART.available() >= 8) {  // 最小帧大小
        uint8_t cmd = 0;
        uint8_t* data = nullptr;
        uint32_t data_len = 0;

        if (receiveCommand(cmd, data, data_len, 1000)) {
            switch (cmd) {
                case CMD_CAPTURE_REQUEST:
                    handleCapture();
                    break;

                case CMD_SET_MIRROR:
                    if (data && data_len >= 1) {
                        h_mirror = (data[0] != 0);
                        sensor_t* s = esp_camera_sensor_get();
                        if (s) s->set_hmirror(s, h_mirror);
                        sendFrame(CMD_ACK);
                    }
                    break;

                case CMD_SET_VFLIP:
                    if (data && data_len >= 1) {
                        v_flip = (data[0] != 0);
                        sensor_t* s = esp_camera_sensor_get();
                        if (s) s->set_vflip(s, v_flip);
                        sendFrame(CMD_ACK);
                    }
                    break;

                default:
                    sendFrame(CMD_NACK);
                    break;
            }
            if (data) delete[] data;
        }
    }
    delay(1);
}
