#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>

#include "camera.h"
#include <driver/uart.h>

/**
 * UartCamera — 通过 UART 连接独立 ESP32-CAM 子板的 Camera 实现
 *
 * 通信协议（简单二进制帧格式）：
 *   帧头: 0xAA 0x55
 *   命令: 1 byte (0x01=拍照请求, 0x02=JPEG数据, 0x03=ACK, 0x04=NACK, 0x10=设置镜像, 0x11=设置翻转)
 *   数据长度: 4 bytes (little-endian uint32)
 *   数据: N bytes
 *   帧尾: 0x0D 0x0A
 *
 * 工作流程：
 *   1. 主板发送拍照请求 (CMD=0x01)
 *   2. ESP32-CAM 拍照后回传 JPEG (CMD=0x02)
 *   3. 主板收到完整 JPEG 后用于 AI 分析
 *
 * UART 参数: 921600 baud, 8N1（高速传输 JPEG）
 */
class UartCamera : public Camera {
public:
    struct Config {
        uart_port_t uart_port = UART_NUM_2;
        int tx_pin = -1;
        int rx_pin = -1;
        int baud_rate = 921600;
        int rx_buffer_size = 1024 * 64;   // 64KB 接收缓冲
        int tx_buffer_size = 1024;
        int capture_timeout_ms = 5000;     // 拍照超时
    };

    explicit UartCamera(const Config& config);
    ~UartCamera();

    // Camera 接口实现
    void SetExplainUrl(const std::string& url, const std::string& token) override;
    bool Capture() override;
    bool SetHMirror(bool enabled) override;
    bool SetVFlip(bool enabled) override;
    std::string Explain(const std::string& question) override;

    // 诊断
    bool IsConnected() const { return connected_; }
    int GetLastFrameSize() const { return last_frame_size_; }

private:
    // 协议常量
    static constexpr uint8_t FRAME_HEADER_0 = 0xAA;
    static constexpr uint8_t FRAME_HEADER_1 = 0x55;
    static constexpr uint8_t FRAME_FOOTER_0 = 0x0D;
    static constexpr uint8_t FRAME_FOOTER_1 = 0x0A;

    static constexpr uint8_t CMD_CAPTURE_REQUEST  = 0x01;
    static constexpr uint8_t CMD_JPEG_DATA        = 0x02;
    static constexpr uint8_t CMD_ACK              = 0x03;
    static constexpr uint8_t CMD_NACK             = 0x04;
    static constexpr uint8_t CMD_SET_MIRROR       = 0x10;
    static constexpr uint8_t CMD_SET_VFLIP        = 0x11;

    bool SendCommand(uint8_t cmd, const uint8_t* data = nullptr, uint32_t data_len = 0);
    bool ReceiveFrame(uint8_t& cmd, std::vector<uint8_t>& payload, int timeout_ms);
    bool WaitAck(int timeout_ms = 2000);

    Config config_;
    uart_port_t uart_port_;
    bool initialized_ = false;
    bool connected_ = false;
    bool h_mirror_ = false;
    bool v_flip_ = false;

    std::string explain_url_;
    std::string explain_token_;

    // 当前帧数据
    std::vector<uint8_t> frame_buffer_;
    int last_frame_size_ = 0;
    std::mutex frame_mutex_;
};
