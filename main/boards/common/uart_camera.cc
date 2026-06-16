#include "uart_camera.h"
#include <cstring>
#include <vector>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "system_info.h"
#include <http.h>
#include <network_interface.h>

static const char* TAG = "UartCamera";

UartCamera::UartCamera(const Config& config)
    : config_(config), uart_port_(config.uart_port) {
    const uart_config_t uart_config = {
        .baud_rate = config_.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_param_config(uart_port_, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = uart_set_pin(uart_port_, config_.tx_pin, config_.rx_pin,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = uart_driver_install(uart_port_, config_.rx_buffer_size,
                              config_.tx_buffer_size, 0, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "UartCamera init OK: UART%d TX=%d RX=%d baud=%d",
             uart_port_, config_.tx_pin, config_.rx_pin, config_.baud_rate);
}

UartCamera::~UartCamera() {
    if (initialized_) {
        uart_driver_delete(uart_port_);
    }
}

void UartCamera::SetExplainUrl(const std::string& url, const std::string& token) {
    explain_url_ = url;
    explain_token_ = token;
}

bool UartCamera::SetHMirror(bool enabled) {
    h_mirror_ = enabled;
    uint8_t data[1] = { static_cast<uint8_t>(enabled ? 1 : 0) };
    bool ok = SendCommand(CMD_SET_MIRROR, data, 1) && WaitAck();
    if (ok) {
        ESP_LOGI(TAG, "H-mirror %s", enabled ? "ON" : "OFF");
    }
    return ok;
}

bool UartCamera::SetVFlip(bool enabled) {
    v_flip_ = enabled;
    uint8_t data[1] = { static_cast<uint8_t>(enabled ? 1 : 0) };
    bool ok = SendCommand(CMD_SET_VFLIP, data, 1) && WaitAck();
    if (ok) {
        ESP_LOGI(TAG, "V-flip %s", enabled ? "ON" : "OFF");
    }
    return ok;
}

bool UartCamera::Capture() {
    if (!initialized_) return false;

    // 发送拍照请求
    if (!SendCommand(CMD_CAPTURE_REQUEST)) {
        ESP_LOGW(TAG, "Send capture request failed");
        return false;
    }

    // 等待 JPEG 数据帧
    uint8_t cmd;
    std::vector<uint8_t> payload;
    if (!ReceiveFrame(cmd, payload, config_.capture_timeout_ms)) {
        ESP_LOGW(TAG, "Capture timeout or error");
        connected_ = false;
        return false;
    }

    if (cmd != CMD_JPEG_DATA) {
        ESP_LOGW(TAG, "Unexpected response: cmd=0x%02X (expected 0x02)", cmd);
        return false;
    }

    if (payload.size() < 100) {
        ESP_LOGW(TAG, "JPEG too small: %u bytes", (uint32_t)payload.size());
        return false;
    }

    // 验证 JPEG 起止标记
    if (payload[0] != 0xFF || payload[1] != 0xD8) {
        ESP_LOGW(TAG, "Invalid JPEG header: 0x%02X 0x%02X", payload[0], payload[1]);
        return false;
    }
    if (payload[payload.size()-2] != 0xFF || payload[payload.size()-1] != 0xD9) {
        ESP_LOGW(TAG, "Invalid JPEG footer: 0x%02X 0x%02X",
                 payload[payload.size()-2], payload[payload.size()-1]);
        return false;
    }

    // 存入帧缓冲
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_buffer_ = std::move(payload);
        last_frame_size_ = static_cast<int>(frame_buffer_.size());
    }

    connected_ = true;
    ESP_LOGI(TAG, "Capture OK: %d bytes", last_frame_size_);

    // 发送 ACK
    SendCommand(CMD_ACK);
    return true;
}

std::string UartCamera::Explain(const std::string& question) {
    if (explain_url_.empty()) {
        ESP_LOGE(TAG, "Explain: URL not set");
        return "{\"error\":\"no explain URL\"}";
    }

    // 读取当前帧 JPEG 数据
    std::vector<uint8_t> jpeg_data;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        jpeg_data = frame_buffer_;
    }

    if (jpeg_data.empty()) {
        ESP_LOGW(TAG, "Explain: no frame captured");
        return "{\"error\":\"no frame\"}";
    }

    // UartCamera 直接拿到 JPEG，无需编码，直接 HTTP 分块上传
    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);

    std::string boundary = "----WALLE_UART_CAMERA";

    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");

    if (!http->Open("POST", explain_url_)) {
        ESP_LOGE(TAG, "Failed to connect to explain URL");
        return "{\"error\":\"connection failed\"}";
    }

    // 第一块：question 字段
    {
        std::string field;
        field += "--" + boundary + "\r\n";
        field += "Content-Disposition: form-data; name=\"question\"\r\n";
        field += "\r\n";
        field += question + "\r\n";
        http->Write(field.c_str(), field.size());
    }

    // 第二块：文件字段头部
    {
        std::string header;
        header += "--" + boundary + "\r\n";
        header += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
        header += "Content-Type: image/jpeg\r\n";
        header += "\r\n";
        http->Write(header.c_str(), header.size());
    }

    // 第三块：JPEG 数据（分块发送，每块 4KB）
    const size_t CHUNK_SIZE = 4096;
    size_t total_sent = 0;
    while (total_sent < jpeg_data.size()) {
        size_t chunk_len = std::min(CHUNK_SIZE, jpeg_data.size() - total_sent);
        http->Write(reinterpret_cast<const char*>(jpeg_data.data() + total_sent), chunk_len);
        total_sent += chunk_len;
    }

    // 第四块：multipart 尾部
    {
        std::string footer;
        footer += "\r\n--" + boundary + "--\r\n";
        http->Write(footer.c_str(), footer.size());
    }
    http->Write("", 0);

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Upload failed, status: %d", http->GetStatusCode());
        http->Close();
        return "{\"error\":\"upload failed\"}";
    }

    std::string result = http->ReadAll();
    http->Close();

    ESP_LOGI(TAG, "Explain OK: image=%u bytes, response=%s",
             (uint32_t)jpeg_data.size(), result.c_str());
    return result;
}

// ============ UART 协议实现 ============

bool UartCamera::SendCommand(uint8_t cmd, const uint8_t* data, uint32_t data_len) {
    if (!initialized_) return false;

    uint8_t header[8];
    header[0] = FRAME_HEADER_0;     // 0xAA
    header[1] = FRAME_HEADER_1;     // 0x55
    header[2] = cmd;
    header[3] = (data_len >> 0) & 0xFF;
    header[4] = (data_len >> 8) & 0xFF;
    header[5] = (data_len >> 16) & 0xFF;
    header[6] = (data_len >> 24) & 0xFF;
    header[7] = 0;  // reserved

    int sent = uart_write_bytes(uart_port_, (const char*)header, 8);
    if (sent < 8) return false;

    if (data_len > 0 && data != nullptr) {
        sent = uart_write_bytes(uart_port_, (const char*)data, data_len);
        if (sent < 0) return false;
    }

    uint8_t footer[2] = {FRAME_FOOTER_0, FRAME_FOOTER_1};
    sent = uart_write_bytes(uart_port_, (const char*)footer, 2);
    return sent >= 0;
}

bool UartCamera::WaitAck(int timeout_ms) {
    uint8_t cmd;
    std::vector<uint8_t> payload;
    if (!ReceiveFrame(cmd, payload, timeout_ms)) return false;
    return cmd == CMD_ACK;
}

bool UartCamera::ReceiveFrame(uint8_t& cmd, std::vector<uint8_t>& payload, int timeout_ms) {
    cmd = 0;
    payload.clear();
    if (!initialized_) return false;

    // 1. 同步帧头 0xAA 0x55
    uint8_t b = 0;
    int phase = 0;  // 0=等待0xAA, 1=等待0x55
    int elapsed = 0;

    while (elapsed < timeout_ms) {
        int read = uart_read_bytes(uart_port_, &b, 1, 50 / portTICK_PERIOD_MS);
        if (read == 1) {
            if (phase == 0 && b == FRAME_HEADER_0) {
                phase = 1;
            } else if (phase == 1 && b == FRAME_HEADER_1) {
                break;  // 帧头同步成功
            } else {
                phase = (b == FRAME_HEADER_0) ? 1 : 0;
            }
        } else {
            elapsed += 50;
        }
    }
    if (elapsed >= timeout_ms) {
        ESP_LOGD(TAG, "Frame header sync timeout");
        return false;
    }

    // 2. 读取命令字节
    if (uart_read_bytes(uart_port_, &cmd, 1, 1000 / portTICK_PERIOD_MS) != 1) {
        ESP_LOGW(TAG, "Command byte timeout");
        return false;
    }

    // 3. 读取数据长度 (4 bytes LE)
    uint8_t len_bytes[4];
    if (uart_read_bytes(uart_port_, len_bytes, 4, 1000 / portTICK_PERIOD_MS) != 4) {
        ESP_LOGW(TAG, "Length bytes timeout");
        return false;
    }
    uint32_t data_len = len_bytes[0] | (len_bytes[1] << 8) |
                        (len_bytes[2] << 16) | (len_bytes[3] << 24);

    if (data_len > 200 * 1024) {
        ESP_LOGW(TAG, "Data length too large: %u", data_len);
        return false;
    }

    // 4. 读取 payload
    payload.resize(data_len);
    if (data_len > 0) {
        uint32_t received = 0;
        int retry = 0;
        while (received < data_len && retry < 100) {
            int chunk = uart_read_bytes(uart_port_, payload.data() + received,
                                        data_len - received,
                                        2000 / portTICK_PERIOD_MS);
            if (chunk <= 0) {
                retry++;
                continue;
            }
            received += chunk;
            retry = 0;
        }
        if (received != data_len) {
            ESP_LOGW(TAG, "Payload incomplete: %u/%u", received, data_len);
            return false;
        }
    }

    // 5. 读取帧尾 0x0D 0x0A
    uint8_t ftr[2];
    if (uart_read_bytes(uart_port_, ftr, 2, 1000 / portTICK_PERIOD_MS) != 2) {
        ESP_LOGW(TAG, "Frame footer timeout");
        return false;
    }
    if (ftr[0] != FRAME_FOOTER_0 || ftr[1] != FRAME_FOOTER_1) {
        ESP_LOGW(TAG, "Invalid footer: 0x%02X 0x%02X", ftr[0], ftr[1]);
        return false;
    }

    return true;
}
