#include "external_audio.h"
#include <esp_log.h>
#include <esp_err.h>
#include <cstring>
#include <memory>
#include <algorithm>
#include <cmath>
#include <esp_heap_caps.h>

static const char TAG[] = "ExternalAudio";

// 播放队列元素
struct PlayItem {
    std::vector<int16_t> data;
    ExternalAudio::PlayDoneCallback callback;
};

ExternalAudio::ExternalAudio() = default;

ExternalAudio::~ExternalAudio() {
    StopPlayback();
    if (play_queue_) {
        vQueueDelete(play_queue_);
        play_queue_ = nullptr;
    }
    // AudioCodec 基类的 tx_handle_ / rx_handle_ 由我们管理
    if (rx_handle_) {
        i2s_channel_disable(rx_handle_);
        i2s_del_channel(rx_handle_);
        rx_handle_ = nullptr;
    }
    if (tx_handle_) {
        i2s_channel_disable(tx_handle_);
        i2s_del_channel(tx_handle_);
        tx_handle_ = nullptr;
    }
    // 释放预分配 PSRAM 缓冲
    if (write_buf_) { heap_caps_free(write_buf_); write_buf_ = nullptr; }
    if (read_buf_)  { heap_caps_free(read_buf_);  read_buf_ = nullptr; }
    if (mic_buf_)   { heap_caps_free(mic_buf_);   mic_buf_ = nullptr; }
    ESP_LOGI(TAG, "ExternalAudio destroyed");
}

esp_err_t ExternalAudio::Initialize(const Config& config) {
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    config_ = config;

    // 设置 AudioCodec 基类参数
    duplex_ = true;
    input_sample_rate_ = config_.sample_rate;
    output_sample_rate_ = config_.sample_rate;
    input_channels_ = config_.input_channels;
    output_channels_ = config_.output_channels;

    // ===== I2S1 全双工: 同时创建 TX + RX 通道 =====
    // 关键：ESP-IDF v5.x 全双工必须一次 i2s_new_channel 同时创建 TX 和 RX，
    // 否则第二次创建时 I2S 控制器已被占用，GPIO 会被锁死。
    // 参考 NoAudioCodecDuplex 的做法。
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_1,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_handle_, &rx_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Duplex channel create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ===== TX 初始化 (→ PCM5102) =====
    // 全双工 I2S: TX 和 RX 共享同一总线，slot 配置必须一致
    // PCM5102 支持左对齐/I2S 格式，INMP441 驱动左声道
    // 使用 MONO+LEFT（与 NoAudioCodecDuplex 一致），确保 RX 正确采集 INMP441 数据
    i2s_std_config_t tx_cfg = {};
    tx_cfg.clk_cfg = {
        .sample_rate_hz = static_cast<uint32_t>(config_.sample_rate),
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
#ifdef I2S_HW_VERSION_2
        .ext_clk_freq_hz = 0,
#endif
    };
    tx_cfg.slot_cfg = {
        .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
        .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
        .slot_mode = I2S_SLOT_MODE_MONO,
        .slot_mask = I2S_STD_SLOT_LEFT,
        .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
        .ws_pol = false,
        .bit_shift = true,
#ifdef I2S_HW_VERSION_2
        .left_align = true,
        .big_endian = false,
        .bit_order_lsb = false,
#endif
    };
    tx_cfg.gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = config_.bclk_pin,
        .ws   = config_.ws_pin,
        .dout = config_.dout_pin,
        .din  = config_.din_pin,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv   = false,
        },
    };
    ret = i2s_channel_init_std_mode(tx_handle_, &tx_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TX init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ===== RX 初始化 (← INMP441) =====
    // 全双工: RX 共享 TX 的 BCLK/WS，slot 配置必须与 TX 一致
    // INMP441 驱动左声道，使用 MONO+LEFT
    i2s_std_config_t rx_cfg = {};
    rx_cfg.clk_cfg = {
        .sample_rate_hz = static_cast<uint32_t>(config_.sample_rate),
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
#ifdef I2S_HW_VERSION_2
        .ext_clk_freq_hz = 0,
#endif
    };
    rx_cfg.slot_cfg = {
        .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
        .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
        .slot_mode = I2S_SLOT_MODE_MONO,
        .slot_mask = I2S_STD_SLOT_LEFT,
        .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
        .ws_pol = false,
        .bit_shift = true,
#ifdef I2S_HW_VERSION_2
        .left_align = true,
        .big_endian = false,
        .bit_order_lsb = false,
#endif
    };
    rx_cfg.gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = config_.bclk_pin,
        .ws   = config_.ws_pin,
        .dout = config_.dout_pin,
        .din  = config_.din_pin,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv   = false,
        },
    };
    ret = i2s_channel_init_std_mode(rx_handle_, &rx_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RX init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 全双工: 不在 Initialize 中 enable 通道，由 AudioService 通过 EnableInput/EnableOutput 控制
    // 与 NoAudioCodecDuplex 保持一致（构造函数只创建+init，不 enable）
    // 注意：如果 RX 不 enable，Read() 会返回 0，这是正常的——等 AudioService 调用 EnableInput(true) 后才开始读数据

    // 预分配 PSRAM 缓冲区（避免 Write/Read/ReadMic 每次堆分配）
    size_t buf_sz = kMaxFrameSamples * sizeof(int32_t);
    write_buf_ = (int32_t*)heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    read_buf_  = (int32_t*)heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    mic_buf_   = (int32_t*)heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!write_buf_ || !read_buf_ || !mic_buf_) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM audio buffers");
        return ESP_ERR_NO_MEM;
    }

    // 创建播放队列
    play_queue_ = xQueueCreate(4, sizeof(PlayItem*));
    if (!play_queue_) {
        ESP_LOGE(TAG, "Failed to create play queue");
        return ESP_ERR_NO_MEM;
    }

    // input_enabled_ / output_enabled_ 保持 false（基类默认值）
    // AudioService::ReadAudioData() 会调用 EnableInput(true) -> i2s_channel_enable(rx)
    // AudioService::AudioOutputTask() 会调用 EnableOutput(true) -> i2s_channel_enable(tx)
    initialized_ = true;
    ESP_LOGI(TAG, "ExternalAudio(AudioCodec) initialized: BCLK=%d, WS=%d, DOUT=%d, DIN=%d, SR=%d, OutCh=%d, InCh=%d",
             config_.bclk_pin, config_.ws_pin, config_.dout_pin, config_.din_pin,
             config_.sample_rate, config_.output_channels, config_.input_channels);
    return ESP_OK;
}

// ============================================================================
// AudioCodec 接口实现
// ============================================================================

int ExternalAudio::Write(const int16_t* data, int samples) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (!tx_handle_) return 0;

    // 分帧写入，每帧不超过 kMaxFrameSamples
    int32_t volume_factor = (int32_t)(pow((double)output_volume_ / 100.0, 2) * 65536);
    int written = 0;
    while (written < samples) {
        int frame = std::min(samples - written, kMaxFrameSamples);
        for (int i = 0; i < frame; i++) {
            int64_t temp = (int64_t)data[written + i] * volume_factor;
            if (temp > INT32_MAX) {
                write_buf_[i] = INT32_MAX;
            } else if (temp < INT32_MIN) {
                write_buf_[i] = INT32_MIN;
            } else {
                write_buf_[i] = static_cast<int32_t>(temp);
            }
        }

        size_t bytes_written;
        esp_err_t ret = i2s_channel_write(tx_handle_, write_buf_, frame * sizeof(int32_t), &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "AudioCodec Write failed: %s", esp_err_to_name(ret));
            return written;
        }
        written += bytes_written / sizeof(int32_t);
    }
    return written;
}

int ExternalAudio::Read(int16_t* dest, int samples) {
    if (!rx_handle_) return 0;

    // 从 I2S 读取 32-bit 数据，转为 16-bit
    // 使用预分配 PSRAM 缓冲，避免实时路径堆分配
    constexpr uint32_t kReadTimeoutMs = 200;
    int total_read = 0;
    while (total_read < samples) {
        int frame = std::min(samples - total_read, kMaxFrameSamples);
        size_t bytes_read;
        esp_err_t ret = i2s_channel_read(rx_handle_, read_buf_, frame * sizeof(int32_t), &bytes_read, kReadTimeoutMs);
        if (ret != ESP_OK) {
            return total_read;
        }

        int samples_read = bytes_read / sizeof(int32_t);
        for (int i = 0; i < samples_read; i++) {
            int32_t value = read_buf_[i] >> 12;  // 32-bit → 16-bit 有效位
            dest[total_read + i] = (value > INT16_MAX) ? INT16_MAX : (value < -INT16_MAX) ? -INT16_MAX : (int16_t)value;
        }

        // 应用输入增益
        if (input_gain_ > 0) {
            int gain_factor = (int)input_gain_;
            for (int i = 0; i < samples_read; i++) {
                int32_t amplified = (int32_t)dest[total_read + i] * gain_factor;
                dest[total_read + i] = (amplified > INT16_MAX) ? INT16_MAX : (amplified < -INT16_MAX) ? -INT16_MAX : (int16_t)amplified;
            }
        }
        total_read += samples_read;
    }
    return total_read;
}

void ExternalAudio::SetOutputVolume(int volume) {
    output_volume_ = volume;
    ESP_LOGI(TAG, "Set output volume to %d", output_volume_);
}

void ExternalAudio::SetInputGain(float gain) {
    input_gain_ = gain;
    ESP_LOGI(TAG, "Set input gain to %.1f", input_gain_);
}

void ExternalAudio::EnableInput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == input_enabled_) return;
    if (rx_handle_) {
        if (enable) {
            i2s_channel_enable(rx_handle_);
        } else {
            i2s_channel_disable(rx_handle_);
        }
    }
    AudioCodec::EnableInput(enable);  // 更新基类标志
    ESP_LOGI(TAG, "Set input enable to %s", enable ? "true" : "false");
}

void ExternalAudio::EnableOutput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == output_enabled_) return;
    if (tx_handle_) {
        if (enable) {
            i2s_channel_enable(tx_handle_);
        } else {
            i2s_channel_disable(tx_handle_);
        }
    }
    AudioCodec::EnableOutput(enable);  // 更新基类标志
    ESP_LOGI(TAG, "Set output enable to %s", enable ? "true" : "false");
}

// ============================================================================
// 音效播放扩展接口
// ============================================================================

esp_err_t ExternalAudio::PlayPcm(const int16_t* data, size_t samples) {
    if (!initialized_ || !tx_handle_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (!data || samples == 0) return ESP_OK;

    std::lock_guard<std::mutex> lock(data_if_mutex_);

    // 拷贝数据并应用音量
    std::vector<int16_t> buf(samples);
    std::copy(data, data + samples, buf.begin());
    ApplyVolumeEffect(buf.data(), buf.size());

    is_playing_ = true;
    size_t offset = 0;
    while (offset < samples) {
        int frame = (int)std::min((size_t)(samples - offset), (size_t)kMaxFrameSamples);
        // 扩展为 32-bit 写入
        for (int i = 0; i < frame; i++) {
            write_buf_[i] = (int32_t)buf[offset + i] << 16;  // 16-bit → 32-bit 左对齐
        }
        size_t bytes_written = 0;
        esp_err_t ret = i2s_channel_write(tx_handle_, write_buf_, frame * sizeof(int32_t), &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            is_playing_ = false;
            ESP_LOGE(TAG, "TX write failed: %s", esp_err_to_name(ret));
            return ret;
        }
        offset += bytes_written / sizeof(int32_t);
    }
    is_playing_ = false;
    return ESP_OK;
}

void ExternalAudio::PlayTaskEntry(void* arg) {
    static_cast<ExternalAudio*>(arg)->PlayTaskFunc();
}

void ExternalAudio::PlayTaskFunc() {
    PlayItem* item = nullptr;
    while (true) {
        if (xQueueReceive(play_queue_, &item, portMAX_DELAY) != pdTRUE) continue;
        if (!item) break;  // nullptr = 退出信号

        std::lock_guard<std::mutex> lock(data_if_mutex_);
        is_playing_ = true;
        // 应用音量
        ApplyVolumeEffect(item->data.data(), item->data.size());

        // 分帧扩展为 32-bit 写入（使用预分配缓冲）
        size_t offset = 0;
        while (offset < item->data.size()) {
            int frame = (int)std::min(item->data.size() - offset, (size_t)kMaxFrameSamples);
            for (int i = 0; i < frame; i++) {
                write_buf_[i] = (int32_t)item->data[offset + i] << 16;
            }
            size_t bytes_written = 0;
            i2s_channel_write(tx_handle_, write_buf_,
                               frame * sizeof(int32_t),
                               &bytes_written, portMAX_DELAY);
            offset += bytes_written / sizeof(int32_t);
        }
        is_playing_ = false;

        if (item->callback) {
            item->callback();
        }
        delete item;
        item = nullptr;
    }
}

esp_err_t ExternalAudio::PlayPcmAsync(const int16_t* data, size_t samples, PlayDoneCallback callback) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (!data || samples == 0) return ESP_OK;

    // 懒创建播放任务
    if (!play_task_) {
        xTaskCreate(PlayTaskEntry, "ext_audio_play", 4096, this, 5, &play_task_);
    }

    auto* item = new PlayItem();
    item->data.assign(data, data + samples);
    item->callback = std::move(callback);

    if (xQueueSend(play_queue_, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        delete item;
        ESP_LOGE(TAG, "Play queue full");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void ExternalAudio::StopPlayback() {
    if (play_task_) {
        // 发送 NULL 信号让任务退出，等待最多 1 秒
        xQueueSend(play_queue_, nullptr, 0);
        for (int i = 0; i < 10; i++) {
            if (!play_task_) break;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (play_task_) {
            vTaskDelete(play_task_);
            ESP_LOGW(TAG, "PlayTask force-killed");
            play_task_ = nullptr;
        }
    }
    is_playing_ = false;
}

int ExternalAudio::ReadMic(int16_t* buffer, size_t max_samples, int timeout_ms) {
    if (!initialized_ || !rx_handle_) {
        ESP_LOGE(TAG, "RX not initialized");
        return 0;
    }
    if (!buffer || max_samples == 0) return 0;

    // 分帧读取，每帧不超过 kMaxFrameSamples
    size_t total_read = 0;
    TickType_t ticks = timeout_ms > 0 ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
    while (total_read < max_samples) {
        int frame = (int)std::min((size_t)(max_samples - total_read), (size_t)kMaxFrameSamples);
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(rx_handle_, mic_buf_, frame * sizeof(int32_t),
                                          &bytes_read, ticks);
        if (ret != ESP_OK) {
            if (ret != ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "RX read failed: %s", esp_err_to_name(ret));
            }
            return (int)total_read;
        }
        size_t samples_read = bytes_read / sizeof(int32_t);
        for (size_t i = 0; i < samples_read; i++) {
            int32_t value = mic_buf_[i] >> 12;
            buffer[total_read + i] = (value > INT16_MAX) ? INT16_MAX : (value < -INT16_MAX) ? -INT16_MAX : (int16_t)value;
        }
        total_read += samples_read;
    }
    ApplyMicGainDirect(buffer, total_read);
    return (int)total_read;
}

void ExternalAudio::ApplyVolumeEffect(int16_t* data, size_t samples) {
    if (output_volume_ == 100) return;
    float scale = output_volume_ / 100.0f;
    for (size_t i = 0; i < samples; ++i) {
        int32_t v = static_cast<int32_t>(data[i] * scale);
        data[i] = static_cast<int16_t>(std::clamp(v, static_cast<int32_t>(INT16_MIN), static_cast<int32_t>(INT16_MAX)));
    }
}

void ExternalAudio::ApplyMicGainDirect(int16_t* data, size_t samples) {
    if (std::abs(input_gain_ - 1.0f) < 0.01f && input_gain_ > 0) return;
    float gain = input_gain_ > 0 ? input_gain_ : 3.0f;
    for (size_t i = 0; i < samples; ++i) {
        int32_t v = static_cast<int32_t>(data[i] * gain);
        data[i] = static_cast<int16_t>(std::clamp(v, static_cast<int32_t>(INT16_MIN), static_cast<int32_t>(INT16_MAX)));
    }
}
