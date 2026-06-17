#ifndef _EXTERNAL_AUDIO_H_
#define _EXTERNAL_AUDIO_H_

#include "audio_codec.h"
#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <vector>
#include <functional>
#include <mutex>

// ============================================================================
// ExternalAudio — I2S1 全双工外接音频 (PCM5102 DAC + INMP441 麦克风)
// ============================================================================
//
// [v10] 升级：实现 AudioCodec 接口，替代板载 ES8388
// 用途：主音频输出 (PCM5102 → PAM8406 → 2828喇叭) + 主麦克风输入 (INMP441)
//
// I2S1 GPIO 分配：
//   BCLK = GPIO15, WS = GPIO18, DOUT = GPIO45 (→ PCM5102 DIN), DIN = GPIO47 (← INMP441 SD)
//
// PCM5102 无需 I2C 控制（硬件自启动），只需 I2S 数据线。
// INMP441 无需 I2C 控制，只需 I2S 时钟+数据线。
//
// AudioCodec 接口实现：
//   - Write(): 通过 I2S1 TX 发送 32-bit 扩展音频数据 → PCM5102
//   - Read():  通过 I2S1 RX 读取 INMP441 麦克风数据
//   - SetOutputVolume(): 软件音量控制
//   - SetInputGain(): 麦克风增益控制
//   - EnableInput/EnableOutput(): 通道开关
// ============================================================================

class ExternalAudio : public AudioCodec {
public:
    // 音效播放完成回调
    using PlayDoneCallback = std::function<void()>;

    struct Config {
        gpio_num_t bclk_pin;
        gpio_num_t ws_pin;
        gpio_num_t dout_pin;     // → PCM5102 DIN
        gpio_num_t din_pin;      // ← INMP441 SD
        int sample_rate;
        int output_channels;     // PCM5102 立体声=2
        int input_channels;      // INMP441 单声道=1
    };

    ExternalAudio();
    virtual ~ExternalAudio();

    /**
     * 初始化 I2S1 全双工通道（AudioCodec 接口需要）
     * @return ESP_OK 成功
     */
    esp_err_t Initialize(const Config& config);

    // ---- AudioCodec 接口实现 ----
    virtual void SetOutputVolume(int volume) override;
    virtual void SetInputGain(float gain) override;
    virtual void EnableInput(bool enable) override;
    virtual void EnableOutput(bool enable) override;

protected:
    // AudioCodec 纯虚函数实现
    virtual int Read(int16_t* dest, int samples) override;
    virtual int Write(const int16_t* data, int samples) override;

public:
    // ---- 音效播放扩展接口（不经过 AudioCodec，直接 I2S 写入） ----

    /**
     * 播放 PCM 数据 (16-bit, sample_rate 采样率)
     * 会阻塞直到播放完成
     */
    esp_err_t PlayPcm(const int16_t* data, size_t samples);

    /**
     * 异步播放 PCM 数据（在后台任务中播放）
     */
    esp_err_t PlayPcmAsync(const int16_t* data, size_t samples, PlayDoneCallback callback = nullptr);

    /**
     * 停止当前播放
     */
    void StopPlayback();

    /**
     * 读取麦克风数据 (非阻塞, 返回实际读取的采样数)
     */
    int ReadMic(int16_t* buffer, size_t max_samples, int timeout_ms = 100);

    bool IsPlaying() const { return is_playing_; }
    bool IsInitialized() const { return initialized_; }

private:
    Config config_;
    bool initialized_ = false;
    bool is_playing_ = false;
    std::mutex data_if_mutex_;

    // 预分配 PSRAM 缓冲区，避免实时音频路径上堆分配导致爆音
    static constexpr int kMaxFrameSamples = 960;  // 40ms @ 24kHz，覆盖所有回调帧大小
    int32_t* write_buf_ = nullptr;   // Write() 32-bit 扩展缓冲
    int32_t* read_buf_ = nullptr;    // Read() 32-bit 读取缓冲
    int32_t* mic_buf_ = nullptr;     // ReadMic() 32-bit 读取缓冲

    TaskHandle_t play_task_ = nullptr;
    QueueHandle_t play_queue_ = nullptr;

    void PlayTaskFunc();

    // 简单的音量缩放（用于音效播放路径，不经过 AudioCodec Write）
    void ApplyVolumeEffect(int16_t* data, size_t samples);
    // 麦克风增益（用于 ReadMic 快捷路径）
    void ApplyMicGainDirect(int16_t* data, size_t samples);

    static void PlayTaskEntry(void* arg);
};

#endif // _EXTERNAL_AUDIO_H_
