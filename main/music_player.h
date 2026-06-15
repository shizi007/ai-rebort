#ifndef MUSIC_PLAYER_H
#define MUSIC_PLAYER_H

#include <string>
#include <cJSON.h>

/**
 * 音乐播放状态枚举
 */
enum MusicPlayState {
    kMusicStateIdle,
    kMusicStatePlaying,
    kMusicStatePaused
};

/**
 * 音乐播放器状态管理类
 * 
 * 在方案A（服务端方案）中，此类的职责是：
 * 1. 维护当前播放的歌曲信息（标题、艺术家、专辑等）
 * 2. 供 Display 层读取以更新屏幕显示
 * 3. 接收服务端推送的状态更新
 * 4. 记录按键事件以转发给服务端
 * 
 * 不负责：
 * - 音频流获取（服务端负责）
 * - 音频解码（服务端负责，下发 Opus 包通过现有音频通道播放）
 * - 播放队列管理（服务端负责）
 */
class MusicPlayer {
public:
    static MusicPlayer& GetInstance() {
        static MusicPlayer instance;
        return instance;
    }

    MusicPlayer(const MusicPlayer&) = delete;
    MusicPlayer& operator=(const MusicPlayer&) = delete;

    /**
     * 解析并应用服务端推送的音乐状态 JSON
     * 
     * JSON 格式:
     * {
     *   "type": "music",
     *   "action": "update" | "stop",
     *   "title": "告白气球",
     *   "artist": "周杰伦",
     *   "album": "周杰伦的床边故事",
     *   "duration": 240,
     *   "position": 30,
     *   "state": "playing" | "paused"
     * }
     */
    void UpdateFromJson(const cJSON* root);

    /**
     * 处理服务端调用的 MCP 工具 media.music.set_status
     */
    bool SetStatus(const cJSON* arguments);

    // Getters
    inline MusicPlayState state() const { return state_; }
    inline const std::string& title() const { return title_; }
    inline const std::string& artist() const { return artist_; }
    inline const std::string& album() const { return album_; }
    inline int duration_sec() const { return duration_sec_; }
    inline int position_sec() const { return position_sec_; }
    inline bool is_playing() const { return state_ == kMusicStatePlaying; }

    /**
     * 序列化当前状态为 JSON（用于 MCP 工具返回值）
     */
    cJSON* ToJson() const;

    /**
     * 清除当前播放状态
     */
    void Clear();

private:
    MusicPlayer() = default;
    ~MusicPlayer() = default;

    MusicPlayState state_ = kMusicStateIdle;
    std::string title_;
    std::string artist_;
    std::string album_;
    int duration_sec_ = 0;
    int position_sec_ = 0;
};

#endif // MUSIC_PLAYER_H
