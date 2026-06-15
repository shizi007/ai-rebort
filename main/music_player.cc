#include "music_player.h"
#include <esp_log.h>
#include <cstring>

#define TAG "MusicPlayer"

void MusicPlayer::UpdateFromJson(const cJSON* root) {
    auto action = cJSON_GetObjectItem(root, "action");
    if (!cJSON_IsString(action)) {
        ESP_LOGW(TAG, "Missing 'action' field in music JSON");
        return;
    }

    if (strcmp(action->valuestring, "stop") == 0) {
        Clear();
        ESP_LOGI(TAG, "Music stopped");
        return;
    }

    if (strcmp(action->valuestring, "update") != 0) {
        ESP_LOGW(TAG, "Unknown music action: %s", action->valuestring);
        return;
    }

    // Parse fields
    auto title = cJSON_GetObjectItem(root, "title");
    auto artist = cJSON_GetObjectItem(root, "artist");
    auto album = cJSON_GetObjectItem(root, "album");
    auto duration = cJSON_GetObjectItem(root, "duration");
    auto position = cJSON_GetObjectItem(root, "position");
    auto state = cJSON_GetObjectItem(root, "state");

    if (cJSON_IsString(title)) {
        title_ = title->valuestring;
    }
    if (cJSON_IsString(artist)) {
        artist_ = artist->valuestring;
    }
    if (cJSON_IsString(album)) {
        album_ = album->valuestring;
    }
    if (cJSON_IsNumber(duration)) {
        duration_sec_ = duration->valueint;
    }
    if (cJSON_IsNumber(position)) {
        position_sec_ = position->valueint;
    }
    if (cJSON_IsString(state)) {
        if (strcmp(state->valuestring, "playing") == 0) {
            state_ = kMusicStatePlaying;
        } else if (strcmp(state->valuestring, "paused") == 0) {
            state_ = kMusicStatePaused;
        } else {
            state_ = kMusicStateIdle;
        }
    } else {
        // Default to playing if state is missing
        state_ = kMusicStatePlaying;
    }

    ESP_LOGI(TAG, "Music update: %s - %s [%s] %d/%ds",
        title_.c_str(), artist_.c_str(),
        state_ == kMusicStatePlaying ? "playing" : "paused",
        position_sec_, duration_sec_);
}

bool MusicPlayer::SetStatus(const cJSON* arguments) {
    auto title = cJSON_GetObjectItem(arguments, "title");
    auto artist = cJSON_GetObjectItem(arguments, "artist");
    auto album = cJSON_GetObjectItem(arguments, "album");
    auto duration = cJSON_GetObjectItem(arguments, "duration");
    auto position = cJSON_GetObjectItem(arguments, "position");
    auto state = cJSON_GetObjectItem(arguments, "state");

    if (cJSON_IsString(title)) {
        title_ = title->valuestring;
    }
    if (cJSON_IsString(artist)) {
        artist_ = artist->valuestring;
    }
    if (cJSON_IsString(album)) {
        album_ = album->valuestring;
    }
    if (cJSON_IsNumber(duration)) {
        duration_sec_ = duration->valueint;
    }
    if (cJSON_IsNumber(position)) {
        position_sec_ = position->valueint;
    }
    if (cJSON_IsString(state)) {
        if (strcmp(state->valuestring, "playing") == 0) {
            state_ = kMusicStatePlaying;
        } else if (strcmp(state->valuestring, "paused") == 0) {
            state_ = kMusicStatePaused;
        } else {
            state_ = kMusicStateIdle;
        }
    }

    ESP_LOGI(TAG, "MCP set_status: %s - %s [%d/%ds]",
        title_.c_str(), artist_.c_str(), position_sec_, duration_sec_);
    return true;
}

cJSON* MusicPlayer::ToJson() const {
    cJSON* json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "state",
        state_ == kMusicStatePlaying ? "playing" :
        state_ == kMusicStatePaused ? "paused" : "idle");
    cJSON_AddStringToObject(json, "title", title_.c_str());
    cJSON_AddStringToObject(json, "artist", artist_.c_str());
    cJSON_AddStringToObject(json, "album", album_.c_str());
    cJSON_AddNumberToObject(json, "duration", duration_sec_);
    cJSON_AddNumberToObject(json, "position", position_sec_);
    return json;
}

void MusicPlayer::Clear() {
    state_ = kMusicStateIdle;
    title_.clear();
    artist_.clear();
    album_.clear();
    duration_sec_ = 0;
    position_sec_ = 0;
    ESP_LOGI(TAG, "Music player cleared");
}
