/**
 * @file walle_debug_server.cc
 * @brief WALL-E 机器人 Web 调试服务器实现
 * 
 * 提供 HTTP API 用于调试和控制 WALL-E 机器人
 * 路由：
 *   GET  /debug      - 调试界面 HTML
 *   GET  /api/status - 系统状态
 *   GET  /api/i2c_scan - I²C 扫描
 *   GET  /api/distance - 测距数据
 *   POST /api/servo  - 舵机控制
 *   POST /api/motor  - 电机控制
 *   POST /api/eyes   - 眼睛灯控制
 *   POST /api/expression - 表情播放
 *   POST /api/stop   - 紧急停止
 *   POST /api/restart - 重启系统
 */

#include "walle_debug_server.h"
#include "boards/atk-dnesp32s3/config.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "driver/temperature_sensor.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include <sys/unistd.h>

// WALL-E 功能 C 包装函数声明
extern "C" {
    void walle_eyes_turnOff();
    void walle_eyes_setColor(uint8_t r, uint8_t g, uint8_t b);
    void walle_eyes_setBreath(uint8_t r, uint8_t g, uint8_t b);
    void walle_eyes_setBlink(uint8_t r, uint8_t g, uint8_t b);
    void walle_eyes_setRainbow();
    void walle_eyes_setBrightness(float level);
    void walle_eyes_setMode(const char* mode_str);
    void walle_eyes_start();
    
    void walle_expression_playHappy();
    void walle_expression_playSad();
    void walle_expression_playSurprised();
    void walle_expression_playAngry();
    void walle_expression_playSleepy();
    void walle_expression_playWave();
    
    void walle_servo_setAngle(int channel, int angle);
    void walle_motor_setSpeed(int left, int right);
    void walle_emergency_stop();
    uint16_t walle_vl53l0x_readRange();
    void walle_system_restart();
    char* walle_i2c_scan_json();

}


// 嵌入式 HTML 文件 (通过 CMake EMBED_FILES 嵌入)
extern const uint8_t debug_html_start[] asm("_binary_debug_html_start");
extern const uint8_t debug_html_end[]   asm("_binary_debug_html_end");

#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/inet.h"

// WALL-E 功能通过 C 包装函数访问（在 atk_dnesp32s3.cc 中定义）
// 不再需要 extern 类指针

static const char* TAG = "WALLE_DEBUG";

static httpd_handle_t g_server = NULL;

// 获取嵌入式 HTML 大小
static size_t get_embedded_html_size() {
    return (debug_html_end - debug_html_start);
}

//
// HTTP 请求处理函数
//

// GET /debug - 返回调试界面 HTML
static esp_err_t debug_html_handler(httpd_req_t *req) {
    size_t html_size = get_embedded_html_size();
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, (const char*)debug_html_start, html_size);
    return ESP_OK;
}

// GET /api/status - 系统状态
static esp_err_t status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    
    // WiFi 状态
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    }
    
    bool wifi_connected = false;
    char ip_str[16] = "0.0.0.0";
    
    if (netif) {
        esp_netif_get_ip_info(netif, &ip_info);
        if (ip_info.ip.addr != 0) {
            wifi_connected = true;
            inet_ntoa_r(ip_info.ip.addr, ip_str, sizeof(ip_str));
        }
    }
    
    cJSON_AddBoolToObject(root, "wifi_connected", wifi_connected);
    cJSON_AddStringToObject(root, "ip", ip_str);
    
    // 系统信息
    size_t heap_free = esp_get_free_heap_size();
    size_t heap_total = heap_free + heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    cJSON_AddNumberToObject(root, "heap_free", heap_free);
    cJSON_AddNumberToObject(root, "heap_total", heap_total);
    
    // CPU 温度 (ESP32-S3)
#ifdef CONFIG_IDF_TARGET_ESP32S3
    static temperature_sensor_handle_t temp_handle = nullptr;
    if (!temp_handle) {
        temperature_sensor_config_t temp_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
        temperature_sensor_install(&temp_config, &temp_handle);
    }
    float cpu_temp = 0;
    if (temp_handle) temperature_sensor_get_celsius(temp_handle, &cpu_temp);
    cJSON_AddNumberToObject(root, "cpu_temp", cpu_temp);
#else
    cJSON_AddNumberToObject(root, "cpu_temp", 0);
#endif
    
    // 运行时间
    cJSON_AddNumberToObject(root, "uptime", esp_timer_get_time() / 1000000);
    
    // 芯片信息
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    cJSON_AddNumberToObject(root, "chip_rev", chip_info.revision);
    cJSON_AddNumberToObject(root, "cpu_cores", chip_info.cores);
    
    const char *resp = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    free((void*)resp);
    cJSON_Delete(root);
    return ESP_OK;
}

// GET /api/i2c_scan - I²C 扫描
static esp_err_t i2c_scan_handler(httpd_req_t *req) {
    char* json = walle_i2c_scan_json();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// GET /api/distance - 测距数据
static esp_err_t distance_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    
    int distance = -1;
    distance = walle_vl53l0x_readRange();
    
    cJSON_AddNumberToObject(root, "distance_mm", distance);
    cJSON_AddBoolToObject(root, "valid", distance > 0 && distance < 2000);
    
    const char *resp = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    free((void*)resp);
    cJSON_Delete(root);
    return ESP_OK;
}

// POST /api/servo - 舵机控制
static esp_err_t servo_handler(httpd_req_t *req) {
    char buf[128] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *ch = cJSON_GetObjectItem(root, "ch");
    cJSON *angle = cJSON_GetObjectItem(root, "angle");
    
    if (!ch || !angle) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ch or angle");
        return ESP_FAIL;
    }
    
    int channel = ch->valueint;
    int angle_val = angle->valueint;
    
    // 限制范围（9个舵机：CH0-CH8）
    if (channel < 0 || channel > 8) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel (0-8)");
        return ESP_FAIL;
    }
    if (angle_val < 0 || angle_val > 180) {
        angle_val = 90;  // 默认归中
    }
    
    // 控制舵机（9个舵机：CH0-CH8）
    walle_servo_setAngle(channel, angle_val);
    
    ESP_LOGI(TAG, "Servo %d set to %d degrees", channel, angle_val);
    
    cJSON_Delete(root);
    
    // 返回成功
    cJSON *resp_json = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp_json, "ok", true);
    cJSON_AddNumberToObject(resp_json, "ch", channel);
    cJSON_AddNumberToObject(resp_json, "angle", angle_val);
    
    const char *resp = cJSON_PrintUnformatted(resp_json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    free((void*)resp);
    cJSON_Delete(resp_json);
    return ESP_OK;
}

// POST /api/motor - 电机控制
static esp_err_t motor_handler(httpd_req_t *req) {
    char buf[128] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *left = cJSON_GetObjectItem(root, "left");
    cJSON *right = cJSON_GetObjectItem(root, "right");
    
    if (!left || !right) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing left or right");
        return ESP_FAIL;
    }
    
    int left_speed = left->valueint;
    int right_speed = right->valueint;
    
    // 限制范围
    left_speed = (left_speed < -255) ? -255 : (left_speed > 255) ? 255 : left_speed;
    right_speed = (right_speed < -255) ? -255 : (right_speed > 255) ? 255 : right_speed;
    
    // 控制电机（速度范围 -255~255）
    walle_motor_setSpeed(left_speed, right_speed);
    
    ESP_LOGI(TAG, "Motor L=%d R=%d", left_speed, right_speed);
    
    cJSON_Delete(root);
    
    cJSON *resp_json = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp_json, "ok", true);
    cJSON_AddNumberToObject(resp_json, "left", left_speed);
    cJSON_AddNumberToObject(resp_json, "right", right_speed);
    
    const char *resp = cJSON_PrintUnformatted(resp_json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    free((void*)resp);
    cJSON_Delete(resp_json);
    return ESP_OK;
}

// POST /api/eyes - 眼睛灯控制
static esp_err_t eyes_handler(httpd_req_t *req) {
    char buf[256] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *mode = cJSON_GetObjectItem(root, "mode");
    cJSON *color = cJSON_GetObjectItem(root, "color");
    cJSON *brightness_json = cJSON_GetObjectItem(root, "brightness");
    
    if (!mode) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing mode");
        return ESP_FAIL;
    }
    
    const char* mode_str = mode->valuestring;
    const char* color_str = color ? color->valuestring : "#ffaa00";
    
    // 解析颜色 (#RRGGBB -> RGB)
    uint32_t rgb = 0;
    if (color_str[0] == '#') {
        char hex[7] = {0};
        strncpy(hex, color_str + 1, 6);
        rgb = strtol(hex, NULL, 16);
    }
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8) & 0xFF;
    uint8_t b = rgb & 0xFF;
    
    // 处理亮度 (0.0~1.0)
    float brightness = 0.6f;
    if (brightness_json) {
        brightness = (float)brightness_json->valuedouble;
        if (brightness < 0.1f) brightness = 0.1f;
        if (brightness > 1.0f) brightness = 1.0f;
        walle_eyes_setBrightness(brightness);
    }
    
    // 控制眼睛 TFT
    if (strcmp(mode_str, "start") == 0) {
        walle_eyes_start();  // GC9A01 延迟初始化
    } else if (strcmp(mode_str, "breath") == 0) {
        walle_eyes_setBreath(r, g, b);
    } else if (strcmp(mode_str, "on") == 0) {
        walle_eyes_setColor(r, g, b);  // SetColor + SetMode(kOn)
    } else if (strcmp(mode_str, "blink") == 0) {
        walle_eyes_setBlink(r, g, b);
    } else if (strcmp(mode_str, "angry") == 0) {
        walle_eyes_setColor(r, g, b);
        walle_eyes_setMode("angry");
    } else if (strcmp(mode_str, "sad") == 0) {
        walle_eyes_setColor(r, g, b);
        walle_eyes_setMode("sleepy");  // sad -> sleepy eyes
    } else if (strcmp(mode_str, "sleepy") == 0) {
        walle_eyes_setColor(r, g, b);
        walle_eyes_setMode("sleepy");
    } else if (strcmp(mode_str, "rainbow") == 0) {
        walle_eyes_setRainbow();
    } else if (strcmp(mode_str, "off") == 0) {
        walle_eyes_turnOff();
    }
    
    ESP_LOGI(TAG, "Eyes mode=%s color=#%02X%02X%02X brightness=%.2f", mode_str, r, g, b, (double)brightness);
    
    cJSON_Delete(root);
    
    cJSON *resp_json = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp_json, "ok", true);
    
    const char *resp = cJSON_PrintUnformatted(resp_json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    free((void*)resp);
    cJSON_Delete(resp_json);
    return ESP_OK;
}

// POST /api/expression - 表情播放
static esp_err_t expression_handler(httpd_req_t *req) {
    char buf[128] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *action = cJSON_GetObjectItem(root, "action");
    if (!action) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing action");
        return ESP_FAIL;
    }
    
    const char* action_str = action->valuestring;
    
    // 播放表情
    if (strcmp(action_str, "happy") == 0) {
        walle_expression_playHappy();
    } else if (strcmp(action_str, "sad") == 0) {
        walle_expression_playSad();
    } else if (strcmp(action_str, "surprised") == 0) {
        walle_expression_playSurprised();
    } else if (strcmp(action_str, "angry") == 0) {
        walle_expression_playAngry();
    } else if (strcmp(action_str, "sleepy") == 0) {
        walle_expression_playSleepy();
    } else if (strcmp(action_str, "wave") == 0) {
        walle_expression_playWave();
    }
    
    ESP_LOGI(TAG, "Expression: %s", action_str);
    
    cJSON_Delete(root);
    
    cJSON *resp_json = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp_json, "ok", true);
    
    const char *resp = cJSON_PrintUnformatted(resp_json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    free((void*)resp);
    cJSON_Delete(resp_json);
    return ESP_OK;
}

// POST /api/stop - 紧急停止
static esp_err_t stop_handler(httpd_req_t *req) {
    ESP_LOGW(TAG, "EMERGENCY STOP triggered!");
    
    // 停止所有电机 + 全部9舵机归中 + 眼睛灯关闭
    walle_emergency_stop();
    for (int i = 0; i < SERVO_COUNT; i++) {
        walle_servo_setAngle(i, 90);
    }
    walle_eyes_turnOff();
    
    cJSON *resp_json = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp_json, "ok", true);
    cJSON_AddStringToObject(resp_json, "message", "All motors stopped, servos centered");
    
    const char *resp = cJSON_PrintUnformatted(resp_json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    free((void*)resp);
    cJSON_Delete(resp_json);
    return ESP_OK;
}

// POST /api/restart - 重启系统
static esp_err_t restart_handler(httpd_req_t *req) {
    cJSON *resp_json = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp_json, "ok", true);
    cJSON_AddStringToObject(resp_json, "message", "Restarting...");
    
    const char *resp = cJSON_PrintUnformatted(resp_json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    
    free((void*)resp);
    cJSON_Delete(resp_json);
    
    // 延迟 1 秒后重启
    vTaskDelay(pdMS_TO_TICKS(1000));
    walle_system_restart();
    
    return ESP_OK;
}

//
// 公共接口实现
//

esp_err_t walle_debug_server_start(void) {
    if (g_server != NULL) {
        ESP_LOGW(TAG, "Server already running");
        return ESP_OK;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_open_sockets = 4;
    config.stack_size = 8192;
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    
    if (httpd_start(&g_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }
    
    // 注册路由
    httpd_uri_t uri;
    
    // GET /debug
    uri = {
        .uri = "/debug",
        .method = HTTP_GET,
        .handler = debug_html_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // GET /api/status
    uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // GET /api/i2c_scan
    uri = {
        .uri = "/api/i2c_scan",
        .method = HTTP_GET,
        .handler = i2c_scan_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // GET /api/distance
    uri = {
        .uri = "/api/distance",
        .method = HTTP_GET,
        .handler = distance_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // POST /api/servo
    uri = {
        .uri = "/api/servo",
        .method = HTTP_POST,
        .handler = servo_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // POST /api/motor
    uri = {
        .uri = "/api/motor",
        .method = HTTP_POST,
        .handler = motor_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // POST /api/eyes
    uri = {
        .uri = "/api/eyes",
        .method = HTTP_POST,
        .handler = eyes_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // POST /api/expression
    uri = {
        .uri = "/api/expression",
        .method = HTTP_POST,
        .handler = expression_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // POST /api/stop
    uri = {
        .uri = "/api/stop",
        .method = HTTP_POST,
        .handler = stop_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    // POST /api/restart
    uri = {
        .uri = "/api/restart",
        .method = HTTP_POST,
        .handler = restart_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &uri);
    
    ESP_LOGI(TAG, "WALL-E Debug Server started successfully");
    ESP_LOGI(TAG, "Access debug interface at http://<ESP32_IP>/debug");
    
    return ESP_OK;
}

void walle_debug_server_stop(void) {
    if (g_server) {
        httpd_stop(g_server);
        g_server = NULL;
        ESP_LOGI(TAG, "Debug server stopped");
    }
}
