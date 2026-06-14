#ifndef WALLE_DEBUG_SERVER_H
#define WALLE_DEBUG_SERVER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 WALL-E 调试 Web 服务器
 * @return esp_err_t ESP_OK 成功, 其他失败
 */
esp_err_t walle_debug_server_start(void);

/**
 * @brief 停止调试 Web 服务器
 */
void walle_debug_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif // WALLE_DEBUG_SERVER_H
