#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// ============================================================================
// WALL-E 机器人 GPIO 分配方案 v3 — 独立 ESP32-CAM 子板
// ============================================================================
//
// 架构变更：OV2640 不再直连主板 DVP，改用 ESP32-CAM 子板通过 UART2 传 JPEG
// 释放 13 个 P1 排针引脚 (GPIO4/5/6/7/15/16/17/18/38/39/45/47/48)
//
// P1 排针分配：
//   GPIO4  → TB6612 AIN1  (原 CAM D0)
//   GPIO5  → TB6612 AIN2  (原 CAM D1)
//   GPIO6  → TB6612 BIN1  (原 CAM D2)
//   GPIO7  → TB6612 BIN2  (原 CAM D3)
//   GPIO16 → UART2 TX (→ ESP32-CAM RX)  (原 CAM D5)
//   GPIO17 → UART2 RX (← ESP32-CAM TX)  (原 CAM D6)
//   GPIO15 → 空闲 (原 CAM D4)
//   GPIO18 → 空闲 (原 CAM D7)
//   GPIO38 → I2C1 SCL (PCA9685 + VL53L0X) (原 CAM SIOC)
//   GPIO39 → I2C1 SDA (PCA9685 + VL53L0X) (原 CAM SIOD)
//   GPIO45 → 空闲 (原 CAM PCLK)
//   GPIO47 → 空闲 (原 CAM VSYNC)
//   GPIO48 → 空闲 (原 CAM HREF)
//
// 板载设备 (不占 P1)：
//   GPIO3/9/10/14/46  → I2S0 音频 (ES8388)
//   GPIO41/42         → I2C0 内部 (XL9555 + ES8388)
//   GPIO11/12/13/21/40 → LCD SPI (ST7789)
//   GPIO0             → BOOT 按键
//
// GPIO 不可用：
//   GPIO26-37 → Octal PSRAM/Flash 硬件占用
//   GPIO43/44 → UART0 控制台
// ============================================================================

// ============ 外部 I2C 总线（PCA9685 + VL53L0X，独享 I2C1） ============
// 不再与摄像头 SCCB 共享！ESP32-CAM 子板有自己的 I2C 总线
#define EXTERNAL_I2C_SDA_PIN    GPIO_NUM_39
#define EXTERNAL_I2C_SCL_PIN    GPIO_NUM_38
#define EXTERNAL_I2C_PORT       I2C_NUM_1

// ============ UART2 连接 ESP32-CAM 子板 ============
#define UART_CAM_PORT           UART_NUM_2
#define UART_CAM_TX_PIN         GPIO_NUM_16     // P1 上排 → ESP32-CAM RX
#define UART_CAM_RX_PIN         GPIO_NUM_17     // P1 下排 ← ESP32-CAM TX
#define UART_CAM_BAUD           921600

// ============ 板载音频（ES8388 codec, I2S0） ============
#define AUDIO_INPUT_SAMPLE_RATE      24000
#define AUDIO_OUTPUT_SAMPLE_RATE     24000

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_3
#define AUDIO_I2S_GPIO_WS   GPIO_NUM_9
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_46
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_14
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_10

#define AUDIO_CODEC_I2C_SDA_PIN GPIO_NUM_41
#define AUDIO_CODEC_I2C_SCL_PIN GPIO_NUM_42
#define AUDIO_CODEC_ES8388_ADDR ES8388_CODEC_DEFAULT_ADDR

// ============ 按键/LED ============
#define BOOT_BUTTON_GPIO  GPIO_NUM_0
#define BUILTIN_LED_GPIO  GPIO_NUM_4   // GPIO4 已释放 (原 CAM D0)

// ============ LCD 显示（ST7789 SPI） ============
#define LCD_SCLK_PIN GPIO_NUM_12
#define LCD_MOSI_PIN GPIO_NUM_11
#define LCD_MISO_PIN GPIO_NUM_13
#define LCD_DC_PIN   GPIO_NUM_40
#define LCD_CS_PIN   GPIO_NUM_21

#define DISPLAY_WIDTH    320
#define DISPLAY_HEIGHT   240
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY  true

#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0

#define DISPLAY_BACKLIGHT_PIN          GPIO_NUM_NC
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT true

// ============ OV2640 摄像头 — 已移至 ESP32-CAM 子板 ============
// 所有 DVP 引脚不再使用，由 UART2 传输 JPEG
// 保留空定义以避免编译错误（某些代码可能引用这些宏）
#define CAM_PIN_PWDN    GPIO_NUM_NC
#define CAM_PIN_RESET   GPIO_NUM_NC
#define CAM_PIN_VSYNC   GPIO_NUM_NC
#define CAM_PIN_HREF    GPIO_NUM_NC
#define CAM_PIN_PCLK    GPIO_NUM_NC
#define CAM_PIN_XCLK    GPIO_NUM_NC
#define CAM_PIN_SIOD    GPIO_NUM_NC
#define CAM_PIN_SIOC    GPIO_NUM_NC
#define CAM_PIN_D0      GPIO_NUM_NC
#define CAM_PIN_D1      GPIO_NUM_NC
#define CAM_PIN_D2      GPIO_NUM_NC
#define CAM_PIN_D3      GPIO_NUM_NC
#define CAM_PIN_D4      GPIO_NUM_NC
#define CAM_PIN_D5      GPIO_NUM_NC
#define CAM_PIN_D6      GPIO_NUM_NC
#define CAM_PIN_D7      GPIO_NUM_NC
// OV2640 PWDN/RESET 不再需要 XL9555 控制
#define OV_PWDN_IO      0xFF  // 不使用
#define OV_RESET_IO     0xFF  // 不使用

// ============ WALL-E 舵机配置（PCA9685 I2C 驱动，9舵机） ============
#define PCA9685_I2C_ADDR      0x40
#define PCA9685_SERVO_FREQ_HZ 50.0f

// 眼睛舵机（CH0-1）
#define SERVO_L_EYE_CH       0
#define SERVO_R_EYE_CH       1

// 头部和脖子舵机（CH2-4）
#define SERVO_NECK_LR_CH     2
#define SERVO_HEAD_UD_CH     3
#define SERVO_NECK_OI_CH     4

// 手臂舵机（CH5-6）
#define SERVO_L_ARM_CH       5
#define SERVO_R_ARM_CH       6

// 眉毛舵机（CH7-8）
#define SERVO_L_BROW_CH      7
#define SERVO_R_BROW_CH      8

#define SERVO_COUNT          9

// 电机 PWM 通道（CH9-10）
#define MOTOR_LEFT_PWM_CH     9
#define MOTOR_RIGHT_PWM_CH    10

// ============ GC9A01 圆形屏眼睛（共享主屏 SPI 总线，7针无BL） ============
#define EYE_SPI_HOST           SPI2_HOST
#define EYE_CS_LEFT            GPIO_NUM_NC
#define EYE_CS_RIGHT           GPIO_NUM_NC
#define WALLE_EYE_CS_L_XL9555  2     // XL9555 P02
#define WALLE_EYE_CS_R_XL9555  3     // XL9555 P03
#define EYE_RESOLUTION     240
#define EYE_PCLK_HZ        (40 * 1000 * 1000)

// ============ VL53L0X 激光测距 ============
#define VL53L0X_I2C_ADDR           0x29
#define VL53L0X_MAX_RANGE_MM       2000
#define VL53L0X_MIN_RANGE_MM       30
#define TRACK_FOLLOW_DIST_MM       800.0f
#define TRACK_DIST_TOLERANCE_MM    200.0f
#define TRACK_TOO_CLOSE_MM         400.0f
#define TRACK_TOO_FAR_MM           1500.0f

// ============ 眼睛渲染参数 ============
#define EYE_IRIS_RADIUS         50
#define EYE_PUPIL_RADIUS        20
#define EYE_HIGHLIGHT_RADIUS    8
#define EYE_LID_COLOR_R         40
#define EYE_LID_COLOR_G         30
#define EYE_LID_COLOR_B         20

// ============ TB6612 电机驱动引脚（直连 P1 排针） ============
// 摄像头外置后，GPIO4/5/6/7 释放，TB6612 不再需要 XL9555 扩展！
#define TB6612_AIN1_GPIO    GPIO_NUM_4   // P1 下排 (原 CAM D0)
#define TB6612_AIN2_GPIO    GPIO_NUM_5   // P1 上排 (原 CAM D1)
#define TB6612_BIN1_GPIO    GPIO_NUM_6   // P1 下排 (原 CAM D2)
#define TB6612_BIN2_GPIO    GPIO_NUM_7   // P1 上排 (原 CAM D3)
// XL9555 位号不再使用（保留定义以防编译错误）
#define TB6612_AIN1_XL9555_BIT  0xFF
#define TB6612_AIN2_XL9555_BIT  0xFF
#define TB6612_BIN1_XL9555_BIT  0xFF
#define TB6612_BIN2_XL9555_BIT  0xFF

// ============ 各舵机角度范围 ============
#define NECK_LR_MIN        30.0f
#define NECK_LR_MAX        150.0f
#define NECK_LR_CENTER     90.0f

#define HEAD_UD_MIN        45.0f
#define HEAD_UD_MAX        135.0f
#define HEAD_UD_CENTER     90.0f

#define NECK_OI_MIN        20.0f
#define NECK_OI_MAX        160.0f
#define NECK_OI_CENTER     90.0f

#define ARM_MIN             20.0f
#define ARM_MAX             160.0f
#define ARM_LEFT_CENTER     70.0f
#define ARM_RIGHT_CENTER    110.0f

#define EYE_MIN             30.0f
#define EYE_MAX             150.0f
#define EYE_LEFT_CENTER     90.0f
#define EYE_RIGHT_CENTER    90.0f

#define BROW_MIN            45.0f
#define BROW_MAX            135.0f
#define BROW_LEFT_CENTER    90.0f
#define BROW_RIGHT_CENTER   90.0f

// ============ P1 排针空闲 GPIO (可供未来扩展) ============
// GPIO15, GPIO18, GPIO45, GPIO47, GPIO48

#endif // _BOARD_CONFIG_H_
