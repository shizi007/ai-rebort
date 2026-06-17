#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// ============================================================================
// WALL-E 机器人 GPIO 分配方案 v10 — 纯 P1 排针方案
// ============================================================================
//
// 架构变更(v10)：完全不使用板载走线，所有外设只从 P1 排针引出
// 删除 XL9555 IO 扩展、ES8388 板载音频、板载 ST7789 LCD
// 改用 ST7735S 1.3" 240×240 外接 LCD，所有 CS 改 GPIO 直控
//
// P1 排针全部分配（19个）：
//   GPIO3  → LCD CS (ST7735S)
//   GPIO4  → TB6612 AIN1
//   GPIO5  → TB6612 AIN2
//   GPIO6  → TB6612 BIN1
//   GPIO7  → TB6612 BIN2
//   GPIO9  → SPI MOSI (LCD + 眼屏共享)
//   GPIO10 → SPI SCLK (LCD + 眼屏共享)
//   GPIO14 → SPI DC (LCD + 眼屏共享)
//   GPIO15 → I2S1 BCLK (PCM5102 + INMP441)
//   GPIO16 → UART2 TX (→ ESP32-CAM RX)
//   GPIO17 → UART2 RX (← ESP32-CAM TX)
//   GPIO18 → I2S1 WS (PCM5102 + INMP441)
//   GPIO19 → 眼屏 CS_L (GC9A01)
//   GPIO20 → 眼屏 CS_R (GC9A01)
//   GPIO38 → I2C1 SCL (PCA9685 + VL53L0X)
//   GPIO39 → I2C1 SDA (PCA9685 + VL53L0X)
//   GPIO45 → I2S1 DOUT (→ PCM5102 DIN)
//   GPIO47 → I2S1 DIN  (← INMP441 SD)
//   GPIO48 → LCD 背光
//
// 板载设备 (不再使用)：
//   [v10] 已删除：I2S0 音频 (ES8388), I2C0 内部 (XL9555 + ES8388)
//   [v10] 已删除：板载 LCD SPI (ST7789), XL9555 IO 扩展
//
// GPIO 不可用：
//   GPIO26-37 → Octal PSRAM/Flash 硬件占用
//   GPIO43/44 → UART0 控制台
// ============================================================================

// ============ 外部 I2C 总线（PCA9685 + VL53L0X，独享 I2C1） ============
#define EXTERNAL_I2C_SDA_PIN    GPIO_NUM_39
#define EXTERNAL_I2C_SCL_PIN    GPIO_NUM_38
#define EXTERNAL_I2C_PORT       I2C_NUM_1

// ============ UART2 连接 ESP32-CAM 子板 ============
#define UART_CAM_PORT           UART_NUM_2
#define UART_CAM_TX_PIN         GPIO_NUM_16     // P1 上排 → ESP32-CAM RX
#define UART_CAM_RX_PIN         GPIO_NUM_17     // P1 下排 ← ESP32-CAM TX
#define UART_CAM_BAUD           921600

// ============ 音频参数 ============
#define AUDIO_INPUT_SAMPLE_RATE      24000
#define AUDIO_OUTPUT_SAMPLE_RATE     24000

// [v10] 已删除：AUDIO_I2S_GPIO_MCLK/WS/BCLK/DIN/DOUT (I2S0/ES8388)
// [v10] 已删除：AUDIO_CODEC_I2C_SDA_PIN / AUDIO_CODEC_I2C_SCL_PIN / AUDIO_CODEC_ES8388_ADDR

// ============ 按键/LED ============
#define BOOT_BUTTON_GPIO  GPIO_NUM_0
// [v10] 已删除：BUILTIN_LED_GPIO (GPIO4 已用于 TB6612 AIN1)
#define BUILTIN_LED_GPIO  GPIO_NUM_NC

// ============ 外接 LCD（ST7735S 1.3" 240×240, SPI, 从 P1 排针引出） ============
#define LCD_SCLK_PIN GPIO_NUM_10   // P1 上排 (原 I2S0 DOUT, 释放)
#define LCD_MOSI_PIN GPIO_NUM_9    // P1 下排 (原 I2S0 WS, 释放)
#define LCD_MISO_PIN GPIO_NUM_NC   // ST7735 7针无MISO
#define LCD_DC_PIN   GPIO_NUM_14   // P1 下排 (原 I2S0 DIN, 释放)
#define LCD_CS_PIN   GPIO_NUM_3    // P1 下排 (原 I2S0 MCLK, 释放)
#define LCD_BL_PIN   GPIO_NUM_48   // P1 下排, LCD背光

// ============ 显示参数 (ST7735S 240×240) ============
#define DISPLAY_WIDTH    240
#define DISPLAY_HEIGHT   240
#define DISPLAY_MIRROR_X false   // ST7735S 可能需要调整
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY  false
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0

// [v10] 已删除：DISPLAY_BACKLIGHT_PIN / DISPLAY_BACKLIGHT_OUTPUT_INVERT (改用 LCD_BL_PIN GPIO直控)

// ============ OV2640 摄像头 — 已移至 ESP32-CAM 子板 ============
// 所有 DVP 引脚不再使用，由 UART2 传输 JPEG
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
#define OV_PWDN_IO      0xFF
#define OV_RESET_IO     0xFF

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
#define SERVO_L_BROW_CH       7
#define SERVO_R_BROW_CH       8

#define SERVO_COUNT          9

// 电机 PWM 通道（CH9-10）
#define MOTOR_LEFT_PWM_CH     9
#define MOTOR_RIGHT_PWM_CH    10

// ============ GC9A01 圆形屏眼睛（共享主屏 SPI 总线，GPIO 直控 CS） ============
#define EYE_SPI_HOST           SPI2_HOST
#define EYE_CS_LEFT_PIN        GPIO_NUM_19  // P1 上排, GPIO 直控
#define EYE_CS_RIGHT_PIN       GPIO_NUM_20  // P1 下排, GPIO 直控
// [v10] 已删除：WALLE_EYE_CS_L_XL9555 / WALLE_EYE_CS_R_XL9555 (改用 GPIO 直控)
#define EYE_CS_LEFT            EYE_CS_LEFT_PIN
#define EYE_CS_RIGHT           EYE_CS_RIGHT_PIN
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
#define TB6612_AIN1_GPIO    GPIO_NUM_4   // P1 下排
#define TB6612_AIN2_GPIO    GPIO_NUM_5   // P1 上排
#define TB6612_BIN1_GPIO    GPIO_NUM_6   // P1 下排
#define TB6612_BIN2_GPIO    GPIO_NUM_7   // P1 上排
// [v10] 已删除：TB6612_AIN1/AIN2/BIN1/BIN2_XL9555_BIT (不再使用 XL9555)

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

// ============ 外接音频 I2S1 (PCM5102 DAC + INMP441 麦克风) ============
// [v10] 升级：ExternalAudio 实现 AudioCodec 接口，替代板载 ES8388
// PCM5102 无需 I2C 控制（硬件自启动），INMP441 无需 I2C 控制
#define EXTERNAL_AUDIO_I2S_NUM       I2S_NUM_1
#define EXTERNAL_AUDIO_BCLK_PIN      GPIO_NUM_15   // P1 下排 → PCM5102 BCK + INMP441 SCK
#define EXTERNAL_AUDIO_WS_PIN        GPIO_NUM_18   // P1 上排 → PCM5102 WS  + INMP441 WS
#define EXTERNAL_AUDIO_DOUT_PIN      GPIO_NUM_45   // P1 上排 → PCM5102 DIN (喇叭输出)
#define EXTERNAL_AUDIO_DIN_PIN       GPIO_NUM_47   // P1 上排 ← INMP441 SD  (麦克风输入)
#define EXTERNAL_AUDIO_SAMPLE_RATE   24000
#define EXTERNAL_AUDIO_OUT_CHANNELS  1             // PCM5102 单声道(与 I2S MONO+LEFT slot 一致)
#define EXTERNAL_AUDIO_IN_CHANNELS   1             // INMP441 单声道

#endif // _BOARD_CONFIG_H_
