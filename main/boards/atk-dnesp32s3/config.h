#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// ============ 音频（同 WALL-E 版） ============
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
#define BUILTIN_LED_GPIO  GPIO_NUM_NC  // GPIO1 已给 GC9A01 CS_RIGHT 用

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

// ============ OV2640 摄像头 DVP 接口 ============
#define CAM_PIN_PWDN    GPIO_NUM_NC
#define CAM_PIN_RESET   GPIO_NUM_NC
#define CAM_PIN_VSYNC   GPIO_NUM_47
#define CAM_PIN_HREF    GPIO_NUM_48
#define CAM_PIN_PCLK    GPIO_NUM_45
#define CAM_PIN_XCLK    GPIO_NUM_NC
#define CAM_PIN_SIOD    GPIO_NUM_39
#define CAM_PIN_SIOC    GPIO_NUM_38
#define CAM_PIN_D0      GPIO_NUM_4
#define CAM_PIN_D1      GPIO_NUM_5
#define CAM_PIN_D2      GPIO_NUM_6
#define CAM_PIN_D3      GPIO_NUM_7
#define CAM_PIN_D4      GPIO_NUM_15
#define CAM_PIN_D5      GPIO_NUM_16
#define CAM_PIN_D6      GPIO_NUM_17
#define CAM_PIN_D7      GPIO_NUM_18
#define OV_PWDN_IO      4
#define OV_RESET_IO     5

// ============ WALL-E 舵机配置（PCA9685 I2C 驱动） ============
#define PCA9685_I2C_ADDR      0x40
#define PCA9685_SERVO_FREQ_HZ 50.0f

#define SERVO_HEAD_PAN_CH     0
#define SERVO_NECK_TILT_CH    1
#define SERVO_LEFT_ARM_CH     2
#define SERVO_RIGHT_ARM_CH    3
#define SERVO_SPARE_CH        4

#define MOTOR_LEFT_PWM_CH     5
#define MOTOR_RIGHT_PWM_CH    6

// ============ GC9A01 圆形屏眼睛（共享主屏 SPI 总线，7针无BL） ============
// 复用 LCD 的 SCLK(GPIO12) / MOSI(GPIO11) / DC(GPIO40)
// 仅需两个独立 CS 引脚
#define EYE_SPI_HOST       SPI2_HOST
#define EYE_CS_LEFT        GPIO_NUM_44
#define EYE_CS_RIGHT       GPIO_NUM_1
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

// ============ TB6612 电机驱动引脚 ============
#define TB6612_AIN1_GPIO    GPIO_NUM_2
#define TB6612_AIN2_GPIO    GPIO_NUM_8
#define TB6612_BIN1_GPIO    GPIO_NUM_19
#define TB6612_BIN2_GPIO    GPIO_NUM_20

// ============ 舵机角度范围 ============
#define HEAD_PAN_MIN        30.0f
#define HEAD_PAN_MAX        150.0f
#define HEAD_PAN_CENTER     90.0f

#define NECK_TILT_MIN       45.0f
#define NECK_TILT_MAX       135.0f
#define NECK_TILT_CENTER    90.0f

#define ARM_MIN             20.0f
#define ARM_MAX             160.0f
#define ARM_LEFT_CENTER     70.0f
#define ARM_RIGHT_CENTER    110.0f

#endif // _BOARD_CONFIG_H_
