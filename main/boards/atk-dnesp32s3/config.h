#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// ============ 外部 I2C 总线（PCA9685 + VL53L0X + OV2640 SCCB 共享） ============
// GPIO41/42 为内部 I2C0（XL9555+ES8388），未引出到排针
// GPIO26-37 被 Octal PSRAM/Flash 占用，绝对不可用！
// 使用 I2C1 + GPIO38/39 作为外部 I2C 总线
// 摄像头 SCCB 也使用 GPIO38/39 (CAM_PIN_SIOC/SIOD)
// I2C 总线设备地址无冲突：OV2640 SCCB=0x3C, PCA9685=0x40, VL53L0X=0x29
// 注意：PCA9685 和 VL53L0X 的 SDA/SCL 需物理连到 GPIO38/39 排针
#define EXTERNAL_I2C_SDA_PIN    GPIO_NUM_39
#define EXTERNAL_I2C_SCL_PIN    GPIO_NUM_38
#define EXTERNAL_I2C_PORT       I2C_NUM_1

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
#define BUILTIN_LED_GPIO  GPIO_NUM_NC  // GPIO4 为板载LED但被CAM D0占用

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

// ============ WALL-E 舵机配置（PCA9685 I2C 驱动，9舵机） ============
// 电源：12V 输入 → DC-DC 10-36V 降压模块 → 5V 输出 → 全系统供电
// PCA9685 I2C 地址 0x40，50Hz 舵机频率
#define PCA9685_I2C_ADDR      0x40
#define PCA9685_SERVO_FREQ_HZ 50.0f

// 眼睛舵机（CH0-1）
#define SERVO_L_EYE_CH       0    // 左眼
#define SERVO_R_EYE_CH       1    // 右眼

// 头部和脖子舵机（CH2-4）
#define SERVO_NECK_LR_CH     2    // 脖子左右转动
#define SERVO_HEAD_UD_CH     3    // 头部上下
#define SERVO_NECK_OI_CH     4    // 脖子伸缩回

// 手臂舵机（CH5-6）
#define SERVO_L_ARM_CH       5    // 左手
#define SERVO_R_ARM_CH       6    // 右手

// 眉毛舵机（新增，CH7-8）
#define SERVO_L_BROW_CH      7    // 左眉毛
#define SERVO_R_BROW_CH      8    // 右眉毛

// 舵机总数
#define SERVO_COUNT          9

// 电机 PWM 通道（CH9-10，紧接舵机之后）
#define MOTOR_LEFT_PWM_CH     9
#define MOTOR_RIGHT_PWM_CH    10

// ============ GC9A01 圆形屏眼睛（共享主屏 SPI 总线，7针无BL） ============
// 复用 LCD 的 SCLK(GPIO12) / MOSI(GPIO11) / DC(GPIO40)
// CS 由 XL9555 IO 扩展芯片控制（释放 GPIO1+GPIO44，避免 UART0 冲突）
#define EYE_SPI_HOST           SPI2_HOST
#define EYE_CS_LEFT            GPIO_NUM_NC   // 已迁移至 XL9555 P02
#define EYE_CS_RIGHT           GPIO_NUM_NC   // 已迁移至 XL9555 P03
#define WALLE_EYE_CS_L_XL9555  2             // XL9555 P02 → GC9A01 CS LEFT
#define WALLE_EYE_CS_R_XL9555  3             // XL9555 P03 → GC9A01 CS RIGHT
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
// 方向引脚通过 XL9555 IO 扩展芯片控制（GPIO2/8/19/20 不在排针上或需保留）
// GPIO_NUM_NC 表示使用 XL9555 回调，不直连 GPIO
#define TB6612_AIN1_GPIO    GPIO_NUM_NC  // XL9555 P00
#define TB6612_AIN2_GPIO    GPIO_NUM_NC  // XL9555 P01
#define TB6612_BIN1_GPIO    GPIO_NUM_NC  // XL9555 P06
#define TB6612_BIN2_GPIO    GPIO_NUM_NC  // XL9555 P07
#define TB6612_AIN1_XL9555_BIT  0        // XL9555 P00
#define TB6612_AIN2_XL9555_BIT  1        // XL9555 P01
#define TB6612_BIN1_XL9555_BIT  6        // XL9555 P06
#define TB6612_BIN2_XL9555_BIT  7        // XL9555 P07

// ============ 各舵机角度范围 ============

// 脖子左右转动 (NECK_LR, CH2)
#define NECK_LR_MIN        30.0f
#define NECK_LR_MAX        150.0f
#define NECK_LR_CENTER     90.0f

// 头部上下 (HEAD_UD, CH3)
#define HEAD_UD_MIN        45.0f
#define HEAD_UD_MAX        135.0f
#define HEAD_UD_CENTER     90.0f

// 脖子伸缩回 (NECK_OI, CH4)
#define NECK_OI_MIN        20.0f
#define NECK_OI_MAX        160.0f
#define NECK_OI_CENTER     90.0f

// 左右手臂 (ARM, CH5-6)
#define ARM_MIN             20.0f
#define ARM_MAX             160.0f
#define ARM_LEFT_CENTER     70.0f
#define ARM_RIGHT_CENTER    110.0f

// 眼睛 (EYE, CH0-1)
#define EYE_MIN             30.0f
#define EYE_MAX             150.0f
#define EYE_LEFT_CENTER     90.0f
#define EYE_RIGHT_CENTER    90.0f

// 眉毛 (BROW, CH7-8) — 小角度摆动
#define BROW_MIN            45.0f
#define BROW_MAX            135.0f
#define BROW_LEFT_CENTER    90.0f
#define BROW_RIGHT_CENTER   90.0f

#endif // _BOARD_CONFIG_H_
