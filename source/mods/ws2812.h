#ifndef __WS2812_H__
#define __WS2812_H__

#include <stdint.h>
#include <stdbool.h>
#include "Driver_GPIO.h"
#include "IOMuxManager.h"

// ==================== 硬件引脚配置 ====================
// 指定 GPIO 端口与引脚，可根据实际开发板硬件更改
// 注意：本工程为 SOP16 封装，PB12 未引出；A11/A12 被协议串口占用，
// 故选用 A10（SOP16 封装 pad13），可作普通 GPIO 使用。
#define WS2812_GPIO_PORT        GPIOA
#define WS2812_GPIO_PAD         CSK_IOMUX_PAD_A
#define WS2812_GPIO_PIN         10
#define WS2812_GPIO_PIN_MASK    CSK_GPIO_PIN10

// ==================== 灯板规格与参数 ====================
#define WS2812_WIDTH            16
#define WS2812_HEIGHT           16
#define WS2812_LEDS_NUM         (WS2812_WIDTH * WS2812_HEIGHT) // 256 颗灯

// 亮度上限控制 (约 3%，255 * 3% ≈ 8)
// 用户反馈满屏图案仍偏亮（摄像头几乎全白），从 12 (5%) 再调低到 8 (3%)
#define WS2812_BRIGHTNESS_MAX   2

// ==================== WS2812B 时序参数（单位 ns / us）====================
// 目标值按「规格中心 + 补偿固定代码开销」选取：
// 实际高/低电平时间 = 该目标值 + DOUTSET/DOUTCLEAR 之间约 54~62ns 的
// 分支/调用/循环判定固定开销（240MHz，-Os 实测反推）。
// 1 码: T1H ≈ 640+54 ≈ 690ns (规格 550~850)，T1L ≈ 500+58 ≈ 558ns (规格 ≥450)
// 0 码: T0H ≈ 290+54 ≈ 344ns (规格 200~500)，T0L ≈ 820+59 ≈ 879ns (规格 650~950)
// 复位: 低电平维持 >= 280us（取 350us 留余量）
#define WS2812_T1H_NS           (640)
#define WS2812_T1L_NS           (500)
#define WS2812_T0H_NS           (290)
#define WS2812_T0L_NS           (820)
#define WS2812_RESET_US         (350)
// 延时循环每轮平均约 7 个 CPU 周期（-Os 优化且取指命中缓存时的估算值）
#define WS2812_LOOP_CYCLES      (7)

// RGB 颜色结构体
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_pixel_t;

// ==================== API 接口函数声明 ====================
/**
 * @brief 初始化 WS2812B GPIO 引脚并清屏
 */
void ws2812_init(void);

/**
 * @brief 清空灯板数据并刷新（全灭）
 */
void ws2812_clear(void);

/**
 * @brief 显示红灯（16x16 中心红色实心圆）
 */
void ws2812_show_red_light(void);

/**
 * @brief 显示纯白屏（16x16 全白，测试用）
 */
void ws2812_show_white(void);

/**
 * @brief 显示直行图案（绿色向上直行箭头）
 */
void ws2812_show_straight(void);

/**
 * @brief 显示左转图案（绿色向左转弯箭头）
 */
void ws2812_show_left_turn(void);

/**
 * @brief 显示右转图案（绿色向右转弯箭头）
 */
void ws2812_show_right_turn(void);

/**
 * @brief 唤醒词响应动作（闪烁提示）
 */
void ws2812_show_wakeup(void);

/**
 * @brief 将图像缓冲区数据发送更新到 WS2812B 灯板
 */
void ws2812_refresh(void);

/**
 * @brief 「显示红灯」：满屏红灯并停留，不触发循环。
 */
void ws2812_traffic_red(void);

/**
 * @brief 「直行/左转/右转」：用指定形状启动一次循环（立即显示绿灯，绿->黄->红后停在红灯）。
 * @param shape 1=直行 2=左转 3=右转
 */
void ws2812_traffic_start(uint8_t shape);

/**
 * @brief 交通灯循环节拍（每 1900ms 切换一次相位，共执行红绿黄红一次）。
 *        建议由主循环每 100ms 调用一次。
 * @param dt_ms 距上次调用的毫秒数（通常为 100）
 */
void ws2812_traffic_tick(uint32_t dt_ms);

/**
 * @brief 设置点阵指定 (x, y) 坐标像素点颜色（自动进行 S 型走线转换与亮度控制）
 * @param x 列坐标 (0 ~ 15)
 * @param y 行坐标 (0 ~ 15)
 * @param r 红色强度 (0 ~ 255)
 * @param g 绿色强度 (0 ~ 255)
 * @param b 蓝色强度 (0 ~ 255)
 */
void ws2812_set_pixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);

#endif /* __WS2812_H__ */
