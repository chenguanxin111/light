#include "ws2812.h"
#include "register/gpio_reg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lisa_mem.h"
#include <string.h>
#include "shell.h"

// 自定义代码段：置于 RES 资源区之后的 flash 末尾段，
// 避免 ws2812 代码挤占 .rodata，推移算法资源区导致启动复位。
#define WS2812_SECTION __attribute__((section(".ws2812code")))

// 时序关键代码段：放入 .fast_text（归入链接脚本 .cache 段）。
// 启动时 x_icache_flush() 会把 .cache 段锁定在 I-cache 中，
// 此后该段代码取指永命中、无 flash 冷启动抖动，位时序完全确定。
#define WS2812_HOT __attribute__((section(".fast_text"), noinline))

// 实际主频查询函数（位于驱动库 drv 中）
extern uint32_t CRM_GetCpuFreq(void);

// 全局图像缓冲区 (256 个像素，768 字节)。
// 分配策略（经多轮二分实验确定）：
// 1. 不能用 .bss 静态数组 —— 会推移 .bss 布局，影响算法内存区，导致启动复位；
// 2. 不能在 boot 早期（main 开头）分配 —— 堆(osheap)此时状态未稳，引发延迟崩溃；
// 3. 不能放链接脚本专用段 —— 该段位于 .osheap（FreeRTOS 堆）起始处，会缩小堆，
//    启动即因堆不足疯狂复位；
// 4. 采用「惰性堆分配 + 分配前检查剩余堆」：仅在首次真正显示时申请 768 字节，
//    若剩余堆不足则放弃显示（只灭灯、绝不崩溃）。
// 注：唤醒词动作不再使用备份缓冲，故只需单缓冲 768 字节。
static ws2812_pixel_t *g_ws2812_buf = NULL;

// 坏点屏蔽表：灯板个别灯珠/走线损坏，固定异常点亮（如常亮绿）。
// 屏蔽后该索引始终输出全黑（0,0,0），避免影响图案显示。
// 索引 0 = 物理左下角（y 翻转后代码 y=0 对应物理底部）。
#define WS2812_BAD_PIXEL_COUNT  (1)
static const uint16_t s_bad_pixel_idx[WS2812_BAD_PIXEL_COUNT] = { 0 };

// GPIO 寄存器指针
static volatile GPIO_RegDef *g_gpio_reg = NULL;

// 位时序循环次数（初始化时由 CPUFREQ() 换算，适配 100M~300M 主频）
static uint32_t s_delay_t1h;    // 1 码高电平
static uint32_t s_delay_t1l;    // 1 码低电平
static uint32_t s_delay_t0h;    // 0 码高电平
static uint32_t s_delay_t0l;    // 0 码低电平
static uint32_t s_delay_reset;  // 帧复位低电平

// 精细延时：每轮循环约 WS2812_LOOP_CYCLES 个 CPU 周期（-Os）
static void WS2812_HOT delay_cycles(volatile uint32_t count) {
    while (count--) {
        __asm__ volatile ("nop");
    }
}

// 将纳秒时长换算为 delay_cycles 的循环次数
static uint32_t WS2812_SECTION ws2812_ns_to_loops(uint32_t ns) {
    uint32_t f = CRM_GetCpuFreq();
    if (f < 1000000) f = 240000000u; // 时钟未就绪时的兜底值
    return (uint32_t)(((uint64_t)ns * f / 1000000000ULL) / WS2812_LOOP_CYCLES);
}

// 蛇形 (S型) 走线坐标映射函数。
// 注意：segtest 实测确认物理灯板 y=0 在底部（y 方向与代码相反），
// 故此处先把 y 翻转为 y' = 15 - y，再按蛇形寻址，保证图案方向与物理一致。
static uint16_t WS2812_SECTION xy_to_index(uint8_t x, uint8_t y) {
    if (x >= WS2812_WIDTH || y >= WS2812_HEIGHT) {
        return 0;
    }
    // 物理方向校正：代码顶部(y 小) -> 物理底部
    y = WS2812_HEIGHT - 1 - y;
    // 偶数行（0, 2, 4...）：从左往右 (0 -> 15)
    // 奇数行（1, 3, 5...）：从右往左 (15 -> 0)
    if (y % 2 == 0) {
        return y * WS2812_WIDTH + x;
    } else {
        return y * WS2812_WIDTH + (WS2812_WIDTH - 1 - x);
    }
}

// 发送 1 个字节数据到 WS2812B (GRB 顺序中单个字节)
// 1 码: 高 ~850ns, 低 ~400ns; 0 码: 高 ~280ns, 低 ~970ns
static void WS2812_HOT ws2812_send_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        if ((byte >> i) & 0x01) {
            // 1 码
            g_gpio_reg->REG_DOUTSET.all = WS2812_GPIO_PIN_MASK;
            delay_cycles(s_delay_t1h);
            g_gpio_reg->REG_DOUTCLEAR.all = WS2812_GPIO_PIN_MASK;
            delay_cycles(s_delay_t1l);
        } else {
            // 0 码
            g_gpio_reg->REG_DOUTSET.all = WS2812_GPIO_PIN_MASK;
            delay_cycles(s_delay_t0h);
            g_gpio_reg->REG_DOUTCLEAR.all = WS2812_GPIO_PIN_MASK;
            delay_cycles(s_delay_t0l);
        }
    }
}

// 惰性分配图像缓冲：仅在首次真正显示时申请 768 字节。
// 分配前检查剩余堆空间，不足则放弃（返回时缓冲为 NULL，调用方已做保护），绝不崩溃。
static void WS2812_SECTION ws2812_buf_ensure(void) {
    if (!g_ws2812_buf) {
        if (xPortGetFreeHeapSize() >= sizeof(ws2812_pixel_t) * WS2812_LEDS_NUM + 64u) {
            g_ws2812_buf = (ws2812_pixel_t *)lisa_mem_alloc(sizeof(ws2812_pixel_t) * WS2812_LEDS_NUM);
            if (g_ws2812_buf) memset(g_ws2812_buf, 0, sizeof(ws2812_pixel_t) * WS2812_LEDS_NUM);
        }
    }
}

// ===== RAM 热时序代码 =====
// 已废弃：热时序代码拷入堆 RAM 执行会与同处堆的像素缓冲/栈争抢 SRAM 端口，
// 抖动反而更严重。保留原 flash（.cache 段锁定）执行路径。

void WS2812_SECTION ws2812_init(void) {
    // 注意：本函数只做 GPIO 初始化，不做任何堆分配！
    // 二分实验确认 boot 早期调用 lisa_mem_alloc 会引发延迟崩溃（set vol 时复位），
    // 而仅做 GPIO 配置（下述 PinConfigure + GPIO_SetDir）已验证安全。

    // 1. 根据运行时主频换算各时序的延时循环次数
    s_delay_t1h   = ws2812_ns_to_loops(WS2812_T1H_NS);
    s_delay_t1l   = ws2812_ns_to_loops(WS2812_T1L_NS);
    s_delay_t0h   = ws2812_ns_to_loops(WS2812_T0H_NS);
    s_delay_t0l   = ws2812_ns_to_loops(WS2812_T0L_NS);
    s_delay_reset = ws2812_ns_to_loops((uint32_t)WS2812_RESET_US * 1000u);

    // 2. 配置 IOMUX 引脚复用为 GPIO
    IOMuxManager_PinConfigure(WS2812_GPIO_PAD, WS2812_GPIO_PIN, CSK_IOMUX_FUNC_DEFAULT);

    // 3. 设置方向为输出
    // 注意：不要调用 GPIO_Initialize()！该接口每次调用都会把 GPIO 驱动已注册的
    // ISR 回调覆盖为传入值（mod_gpio 启动时已注册 gpio_porta_isr），
    // 用 NULL 重复初始化会把系统中断回调清空，导致 GPIOA 中断触发时崩溃复位。
    // 参考 SDK 内 ls_app_msg.c 的可用写法：仅 PinConfigure + GPIO_SetDir。
    GPIO_SetDir(WS2812_GPIO_PORT(), WS2812_GPIO_PIN_MASK, CSK_GPIO_DIR_OUTPUT);

    // 5. 保存寄存器基地址
    GPIO_RESOURCES *gpio_res = (GPIO_RESOURCES *)WS2812_GPIO_PORT();
    g_gpio_reg = gpio_res->reg;

    // 拉低初始电平
    g_gpio_reg->REG_DOUTCLEAR.all = WS2812_GPIO_PIN_MASK;
}

void WS2812_SECTION ws2812_set_pixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
    if (x >= WS2812_WIDTH || y >= WS2812_HEIGHT) return;
    ws2812_buf_ensure(); // 惰性分配：首次使用时再申请（避免 boot 早期分配）
    if (!g_ws2812_buf) return;

    // 限制亮度，防止电流过大
    if (r > WS2812_BRIGHTNESS_MAX) r = WS2812_BRIGHTNESS_MAX;
    if (g > WS2812_BRIGHTNESS_MAX) g = WS2812_BRIGHTNESS_MAX;
    if (b > WS2812_BRIGHTNESS_MAX) b = WS2812_BRIGHTNESS_MAX;

    uint16_t idx = xy_to_index(x, y);
    g_ws2812_buf[idx].r = r;
    g_ws2812_buf[idx].g = g;
    g_ws2812_buf[idx].b = b;
}

// 全局中断屏蔽（RISC-V mstatus.MIE 位）。
// taskENTER_CRITICAL 只用 ECLIC 阈值屏蔽低优先级中断，
// 后台语音采集中断（I2S/PDM/DMA）优先级更高，仍会在 7.7ms 帧传输中途插入，
// 把"0"码高电平拉长成"1"，导致每帧随机位置/随机颜色的杂点。
// 故这里直接读写 mstatus，传输 + 复位期间屏蔽全部中断。
static inline uint32_t ws2812_mstatus_read(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mstatus" : "=r"(v));
    return v;
}

static inline void ws2812_mstatus_restore(uint32_t v) {
    __asm__ volatile ("csrw mstatus, %0" :: "r"(v));
}

#define WS2812_MSTATUS_MIE    (1u << 3)

// 实验：暂停/恢复麦克风采集（由 mod_aadc.c 提供），隔离音频 DMA 对位时序的干扰
extern void aadc_pause_capture(void);
extern void aadc_resume_capture(void);

void WS2812_HOT ws2812_refresh(void) {
    ws2812_buf_ensure(); // 惰性分配：首次使用时再申请（避免 boot 早期分配）
    if (!g_gpio_reg || !g_ws2812_buf) return;

    uint32_t mstatus = ws2812_mstatus_read();
    ws2812_mstatus_restore(mstatus & ~WS2812_MSTATUS_MIE);

    aadc_pause_capture(); // 全中断屏蔽窗口内停掉麦克风 DMA

    // 唤醒脉冲 + 复位：灯带长时间空闲（数据线保持低电平）后首像素容易误判，
    // 先发一个脉冲让各灯内部同步一次，再用 >280us 复位清零，之后首帧不再闪杂点。
    g_gpio_reg->REG_DOUTSET.all = WS2812_GPIO_PIN_MASK;
    delay_cycles(s_delay_t1h);
    g_gpio_reg->REG_DOUTCLEAR.all = WS2812_GPIO_PIN_MASK;
    delay_cycles(s_delay_reset);

    // 暖缓存：时序循环在 .cache 段（启动时锁定 I-cache），但为保险仍连续发两帧，
    // 第一帧预热分支预测/取指，最后一帧（实际显示帧）时序确定。
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < WS2812_LEDS_NUM; i++) {
            // WS2812B 发送顺序为 GRB (Green, Red, Blue)
            uint8_t r = g_ws2812_buf[i].r, g = g_ws2812_buf[i].g, b = g_ws2812_buf[i].b;
            // 坏点屏蔽：损坏灯珠强制输出全黑
            for (int k = 0; k < WS2812_BAD_PIXEL_COUNT; k++) {
                if (i == s_bad_pixel_idx[k]) { r = 0; g = 0; b = 0; break; }
            }
            ws2812_send_byte(g);
            ws2812_send_byte(r);
            ws2812_send_byte(b);
        }

        // 复位信号: 低电平维持 >= 280us（全中断屏蔽执行，等效于临界区内）
        g_gpio_reg->REG_DOUTCLEAR.all = WS2812_GPIO_PIN_MASK;
        delay_cycles(s_delay_reset);
    }

    aadc_resume_capture(); // 恢复麦克风采集

    ws2812_mstatus_restore(mstatus);
}

void WS2812_SECTION ws2812_clear(void) {
    ws2812_buf_ensure();
    if (!g_ws2812_buf) return;
    memset(g_ws2812_buf, 0, sizeof(ws2812_pixel_t) * WS2812_LEDS_NUM);
    ws2812_refresh();
}

// 满屏填充指定颜色（WS2812B 满屏实心色块）
static void WS2812_SECTION ws2812_fill_color(uint8_t r, uint8_t g, uint8_t b) {
    ws2812_buf_ensure();
    if (!g_ws2812_buf) return;
    for (uint8_t y = 0; y < WS2812_HEIGHT; y++) {
        for (uint8_t x = 0; x < WS2812_WIDTH; x++) {
            ws2812_set_pixel(x, y, r, g, b);
        }
    }
    ws2812_refresh();
}

// 1. 命令词1：大红 (da4 hong2) -> 满屏红色实心（YOLO 识别用大面积纯色块）
void WS2812_SECTION ws2812_show_red_light(void) {
    ws2812_fill_color(6, 0, 0); // 红色满屏
}

void WS2812_SECTION ws2812_show_white(void) {
    ws2812_buf_ensure();
    if (!g_ws2812_buf) return;
    for (uint8_t y = 0; y < WS2812_HEIGHT; y++) {
        for (uint8_t x = 0; x < WS2812_WIDTH; x++) {
            ws2812_set_pixel(x, y, WS2812_BRIGHTNESS_MAX, WS2812_BRIGHTNESS_MAX, WS2812_BRIGHTNESS_MAX);
        }
    }
    ws2812_refresh();
}

// 按形状绘制实心箭头（颜色可指定），shape: 1=直行 2=左转 3=右转。
// 三种形状均为 16x16 大实心箭头，四周留 1px 黑边，YOLO 识别用。
// 直行: 三角形 y=1 宽2[7,8] 每行+2 至 y=7 宽14[1,14]；竖杆 x=6..9 宽4 y=8..15
// 左转: 三角形顶点朝左，x=1 宽2[7,8] 每列+2 至 x=7 宽14[1,14]；横杆 y=6..9 宽4 x=8..15
// 右转: 三角形顶点朝右，x=14 宽2[7,8] 每列-2 至 x=8 宽14[1,14]；横杆 y=6..9 宽4 x=0..7
static void WS2812_SECTION ws2812_draw_shape(uint8_t shape, uint8_t r, uint8_t g, uint8_t b) {
    ws2812_buf_ensure();
    if (!g_ws2812_buf) return;
    memset(g_ws2812_buf, 0, sizeof(ws2812_pixel_t) * WS2812_LEDS_NUM);

    if (shape == 1) {
        for (int y = 1; y <= 7; y++) {
            int left  = 8 - y;
            int right = 7 + y;
            for (int x = left; x <= right; x++) ws2812_set_pixel(x, y, r, g, b);
        }
        for (int y = 8; y <= 15; y++) {
            for (int x = 6; x <= 9; x++) ws2812_set_pixel(x, y, r, g, b);
        }
    } else if (shape == 2) {
        for (int x = 1; x <= 7; x++) {
            int top    = 8 - x;
            int bottom = 6 + x;
            for (int y = top; y <= bottom; y++) ws2812_set_pixel(x, y, r, g, b);
        }
        for (int y = 6; y <= 9; y++) {
            for (int x = 8; x <= 15; x++) ws2812_set_pixel(x, y, r, g, b);
        }
    } else if (shape == 3) {
        for (int x = 8; x <= 14; x++) {
            int top    = x - 7;
            int bottom = 23 - x;
            for (int y = top; y <= bottom; y++) ws2812_set_pixel(x, y, r, g, b);
        }
        for (int y = 6; y <= 9; y++) {
            for (int x = 0; x <= 7; x++) ws2812_set_pixel(x, y, r, g, b);
        }
    }

    ws2812_refresh();
}

// 2. 命令词2：直行 (zhi2 xing2) -> 绿色大实心向上箭头
void WS2812_SECTION ws2812_show_straight(void) {
    ws2812_draw_shape(1, 0, 6, 0);
}

// 3. 命令词3：左转 (zuo3 zhuan3) -> 绿色大实心向左箭头
void WS2812_SECTION ws2812_show_left_turn(void) {
    ws2812_draw_shape(2, 0, 6, 0);
}

// 4. 命令词4：右转 (you4 zhuan3) -> 绿色大实心向右箭头
void WS2812_SECTION ws2812_show_right_turn(void) {
    ws2812_draw_shape(3, 0, 6, 0);
}

// 唤醒词响应动作：暂不显示（no-op）。
// 原因：唤醒路径堆余量仅 ~2KB，此动作原需备份缓冲 768B + 多次 vTaskDelay，
// 是堆/栈双重崩溃的高发点。命令词显示（红色/直行/左转/右转）不受影响。
void WS2812_SECTION ws2812_show_wakeup(void) {
    return;
}

// ===== 交通灯定时切换状态机（红->绿->黄->红，每 1900ms 切换一次相位） =====
#define WS2812_TRAFFIC_INTERVAL_MS  (1900u)

static volatile bool    s_tl_running   = false;   // 是否已开始循环（由「显示红灯」开启）
static volatile uint8_t s_tl_phase     = 0;       // 0=红灯 1=绿灯 2=黄灯
static volatile uint8_t s_tl_shape     = 0;       // 0=满屏色块 1=直行 2=左转 3=右转（粘滞，可说新命令词更新）
static volatile uint16_t s_tl_phase_ms = 0;       // 当前相位已累计毫秒数

// 按当前相位渲染一帧
static void WS2812_SECTION ws2812_tl_render_phase(void) {
    switch (s_tl_phase) {
    case 0:
        ws2812_show_red_light();                                    // 红灯：满屏红
        break;
    case 1:
        if (s_tl_shape) ws2812_draw_shape(s_tl_shape, 0, 6, 0);     // 绿灯：当前形状箭头
        else            ws2812_fill_color(0, 6, 0);                 //       或满屏绿
        break;
    case 2:
        if (s_tl_shape) ws2812_draw_shape(s_tl_shape, 6, 6, 0);     // 黄灯：当前形状箭头
        else            ws2812_fill_color(6, 6, 0);                 //       或满屏黄
        break;
    }
}

void WS2812_SECTION ws2812_traffic_start(void) {
    s_tl_running   = true;
    s_tl_phase     = 0;
    s_tl_phase_ms  = 0;
    ws2812_tl_render_phase();   // 立即显示红灯并从红灯开始计时循环
}

void WS2812_SECTION ws2812_traffic_set_shape(uint8_t shape) {
    s_tl_shape = shape;         // 粘滞记忆：后续所有绿/黄阶段沿用
    if (s_tl_running) ws2812_tl_render_phase();   // 再说新命令词时当前相位立即更新
}

void WS2812_SECTION ws2812_traffic_tick(uint32_t dt_ms) {
    if (!s_tl_running) return;
    s_tl_phase_ms += dt_ms;
    if (s_tl_phase_ms >= WS2812_TRAFFIC_INTERVAL_MS) {
        s_tl_phase_ms = 0;
        s_tl_phase = (s_tl_phase + 1u) % 3u;
        ws2812_tl_render_phase();
    }
}

// ===== 走线诊断：逐行点亮，确认物理灯板行列映射 =====
// 触发后依次点亮 16 行（y=0..15），每行亮 500ms 后自动下一行。
// 观察物理屏幕上哪一行先亮，即可确定 y 方向与蛇形是否正确。
void ws2812_scan_rows(void)
{
    ws2812_buf_ensure();
    if (!g_ws2812_buf) return;
    for (uint8_t row = 0; row < WS2812_HEIGHT; row++) {
        memset(g_ws2812_buf, 0, sizeof(ws2812_pixel_t) * WS2812_LEDS_NUM);
        for (uint8_t x = 0; x < WS2812_WIDTH; x++) {
            ws2812_set_pixel(x, row, 6, 0, 0); // 红色点亮整行
        }
        ws2812_refresh();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ws2812_clear();
}

void shell_ws2812_scan_rows(void)
{
    SHELL_ITEM_EXPORT("scanrows", shell_ws2812_scan_rows, "scan 16 rows one by one (500ms each)");
    ws2812_scan_rows();
}

void shell_ws2812_show_white(void)
{
    SHELL_ITEM_EXPORT("wswhite", shell_ws2812_show_white, "fill 16x16 full white (test noise)");
    ws2812_show_white();
}

void shell_ws2812_clear(void)
{
    SHELL_ITEM_EXPORT("wsclear", shell_ws2812_clear, "clear ws2812 all off");
    ws2812_clear();
}
