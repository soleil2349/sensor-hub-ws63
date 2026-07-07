/**
 * BMD101 ECG driver — Hardware UART2 + NeuroSky ThinkGear protocol
 *
 * ===== 新板 V3 PCB（引脚已交换，硬件 UART 方向正确）=====
 *   BMD101 TX → MCU GPIO7 (Pin12) = UART2_RXD  硬件接收
 *   BMD101 RX → MCU GPIO8 (Pin13) = UART2_TXD  硬件发送
 *
 * 使用硬件 UART2（PIN_MODE_2），替代之前的 GPIO bit-bang 软件 UART。
 * 优势：零 CPU 占用接收、无校验失败、无时序敏感问题。
 *
 * BMD101 协议（57600, 8N1）：
 *   [0xAA][0xAA][PLENGTH][TLV payload][CHKSUM]
 *   Code 0x02  POOR_SIGNAL: 1字节（0=最好，200=无接触）
 *   Code 0x03  HEART_RATE : 1字节 BPM
 *   Code 0x80  RAW_ECG    : 长度字节 + int16 大端，512SPS
 */

#include "bmd101_drv.h"
#include "bio_pkt.h"
#include "errcode.h"
#include "pinctrl.h"
#include "uart.h"
#include "soc_osal.h"
#include <string.h>

/* ── 硬件 UART2 配置 ──────────────────────────────────────────── */
#define BMD_UART_BUS    UART_BUS_2
#define BMD_RX_PIN      7u              /* GPIO7 (Pin12) = UART2_RXD */
#define BMD_TX_PIN      8u              /* GPIO8 (Pin13) = UART2_TXD */
#define BMD_PIN_MODE    PIN_MODE_2
#define BMD_BAUD        57600u

/* UART2 硬件 RX 缓冲区（DMA/中断用） */
#define HW_RX_BUF_SIZE 256u
static uint8_t g_hw_rx_buf[HW_RX_BUF_SIZE];
static uart_buffer_config_t g_uart_buf_cfg = {
    .rx_buffer      = g_hw_rx_buf,
    .rx_buffer_size = HW_RX_BUF_SIZE
};

/* ── 接收 ring buffer ─────────────────────────────────────────── */
#define RING_SIZE   2048u

static uint8_t   g_ring[RING_SIZE];
static volatile uint16_t g_ring_head = 0;
static volatile uint16_t g_ring_tail = 0;

static volatile uint32_t g_uart_rx_total = 0;
static volatile uint32_t g_uart_rx_error = 0;
static volatile uint32_t g_uart_rx_drop  = 0;

/* 诊断缓冲区 */
static uint8_t  g_first16[16];
static uint8_t  g_first16_cnt = 0;
static uint8_t  g_last16[16];
static uint8_t  g_last16_idx = 0;
static volatile uint32_t g_byte_aa = 0;
static volatile uint32_t g_byte_55 = 0;

#define TRACE_N  64u
static uint8_t  g_trace[TRACE_N];
static volatile uint16_t g_trace_idx  = 0;
static volatile uint8_t  g_trace_done = 0;

static inline void ring_put_byte(uint8_t byte)
{
    g_uart_rx_total++;
    if (byte == 0xAAu) { g_byte_aa++; }
    if (byte == 0x55u) { g_byte_55++; }
    if (!g_trace_done && g_uart_rx_total > 5000u) {
        g_trace[g_trace_idx] = byte;
        if (++g_trace_idx >= TRACE_N) { g_trace_done = 1; }
    }
    g_last16[g_last16_idx] = byte;
    g_last16_idx = (g_last16_idx + 1u) & 0x0Fu;
    if (g_first16_cnt < 16u) {
        g_first16[g_first16_cnt++] = byte;
    }
    uint16_t next = (g_ring_head + 1u) % RING_SIZE;
    if (next != g_ring_tail) {
        g_ring[g_ring_head] = byte;
        g_ring_head = next;
    } else {
        g_uart_rx_drop++;
    }
}

/* ── UART2 RX 回调（中断上下文，将字节送入 ring buffer）──────── */
static void uart2_rx_callback(const void *buffer, uint16_t length, bool error)
{
    if (error) { g_uart_rx_error++; return; }
    if (buffer == NULL || length == 0u) { return; }
    const uint8_t *data = (const uint8_t *)buffer;
    for (uint16_t i = 0; i < length; i++) {
        ring_put_byte(data[i]);
    }
}

static int ring_read_byte(uint8_t *out)
{
    if (g_ring_tail == g_ring_head) { return 0; }
    *out = g_ring[g_ring_tail];
    g_ring_tail = (g_ring_tail + 1u) % RING_SIZE;
    return 1;
}

/* ── UART2 TX（硬件发送到 BMD101 RX）──────────────────────────── */
void bmd101_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u) { return; }
    uapi_uart_write(BMD_UART_BUS, data, len, 0);
}

/* ── ThinkGear 协议解析 ───────────────────────────────────────── */
typedef enum { TG_SYNC1, TG_SYNC2, TG_PLEN, TG_DATA, TG_CHKSUM } tg_st_t;

static tg_st_t s_st      = TG_SYNC1;
static uint8_t s_pl[20];
static uint8_t s_plen    = 0;
static uint8_t s_poff    = 0;
static uint8_t s_chk     = 0;

static bmd101_result_t s_result;
static uint32_t s_chk_fail = 0;
static uint32_t s_sig_update = 0;
static uint32_t s_hr_update  = 0;

/* ── ECG R 波检测（峰值搜索窗口 + 自适应阈值 + 斜率检查 + RR 一致性）── */
#define ECG_REFRACTORY      300u   /* ~586ms @512SPS 不应期，跳过 P/T 波 */
#define ECG_NO_PEAK_TIMEOUT 2560u  /* ~5s @512SPS 无 R-peak → 信号丢失 */
#define PEAK_SEARCH_WIN     64u    /* ~125ms @512SPS 搜索窗口 */
#define RR_BUF_N            4u
#define RR_VAR_LIMIT        30u
#define MIN_SLOPE           30     /* R 波上升斜率最小值 (per sample) */

static uint32_t g_ecg_sample_cnt = 0;
static uint32_t g_rpeak_sample   = 0;
static int32_t  g_ecg_baseline   = 0;
static int32_t  g_ecg_peak_dev   = 1000;
static uint8_t  g_ecg_bl_init    = 0;

static uint8_t  g_in_search      = 0;
static uint32_t g_search_start   = 0;
static int32_t  g_search_max_dev = 0;
static uint32_t g_search_max_pos = 0;
static int16_t  g_ecg_prev_val   = 0;
static int32_t  g_search_max_slope = 0;

static uint32_t g_rr_buf[RR_BUF_N];
static uint8_t  g_rr_idx = 0;
static uint8_t  g_rr_cnt = 0;

static uint32_t rr_median4(void)
{
    if (g_rr_cnt < 2u) { return g_rr_buf[0]; }
    uint32_t tmp[RR_BUF_N];
    uint8_t n = (g_rr_cnt < RR_BUF_N) ? g_rr_cnt : RR_BUF_N;
    for (uint8_t i = 0; i < n; i++) { tmp[i] = g_rr_buf[i]; }
    for (uint8_t i = 0; i < n - 1u; i++) {
        for (uint8_t j = i + 1u; j < n; j++) {
            if (tmp[j] < tmp[i]) { uint32_t t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
        }
    }
    return (n & 1u) ? tmp[n / 2u] : (tmp[n / 2u - 1u] + tmp[n / 2u]) / 2u;
}

static void register_rpeak(int32_t dev)
{
    uint32_t since = g_search_max_pos - g_rpeak_sample;

    if (g_rpeak_sample > 0u && since < ECG_NO_PEAK_TIMEOUT) {
        g_rr_buf[g_rr_idx] = since;
        g_rr_idx = (g_rr_idx + 1u) % RR_BUF_N;
        if (g_rr_cnt < RR_BUF_N) { g_rr_cnt++; }

        if (g_rr_cnt >= 3u) {
            uint32_t med = rr_median4();
            uint8_t consistent = 1u;
            uint8_t n = (g_rr_cnt < RR_BUF_N) ? g_rr_cnt : RR_BUF_N;
            for (uint8_t j = 0; j < n; j++) {
                uint32_t d = (g_rr_buf[j] > med)
                             ? (g_rr_buf[j] - med) : (med - g_rr_buf[j]);
                if (d * 100u > med * RR_VAR_LIMIT) {
                    consistent = 0u;
                    break;
                }
            }
            if (consistent && med > 0u) {
                uint32_t hr = (60u * 512u) / med;
                if (hr >= 30u && hr <= 220u) {
                    s_result.heart_rate  = (uint8_t)hr;
                    s_result.poor_signal = 0u;
                }
            } else {
                s_result.poor_signal = 200u;
                s_result.heart_rate  = 0u;
            }
        }
    }

    g_rpeak_sample = g_search_max_pos;
    if (dev >= g_ecg_peak_dev) {
        g_ecg_peak_dev = dev;
    } else {
        g_ecg_peak_dev = g_ecg_peak_dev - (g_ecg_peak_dev >> 3) + (dev >> 3);
    }
    s_result.rpeak_tick_ms = g_bio_ms;
    s_result.rpeak_new     = 1u;
}

static void detect_rpeak(int16_t val)
{
    g_ecg_sample_cnt++;

    if (!g_ecg_bl_init) {
        g_ecg_baseline = (int32_t)val;
        g_ecg_bl_init  = 1;
    } else {
        g_ecg_baseline += ((int32_t)val - g_ecg_baseline) >> 9;
    }

    int32_t dev   = (int32_t)val - g_ecg_baseline;
    int32_t slope = (int32_t)val - (int32_t)g_ecg_prev_val;
    g_ecg_prev_val = val;

    if (g_in_search) {
        if (slope > g_search_max_slope) {
            g_search_max_slope = slope;
        }
        if (dev > g_search_max_dev) {
            g_search_max_dev = dev;
            g_search_max_pos = g_ecg_sample_cnt;
        }
        if ((g_ecg_sample_cnt - g_search_start) >= PEAK_SEARCH_WIN) {
            g_in_search = 0;
            int32_t slope_thr = g_ecg_peak_dev >> 5;
            if (slope_thr < MIN_SLOPE) { slope_thr = MIN_SLOPE; }
            if (g_search_max_slope >= slope_thr) {
                register_rpeak(g_search_max_dev);
            }
        }
    } else {
        uint32_t since = g_ecg_sample_cnt - g_rpeak_sample;
        int32_t thr = g_ecg_peak_dev >> 1;
        if (thr < 400) { thr = 400; }

        if (dev > thr && since > ECG_REFRACTORY) {
            g_in_search        = 1;
            g_search_start     = g_ecg_sample_cnt;
            g_search_max_dev   = dev;
            g_search_max_pos   = g_ecg_sample_cnt;
            g_search_max_slope = slope;
        }

        if (since > ECG_NO_PEAK_TIMEOUT) {
            s_result.poor_signal = 200u;
            s_result.heart_rate  = 0u;
        }
    }
}

/* ── 50Hz + 100Hz IIR 陷波滤波器（定点 Q14，512 SPS）──────────
 *
 * 2nd-order IIR notch:  y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2]
 *                                - a1·y[n-1] - a2·y[n-2]
 *
 * 对于陷波滤波器 b1 == a1，b0 == b2，系数通过 Python scipy
 * iirnotch(f0, Q=25, fs=512) 计算后乘 16384 取整。
 * 使用 int64 累加器避免 32 位溢出。
 */
#define NOTCH_SHIFT  14

/* 50 Hz 陷波（Q=25, fs=512, scipy.signal.iirnotch 验证） */
#define N50_B0   16197
#define N50_B1  (-26486)
#define N50_A2   16011

/* 100 Hz 二次谐波陷波（Q=25, fs=512） */
#define N100_B0  16081
#define N100_B1 (-10835)
#define N100_A2  15778

typedef struct {
    int32_t x1, x2;
    int32_t y1, y2;
} notch_t;

static notch_t s_n50  = {0};
static notch_t s_n100 = {0};

static int16_t notch_apply(notch_t *st, int16_t in,
                           int32_t b0, int32_t b1, int32_t a2)
{
    int32_t xn = (int32_t)in;
    int64_t acc = (int64_t)b0 * xn
                + (int64_t)b1 * st->x1
                + (int64_t)b0 * st->x2      /* b2 == b0 */
                - (int64_t)b1 * st->y1       /* a1 == b1 */
                - (int64_t)a2 * st->y2;
    int32_t yn = (int32_t)(acc >> NOTCH_SHIFT);
    if (yn > 32767)  { yn = 32767; }
    if (yn < -32768) { yn = -32768; }
    st->x2 = st->x1; st->x1 = xn;
    st->y2 = st->y1; st->y1 = yn;
    return (int16_t)yn;
}

/* ── 0.5Hz 1阶高通（DC blocker，去除基线漂移）──────────────────
 *   y[n] = x[n] - x[n-1] + α·y[n-1]
 *   α = 0.99386 → fc ≈ 0.5 Hz @ 512 SPS
 *   Q14: round(0.99386 × 16384) = 16284
 */
#define HPF_ALPHA  16284

static int32_t s_hpf_x1 = 0;
static int32_t s_hpf_y1 = 0;

static int16_t ecg_filter(int16_t raw)
{
    int16_t v = notch_apply(&s_n50,  raw, N50_B0,  N50_B1,  N50_A2);
    v         = notch_apply(&s_n100, v,   N100_B0, N100_B1, N100_A2);

    /* 高通：去除 DC 和基线漂移 */
    int32_t xn = (int32_t)v;
    int32_t yn = xn - s_hpf_x1 + ((int64_t)HPF_ALPHA * s_hpf_y1 >> NOTCH_SHIFT);
    if (yn > 32767)  { yn = 32767; }
    if (yn < -32768) { yn = -32768; }
    s_hpf_x1 = xn;
    s_hpf_y1 = yn;

    return (int16_t)yn;
}

/* ── ECG 流式输出缓冲区（供 SLE 传输使用）──────────────────── */
#define ECG_STREAM_SIZE 64u
static int16_t  g_ecg_stream[ECG_STREAM_SIZE];
static volatile uint8_t g_ecg_stream_head = 0;
static volatile uint8_t g_ecg_stream_tail = 0;
static uint8_t s_ecg_subsamp = 0;

uint8_t bmd101_read_ecg_stream(int16_t *buf, uint8_t max_count)
{
    uint8_t n = 0;
    while (n < max_count && g_ecg_stream_tail != g_ecg_stream_head) {
        buf[n++] = g_ecg_stream[g_ecg_stream_tail];
        g_ecg_stream_tail = (g_ecg_stream_tail + 1u) % ECG_STREAM_SIZE;
    }
    return n;
}

static void process_payload(const uint8_t *pl, uint8_t len)
{
    uint8_t i = 0;
    while (i < len) {
        uint8_t code = pl[i++];
        if (code < 0x80u) {
            if (i >= len) { break; }
            uint8_t val = pl[i++];
            switch (code) {
            case 0x02u:
                if (val <= 200u) { s_result.poor_signal = val; }
                s_sig_update++;
                break;
            case 0x03u:
                if (val <= 220u) { s_result.heart_rate = val; }
                s_hr_update++;
                break;
            default: break;
            }
        } else {
            if (i >= len) { break; }
            uint8_t vlen = pl[i++];
            if (code == 0x80u && vlen >= 2u && (i + 2u) <= len) {
                int16_t raw = (int16_t)(((uint16_t)pl[i] << 8) | pl[i + 1u]);
                int16_t filt = ecg_filter(raw);

                s_result.raw_ecg = raw;
                detect_rpeak(filt);

                /* 滤波后 ECG 串口输出（512 SPS） */
                osal_printk("E,%d\r\n", (int)filt);

                /* 2:1 降采样送入 SLE 流式缓冲区 */
                if (++s_ecg_subsamp >= 2u) {
                    s_ecg_subsamp = 0;
                    uint8_t nx = (g_ecg_stream_head + 1u) % ECG_STREAM_SIZE;
                    if (nx != g_ecg_stream_tail) {
                        g_ecg_stream[g_ecg_stream_head] = filt;
                        g_ecg_stream_head = nx;
                    }
                }
            }
            i += vlen;
        }
    }
    s_result.pkt_count++;
}

static void tg_feed(uint8_t b)
{
    s_result.byte_count++;
    switch (s_st) {
    case TG_SYNC1:
        if (b == 0xAAu) { s_st = TG_SYNC2; }
        break;
    case TG_SYNC2:
        s_st = (b == 0xAAu) ? TG_PLEN : TG_SYNC1;
        break;
    case TG_PLEN:
        if (b == 0xAAu) { break; }
        if (b > 16u)    { s_st = TG_SYNC1; break; }
        s_plen = b; s_poff = 0; s_chk = 0;
        s_st = (b > 0u) ? TG_DATA : TG_CHKSUM;
        break;
    case TG_DATA:
        s_pl[s_poff++] = b;
        s_chk = (uint8_t)(s_chk + b);
        if (s_poff >= s_plen) { s_st = TG_CHKSUM; }
        break;
    case TG_CHKSUM:
        if (((~s_chk) & 0xFFu) == b) {
            process_payload(s_pl, s_plen);
        } else {
            s_chk_fail++;
        }
        s_st = TG_SYNC1;
        break;
    default:
        s_st = TG_SYNC1;
        break;
    }
}

/* ── 公共 API ─────────────────────────────────────────────────── */
int bmd101_init(void)
{
    memset(&s_result, 0, sizeof(s_result));
    s_result.poor_signal = 200u;
    g_uart_rx_total  = 0;
    g_uart_rx_error  = 0;
    g_uart_rx_drop   = 0;
    g_first16_cnt    = 0;
    g_last16_idx     = 0;
    memset(g_last16, 0, sizeof(g_last16));
    g_byte_aa        = 0;
    g_byte_55        = 0;
    g_trace_idx      = 0;
    g_trace_done     = 0;
    s_chk_fail       = 0;
    g_ecg_sample_cnt = 0;
    g_rpeak_sample   = 0;
    g_ecg_baseline   = 0;
    g_ecg_peak_dev   = 1000;
    g_ecg_bl_init    = 0;
    g_in_search        = 0;
    g_search_start     = 0;
    g_search_max_dev   = 0;
    g_search_max_pos   = 0;
    g_ecg_prev_val     = 0;
    g_search_max_slope = 0;
    g_rr_idx           = 0;
    g_rr_cnt         = 0;
    memset(g_rr_buf, 0, sizeof(g_rr_buf));
    g_ecg_stream_head = 0;
    g_ecg_stream_tail = 0;
    s_ecg_subsamp     = 0;
    memset(&s_n50,  0, sizeof(s_n50));
    memset(&s_n100, 0, sizeof(s_n100));
    s_hpf_x1 = 0;
    s_hpf_y1 = 0;

    /* GPIO7/GPIO8 切换为 UART2 功能（PIN_MODE_2） */
    uapi_pin_set_mode(BMD_RX_PIN, BMD_PIN_MODE);
    uapi_pin_set_mode(BMD_TX_PIN, BMD_PIN_MODE);

    /* 初始化 UART2：57600 8N1 */
    uart_attr_t attr = {
        .baud_rate = BMD_BAUD,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity    = UART_PARITY_NONE
    };
    uart_pin_config_t pins = {
        .tx_pin  = BMD_TX_PIN,
        .rx_pin  = BMD_RX_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };

    uapi_uart_deinit(BMD_UART_BUS);
    errcode_t ret = uapi_uart_init(BMD_UART_BUS, &pins, &attr,
                                    NULL, &g_uart_buf_cfg);
    if (ret != ERRCODE_SUCC) {
        osal_printk("[BMD] UART2 init FAILED ret=0x%x\r\n", ret);
        return -1;
    }

    /* 注册 RX 回调（空闲或满时触发） */
    ret = uapi_uart_register_rx_callback(BMD_UART_BUS,
              UART_RX_CONDITION_FULL_OR_IDLE, 1, uart2_rx_callback);
    if (ret != ERRCODE_SUCC) {
        osal_printk("[BMD] UART2 rx_callback reg FAILED ret=0x%x\r\n", ret);
        return -1;
    }

    osal_printk("[BMD] HW-UART2 init OK: 57600 8N1\r\n");
    osal_printk("[BMD] RX=GPIO7(Pin12) TX=GPIO8(Pin13) PIN_MODE_2\r\n");
    osal_printk("[BMD] Waiting for 0xAA 0xAA sync from BMD101...\r\n");
    return 0;
}

void bmd101_poll(void)
{
    uint8_t b;
    uint16_t cnt = 0;
    while (ring_read_byte(&b) && cnt < 800u) {
        tg_feed(b);
        cnt++;
    }
}

void bmd101_get_result(bmd101_result_t *out)
{
    if (out) {
        *out = s_result;
        out->uart_rx_total = g_uart_rx_total;
        out->uart_rx_error = g_uart_rx_error;
        out->chk_fail      = s_chk_fail;
        out->sig_update    = (uint16_t)(s_sig_update & 0xFFFFu);
        out->hr_update     = (uint16_t)(s_hr_update  & 0xFFFFu);
        s_result.rpeak_new = 0;
    }
}

void bmd101_print_diag(void)
{
    osal_printk("[BMD] uart_rx=%u err=%u drop=%u ring=%u/%u\r\n",
                (unsigned)g_uart_rx_total, (unsigned)g_uart_rx_error,
                (unsigned)g_uart_rx_drop,
                (unsigned)g_ring_head, (unsigned)g_ring_tail);
    osal_printk("[BMD] parser: byte=%u ok=%u chk_fail=%u ps=%u hr=%u\r\n",
                (unsigned)s_result.byte_count, (unsigned)s_result.pkt_count,
                (unsigned)s_chk_fail,
                (unsigned)s_result.poor_signal, (unsigned)s_result.heart_rate);
    osal_printk("[BMD] sig_upd=%u hr_upd=%u ecg_samp=%u\r\n",
                (unsigned)s_sig_update, (unsigned)s_hr_update,
                (unsigned)g_ecg_sample_cnt);

    osal_printk("[BMD] 0xAA=%u 0x55=%u\r\n",
                (unsigned)g_byte_aa, (unsigned)g_byte_55);

    if (g_trace_done) {
        osal_printk("[BMD] TRACE 64 bytes:\r\n");
        for (uint8_t r = 0; r < 4u; r++) {
            osal_printk("[BMD] T");
            for (uint8_t c = 0; c < 16u; c++) {
                osal_printk(" %02X", g_trace[r * 16u + c]);
            }
            osal_printk("\r\n");
        }
    }

    if (g_first16_cnt > 0u) {
        osal_printk("[BMD] first16:");
        for (uint8_t i = 0; i < g_first16_cnt; i++) {
            osal_printk(" %02X", g_first16[i]);
        }
        osal_printk("\r\n");
    }

    if (g_uart_rx_total > 0u) {
        osal_printk("[BMD] last16:");
        for (uint8_t i = 0; i < 16u; i++) {
            osal_printk(" %02X", g_last16[(g_last16_idx + i) & 0x0Fu]);
        }
        osal_printk("\r\n");
    }
}
