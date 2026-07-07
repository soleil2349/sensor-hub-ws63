/**
 * max30102_drv.c — MAX30102 硬件 I2C1 驱动（GPIO15=SDA, GPIO16=SCL）
 *
 * ===== 新板 V2 接线（PCB 直连，无飞线）=====
 *   MCU Pin21 GPIO15 (I2C1_SDA) → MAX30102 模块 SDA (pin2) + 2.2K 上拉
 *   MCU Pin22 GPIO16 (I2C1_SCL) → MAX30102 模块 SCL (pin3) + 2.2K 上拉
 *   MAX30102 VIN (pin1) = 3.3V
 *   MAX30102 GND (pin4) = GND
 *   MAX30102 INT (pin5) = 3.3V 或接 GPIO（可选中断驱动）
 *
 * MAX30102 I2C 地址：0x57 (7-bit)
 * 使用硬件 I2C1，PIN_MODE_2，400kHz Fast Mode
 */

#include "max30102_drv.h"
#include "bio_pkt.h"
#include "pinctrl.h"
#include "i2c.h"
#include "soc_osal.h"
#include <string.h>

/* ── 硬件 I2C1 配置 ──────────────────────────────────────────── */
#define I2C_BUS_ID    I2C_BUS_1
#define I2C_SCL_PIN   16           /* GPIO16 = I2C1_SCL (Pin 22) */
#define I2C_SDA_PIN   15           /* GPIO15 = I2C1_SDA (Pin 21) */
#define I2C_PIN_MODE  PIN_MODE_2
#define I2C_BAUDRATE  400000u      /* 400kHz Fast Mode */

/* ── MAX30102 I2C 地址（7-bit）────────────────────────────────── */
#define MAX_I2C_ADDR  0x57u

/* ── MAX30102 寄存器定义 ──────────────────────────────────────── */
#define REG_INTR_STATUS1  0x00u
#define REG_INTR_STATUS2  0x01u
#define REG_INTR_ENABLE1  0x02u
#define REG_INTR_ENABLE2  0x03u
#define REG_FIFO_WR_PTR   0x04u
#define REG_OVF_COUNTER   0x05u
#define REG_FIFO_RD_PTR   0x06u
#define REG_FIFO_DATA     0x07u
#define REG_FIFO_CONFIG   0x08u
#define REG_MODE_CONFIG   0x09u
#define REG_SPO2_CONFIG   0x0Au
#define REG_LED1_PA       0x0Cu
#define REG_LED2_PA       0x0Du
#define REG_TEMP_INTR     0x1Fu
#define REG_TEMP_FRAC     0x20u
#define REG_TEMP_CONFIG   0x21u
#define REG_PART_ID       0xFFu

/* ── MAX30102 寄存器读写（硬件 I2C1）─────────────────────────── */

static int max_wreg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_data_t data = { .send_buf = buf, .send_len = 2,
                        .receive_buf = NULL, .receive_len = 0 };
    return (uapi_i2c_master_write(I2C_BUS_ID, MAX_I2C_ADDR, &data)
            == ERRCODE_SUCC) ? 0 : -1;
}

static int max_rreg(uint8_t reg, uint8_t *val)
{
    i2c_data_t data;
    data.send_buf    = &reg;
    data.send_len    = 1;
    data.receive_buf = NULL;
    data.receive_len = 0;
    if (uapi_i2c_master_write(I2C_BUS_ID, MAX_I2C_ADDR, &data) != ERRCODE_SUCC) {
        return -1;
    }
    data.send_buf    = NULL;
    data.send_len    = 0;
    data.receive_buf = val;
    data.receive_len = 1;
    return (uapi_i2c_master_read(I2C_BUS_ID, MAX_I2C_ADDR, &data)
            == ERRCODE_SUCC) ? 0 : -2;
}

static int max_rbytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len == 0u) { return 0; }
    i2c_data_t data;
    data.send_buf    = &reg;
    data.send_len    = 1;
    data.receive_buf = NULL;
    data.receive_len = 0;
    if (uapi_i2c_master_write(I2C_BUS_ID, MAX_I2C_ADDR, &data) != ERRCODE_SUCC) {
        return -1;
    }
    data.send_buf    = NULL;
    data.send_len    = 0;
    data.receive_buf = buf;
    data.receive_len = (uint32_t)len;
    return (uapi_i2c_master_read(I2C_BUS_ID, MAX_I2C_ADDR, &data)
            == ERRCODE_SUCC) ? 0 : -2;
}

/* ── 状态变量 ─────────────────────────────────────────────────── */
#define SAMPLE_BUF  32u

static uint32_t g_ir_buf[SAMPLE_BUF];
static uint32_t g_red_buf[SAMPLE_BUF];
static uint8_t  g_buf_head  = 0u;
static uint8_t  g_buf_cnt   = 0u;
static uint32_t g_total     = 0u;
static uint8_t  g_part_id   = 0u;
static int      g_init_ok   = 0;

static max30102_result_t g_result = {0};

/* ── 算法状态（DC/AC 滤波 + HR + SpO2 + PI + PPG foot）──────── */
#define ALGO_WARMUP   50u
#define HR_AVG_N      4u
#define HR_INT_MIN    8u
#define HR_INT_MAX    38u
#define SAMPLE_MS     40u

static int32_t  g_dc_ir  = 0;
static int32_t  g_dc_red = 0;
static uint8_t  g_dc_init = 0;
static int32_t  g_prev_ac_ir = 0;
static uint32_t g_algo_cnt   = 0;

static uint16_t g_hr_intervals[HR_AVG_N];
static uint8_t  g_hr_idx = 0;
static uint8_t  g_hr_cnt = 0;
static uint32_t g_last_zx = 0;

static int32_t g_pk_ir = 0, g_vl_ir = 0;
static int32_t g_pk_red = 0, g_vl_red = 0;
static int32_t g_cur_max_ir = 0, g_cur_max_red = 0;
static int32_t g_cur_min_ir = 0, g_cur_min_red = 0;

static uint32_t g_temp_timer   = 0;
static uint8_t  g_temp_pending = 0;
static uint8_t  g_was_finger   = 0;

static void algo_reset(void)
{
    g_dc_ir = 0; g_dc_red = 0; g_dc_init = 0;
    g_prev_ac_ir = 0; g_algo_cnt = 0;
    g_hr_idx = 0; g_hr_cnt = 0; g_last_zx = 0;
    g_pk_ir = 0; g_vl_ir = 0; g_pk_red = 0; g_vl_red = 0;
    g_cur_max_ir = 0; g_cur_max_red = 0;
    g_cur_min_ir = 0; g_cur_min_red = 0;
    g_result.hr = 0; g_result.spo2 = 0; g_result.pi_x10 = 0;
    g_result.rr_interval = 0;
    g_result.ppg_foot_ms = 0; g_result.ppg_foot_new = 0;
}

/* ── 初始化 ───────────────────────────────────────────────────── */
int max30102_init(void)
{
    g_init_ok = 0;
    (void)memset(&g_result, 0, sizeof(g_result));
    g_buf_head = 0u;
    g_buf_cnt  = 0u;
    g_total    = 0u;

    /* 1. 硬件 I2C1 初始化（GPIO15=SDA, GPIO16=SCL, PIN_MODE_2）*/
    uapi_pin_set_mode(I2C_SCL_PIN, I2C_PIN_MODE);
    uapi_pin_set_mode(I2C_SDA_PIN, I2C_PIN_MODE);
    uapi_pin_set_pull(I2C_SCL_PIN, PIN_PULL_TYPE_UP);
    uapi_pin_set_pull(I2C_SDA_PIN, PIN_PULL_TYPE_UP);

    if (uapi_i2c_master_init(I2C_BUS_ID, I2C_BAUDRATE, 0) != ERRCODE_SUCC) {
        osal_printk("[OPT] I2C1 master init FAILED\r\n");
        return -1;
    }
    osal_msleep(50);

    /* 2. 探测芯片（读 Part ID，expect 0x15）*/
    if (max_rreg(REG_PART_ID, &g_part_id) != 0) {
        osal_printk("[OPT] MAX30102 NACK (I2C1: SDA=GPIO15, SCL=GPIO16, addr=0x57)\r\n");
        return -1;
    }
    osal_printk("[OPT] MAX30102 Part ID = 0x%02X%s (HW I2C1 400kHz)\r\n",
                g_part_id, (g_part_id == 0x15u) ? " OK" : " unexpected!");

    /* 3. 软复位 */
    int wret = max_wreg(REG_MODE_CONFIG, 0x40u);
    osal_printk("[OPT] wreg RESET ret=%d\r\n", wret);
    osal_msleep(50);

    /* 4. 配置 SpO2 模式 */
    int e = 0;
    e |= max_wreg(REG_FIFO_CONFIG, 0x4Fu);
    e |= max_wreg(REG_MODE_CONFIG, 0x03u);
    e |= max_wreg(REG_SPO2_CONFIG, 0x67u);
    e |= max_wreg(REG_LED1_PA,     0xA0u);
    e |= max_wreg(REG_LED2_PA,     0xA0u);
    osal_printk("[OPT] wreg config ret=%d\r\n", e);

    /* 5. 清空 FIFO 指针 */
    e  = max_wreg(REG_FIFO_WR_PTR, 0x00u);
    e |= max_wreg(REG_OVF_COUNTER, 0x00u);
    e |= max_wreg(REG_FIFO_RD_PTR, 0x00u);
    osal_printk("[OPT] wreg FIFO clear ret=%d\r\n", e);

    /* 6. 使能 PPG_RDY 中断标志（轮询读取）*/
    e = max_wreg(REG_INTR_ENABLE1, 0x40u);
    osal_printk("[OPT] wreg INT_EN ret=%d\r\n", e);

    algo_reset();
    g_temp_timer = 0;
    g_temp_pending = 0;
    g_was_finger = 0;

    g_init_ok = 1;
    osal_printk("[OPT] MAX30102 init OK  HW-I2C1 GPIO15/16  SpO2 mode  LED=0xA0  SR=100Hz\r\n");
    return 0;
}

/* ── 轮询 FIFO ────────────────────────────────────────────────── */
int max30102_poll(void)
{
    if (!g_init_ok) { return -1; }

    uint8_t wr = 0u, rd = 0u, ovf = 0u;
    int rr1 = max_rreg(REG_FIFO_WR_PTR, &wr);
    int rr2 = max_rreg(REG_FIFO_RD_PTR, &rd);
    int rr3 = max_rreg(REG_OVF_COUNTER,  &ovf);
    if (rr1 != 0 || rr2 != 0 || rr3 != 0) { return -2; }

    static uint32_t s_poll_dbg = 0;
    if ((s_poll_dbg++ % 50u) == 0u) {
        osal_printk("[POLL] wr=%u rd=%u ovf=%u rr=%d/%d/%d\r\n",
                    wr, rd, ovf, rr1, rr2, rr3);
    }

    uint8_t nsamp;
    if (ovf > 0u) {
        (void)max_wreg(REG_OVF_COUNTER, 0x00u);
        nsamp = 32u;
    } else {
        nsamp = (uint8_t)((wr - rd + 32u) % 32u);
        if (nsamp == 0u) { return 0; }
    }
    if (nsamp > 8u) { nsamp = 8u; }

    uint8_t raw[6u * 8u];
    uint8_t rlen = (uint8_t)(nsamp * 6u);
    if (max_rbytes(REG_FIFO_DATA, raw, rlen) != 0) { return -3; }

    for (uint8_t i = 0u; i < nsamp; i++) {
        const uint8_t *p = raw + (i * 6u);
        uint32_t red = ((uint32_t)(p[0u] & 0x03u) << 16u) |
                       ((uint32_t)p[1u] << 8u) | p[2u];
        uint32_t ir  = ((uint32_t)(p[3u] & 0x03u) << 16u) |
                       ((uint32_t)p[4u] << 8u) | p[5u];

        g_ir_buf [g_buf_head] = ir;
        g_red_buf[g_buf_head] = red;
        g_buf_head = (uint8_t)((g_buf_head + 1u) % SAMPLE_BUF);
        if (g_buf_cnt < SAMPLE_BUF) { g_buf_cnt++; }
        g_total++;

        if (!g_dc_init) {
            g_dc_ir  = (int32_t)ir;
            g_dc_red = (int32_t)red;
            g_dc_init = 1;
        } else {
            g_dc_ir  += ((int32_t)ir  - g_dc_ir)  >> 5;
            g_dc_red += ((int32_t)red - g_dc_red) >> 5;
        }
        int32_t ac_ir  = (int32_t)ir  - g_dc_ir;
        int32_t ac_red = (int32_t)red - g_dc_red;
        g_algo_cnt++;

        /* P, PPG 逐样本输出（25 SPS，~750 bytes/sec） */
        osal_printk("P,%u,%u,%u,%d,%d\r\n",
                    (unsigned)g_total, (unsigned)ir, (unsigned)red,
                    (int)ac_ir, (int)ac_red);

        if (g_algo_cnt < ALGO_WARMUP) {
            g_prev_ac_ir = ac_ir;
            continue;
        }

        if (ac_ir > 0) {
            if (ac_ir  > g_cur_max_ir)  { g_cur_max_ir  = ac_ir;  }
            if (ac_red > g_cur_max_red) { g_cur_max_red = ac_red; }
        } else {
            if (ac_ir  < g_cur_min_ir)  { g_cur_min_ir  = ac_ir;  }
            if (ac_red < g_cur_min_red) { g_cur_min_red = ac_red; }
        }

        if (g_prev_ac_ir > 0 && ac_ir <= 0) {
            g_pk_ir  = g_cur_max_ir;
            g_pk_red = g_cur_max_red;
            g_cur_max_ir  = 0;
            g_cur_max_red = 0;
        }

        if (g_prev_ac_ir <= 0 && ac_ir > 0) {
            g_vl_ir  = g_cur_min_ir;
            g_vl_red = g_cur_min_red;
            g_cur_min_ir  = 0;
            g_cur_min_red = 0;

            if (g_last_zx > 0u) {
                uint32_t interval = g_total - g_last_zx;
                if (interval >= HR_INT_MIN && interval <= HR_INT_MAX) {
                    g_hr_intervals[g_hr_idx] = (uint16_t)interval;
                    g_hr_idx = (uint8_t)((g_hr_idx + 1u) % HR_AVG_N);
                    if (g_hr_cnt < HR_AVG_N) { g_hr_cnt++; }

                    uint32_t sum = 0;
                    for (uint8_t j = 0; j < g_hr_cnt; j++) {
                        sum += g_hr_intervals[j];
                    }
                    uint32_t avg_int = sum / g_hr_cnt;
                    if (avg_int > 0u) {
                        g_result.hr = (uint8_t)(1500u / avg_int);
                    }
                    g_result.rr_interval = (uint16_t)(interval * SAMPLE_MS);
                }
            }
            g_last_zx = g_total;

            int32_t pp_ir  = g_pk_ir  - g_vl_ir;
            int32_t pp_red = g_pk_red - g_vl_red;
            if (pp_ir > 100 && g_dc_ir > 1000 && g_dc_red > 1000) {
                uint64_t num = (uint64_t)((uint32_t)(pp_red > 0 ? pp_red : -pp_red))
                               * (uint32_t)g_dc_ir * 1000u;
                uint64_t den = (uint64_t)(uint32_t)pp_ir * (uint32_t)g_dc_red;
                if (den > 0u) {
                    uint32_t r_x1000 = (uint32_t)(num / den);
                    int32_t spo2 = 110 - (int32_t)(r_x1000 * 25u / 1000u);
                    if (spo2 > 100) { spo2 = 100; }
                    if (spo2 < 50)  { spo2 = 0;   }
                    g_result.spo2 = (uint8_t)spo2;
                }
                uint32_t pi = (uint32_t)pp_ir * 1000u / (uint32_t)g_dc_ir;
                if (pi > 250u) { pi = 250u; }
                g_result.pi_x10 = (uint8_t)pi;
            }

            uint32_t foot_offset = (uint32_t)(nsamp - 1u - i) * SAMPLE_MS;
            g_result.ppg_foot_ms  = (g_bio_ms > foot_offset)
                                    ? (g_bio_ms - foot_offset) : g_bio_ms;
            g_result.ppg_foot_new = 1u;
        }

        g_prev_ac_ir = ac_ir;
    }

    uint8_t avg_n = (g_buf_cnt > 8u) ? 8u : g_buf_cnt;
    uint32_t ir_sum = 0u, red_sum = 0u;
    uint8_t idx = (uint8_t)((g_buf_head + SAMPLE_BUF - avg_n) % SAMPLE_BUF);
    for (uint8_t i = 0u; i < avg_n; i++) {
        ir_sum  += g_ir_buf [(idx + i) % SAMPLE_BUF];
        red_sum += g_red_buf[(idx + i) % SAMPLE_BUF];
    }
    if (avg_n > 0u) {
        g_result.ir_raw  = ir_sum  / avg_n;
        g_result.red_raw = red_sum / avg_n;
    }

    g_result.finger = (g_result.ir_raw > 7000u) ? 1u : 0u;
    if (!g_result.finger && g_was_finger) {
        algo_reset();
    } else if (g_result.finger && !g_was_finger) {
        g_dc_ir  = (int32_t)g_result.ir_raw;
        g_dc_red = (int32_t)g_result.red_raw;
        g_dc_init = 1;
        g_algo_cnt = 0;
    }
    g_was_finger = g_result.finger;

    g_temp_timer++;
    if (g_temp_pending) {
        uint8_t ti = 0u, tf = 0u;
        (void)max_rreg(REG_TEMP_INTR, &ti);
        (void)max_rreg(REG_TEMP_FRAC, &tf);
        g_result.temp_int  = (int8_t)ti;
        g_result.temp_frac = tf & 0x0Fu;
        g_temp_pending = 0;
    } else if (g_temp_timer >= 50u) {
        g_temp_timer = 0;
        (void)max_wreg(REG_TEMP_CONFIG, 0x01u);
        g_temp_pending = 1;
    }

    return (int)nsamp;
}

/* ── 获取结果 ─────────────────────────────────────────────────── */
void max30102_get_result(max30102_result_t *out)
{
    if (out != NULL) {
        *out = g_result;
        g_result.ppg_foot_new = 0;
    }
}

uint32_t s_total_samples_get(void)
{
    return g_total;
}

/* ── 诊断函数 ─────────────────────────────────────────────────── */
void max30102_i2c_scan(void)
{
    osal_printk("[OPT] === MAX30102 HW-I2C1 Scan ===\r\n");
    osal_printk("[OPT]   SDA=GPIO15  SCL=GPIO16  addr=0x57\r\n");
    osal_printk("[OPT]   init_ok=%d  part_id=0x%02X  total=%u\r\n",
                g_init_ok, g_part_id, (unsigned)g_total);

    if (!g_init_ok) {
        uint8_t id = 0u;
        int ret = max_rreg(REG_PART_ID, &id);
        osal_printk("[OPT]   probe: %s  id=0x%02X\r\n",
                    (ret == 0) ? "ACK" : "NACK", id);
        if (ret != 0) {
            osal_printk("[OPT]   >> 检查 MAX30102 SDA→GPIO15, SCL→GPIO16, 2.2K上拉\r\n");
        }
    } else {
        uint8_t wr = 0u, rd = 0u, ovf = 0u;
        (void)max_rreg(REG_FIFO_WR_PTR, &wr);
        (void)max_rreg(REG_FIFO_RD_PTR, &rd);
        (void)max_rreg(REG_OVF_COUNTER, &ovf);
        osal_printk("[OPT]   FIFO wr=%u rd=%u ovf=%u  buf=%u\r\n",
                    wr, rd, ovf, g_buf_cnt);
        osal_printk("[OPT]   IR=%u  RED=%u  finger=%u\r\n",
                    (unsigned)g_result.ir_raw,
                    (unsigned)g_result.red_raw,
                    g_result.finger);
    }
}

void max30102_dump_regs(void)
{
    osal_printk("[OPT] === MAX30102 Register Dump ===\r\n");
    static const uint8_t regs[] = {
        0x00u, 0x01u, 0x02u, 0x03u,
        0x04u, 0x05u, 0x06u, 0x08u,
        0x09u, 0x0Au, 0x0Cu, 0x0Du,
        0xFFu
    };
    static const char *names[] = {
        "INT_ST1", "INT_ST2", "INT_EN1", "INT_EN2",
        "FIFO_WR", "OVF_CNT", "FIFO_RD", "FIFO_CFG",
        "MODE   ", "SPO2   ", "LED1_PA", "LED2_PA",
        "PART_ID"
    };
    for (uint8_t i = 0u; i < (uint8_t)(sizeof(regs) / sizeof(regs[0])); i++) {
        uint8_t v = 0u;
        int ret = max_rreg(regs[i], &v);
        osal_printk("[OPT]   [0x%02X] %s = 0x%02X  %s\r\n",
                    regs[i], names[i], v, (ret == 0) ? "OK" : "NACK");
    }
}

void max30102_force_read(void)
{
    osal_printk("[OPT] === FIFO Force Read ===\r\n");
    uint8_t wr = 0u, rd = 0u;
    (void)max_rreg(REG_FIFO_WR_PTR, &wr);
    (void)max_rreg(REG_FIFO_RD_PTR, &rd);
    uint8_t pending = (uint8_t)((wr - rd + 32u) % 32u);
    osal_printk("[OPT]   FIFO wr=%u rd=%u  pending=%u\r\n", wr, rd, pending);
    osal_printk("[OPT]   buf_cnt=%u  IR=%u  RED=%u  total=%u\r\n",
                g_buf_cnt,
                (unsigned)g_result.ir_raw,
                (unsigned)g_result.red_raw,
                (unsigned)g_total);
}

void max30102_set_baud(uint32_t baud)
{
    (void)baud;
}

void max30102_send(const uint8_t *data, uint16_t len)
{
    (void)data; (void)len;
}
