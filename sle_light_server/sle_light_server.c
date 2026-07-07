/**
 * sle_light_server.c — 大板 SLE 客户端（生理数据接收 + 串口输出 + OLED 显示）
 *
 * 硬件：大板 FS-Hi3863-V2 COM7
 *   SSD1306 0.96" OLED : I2C bus1，IO15=SDA，IO16=SCL（FPC 已焊接）
 *   SLE 接收            : 扫描广播名 "sle_test"，找到后连接，接收 bio_pkt_t（16 字节）
 *
 * 功能：
 *   1. 扫描 SLE 广播，找到名为 "sle_test" 的设备后连接
 *   2. 每收到一包 bio_pkt_t，通过 COM7 串口（115200）输出可读格式
 *   3. OLED 显示 HR / SpO2 / ECG 心率 / ECG 信号质量
 *
 * 注意：自绘板广播名为 "sle_test"（由 sle_server_adv.c 中 g_sle_local_name 决定）
 *       SLE_TARGET_NAME 必须与 sle_server_adv.c 中的设备名严格一致
 */

#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "watchdog.h"
#include "securec.h"
#include "sle_ssap_client.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "i2c.h"
#include "pinctrl.h"
#include <stdio.h>
#include <string.h>

/* ── 任务配置 ─────────────────────────────────────────────────── */
#define TASK_PRIO    17
#define TASK_STACK   0x3000

/* ── I2C / OLED 配置 ──────────────────────────────────────────── */
#define I2C_BUS         1
#define I2C_SCL_PIN     16
#define I2C_SDA_PIN     15
#define I2C_PIN_MODE    2
#define I2C_SPEED       400000

/* ── SLE 扫描目标设备名（必须与 sle_server_adv.c 中 g_sle_local_name 一致）── */
/* sle_server_adv.c 第 45 行: g_sle_local_name = "sle_test" → 此处必须匹配 */
#define SLE_TARGET_NAME     "sle_test"
#define SLE_TARGET_NAME_LEN 8u

#define SLE_MTU_SIZE_DEFAULT  512u
#define UUID_LEN_2            2u
#define SLE_UUID_SERVER_NTF   0xBCDEu


/* ── 数据包定义 V2（必须与自绘板完全一致，20 字节）──────────── */
#define BIO_PKT_VER  2u

typedef struct {
    uint8_t  hr;
    uint8_t  spo2;
    uint32_t ir;
    int8_t   temp_i;
    uint8_t  temp_f;
    uint8_t  pi_x10;
    uint8_t  sbp;       /* 收缩压 mmHg (0=无效) */
    uint8_t  dbp;       /* 舒张压 mmHg (0=无效) */
    uint8_t  flags;     /* bit0=max_ok bit1=bmd_ok bit2=finger bit3=ptt_valid */
    uint16_t ptt_ms;    /* PTT ms */
    uint8_t  ecg_hr;
    uint8_t  ecg_sig;
    int16_t  ecg_raw;
    uint16_t rr_ms;     /* PPG R-R 间期 ms */
} __attribute__((packed)) bio_pkt_t;   /* 20 bytes */

#define CMD_HEARTBEAT  0u
typedef struct {
    uint8_t  cmd;
    uint8_t  seq;
    uint16_t uptime_s;
} __attribute__((packed)) ctrl_pkt_t;

/* ── UUID 辅助 ────────────────────────────────────────────────── */
static uint8_t g_sle_base[SLE_UUID_LEN] = {
    0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA,
    0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void sle_uuid_setu2(uint16_t u2, sle_uuid_t *out)
{
    (void)memcpy_s(out->uuid, SLE_UUID_LEN, g_sle_base, SLE_UUID_LEN);
    out->len = UUID_LEN_2;
    out->uuid[14] = (uint8_t)(u2 & 0xFFu);
    out->uuid[15] = (uint8_t)(u2 >> 8u);
}

/* ── 共享数据 ─────────────────────────────────────────────────── */
static volatile uint8_t g_new_data  = 0;
static bio_pkt_t        g_latest    = {0};

/* ── SLE 客户端状态 ───────────────────────────────────────────── */
static uint16_t         g_conn_id   = 0;
static volatile uint8_t g_connected = 0;
static volatile uint8_t g_scanning  = 0;
static uint8_t          g_client_id = 0;
static uint16_t         g_prop_hdl  = 0;
static uint8_t          g_ctrl_seq  = 0;
static sle_addr_t       g_server_addr = {0};  /* 扫描到的服务端地址 */

/* ── 二进制内存中查找子串（AD 数据含 \x00，不能用 strstr）────── */
static int memfind(const uint8_t *haystack, uint16_t hlen,
                   const char *needle, uint8_t nlen)
{
    if (hlen < nlen) { return 0; }
    for (uint16_t i = 0u; i <= (uint16_t)(hlen - nlen); i++) {
        if (memcmp(&haystack[i], needle, nlen) == 0) { return 1; }
    }
    return 0;
}

/* ── SLE 扫描结果回调 ─────────────────────────────────────────── */
static void sle_seek_result_cbk(sle_seek_result_info_t *r)
{
    if (r == NULL || g_connected || !g_scanning) { return; }
    if (r->data == NULL || r->data_length == 0u) { return; }

    /* 在二进制 AD 数据中逐字节查找设备名（避免 strstr 遇 \x00 截断）*/
    if (memfind(r->data, r->data_length,
                SLE_TARGET_NAME, SLE_TARGET_NAME_LEN)) {
        osal_printk("[CLI] Found %s! addr=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    SLE_TARGET_NAME,
                    r->addr.addr[5], r->addr.addr[4], r->addr.addr[3],
                    r->addr.addr[2], r->addr.addr[1], r->addr.addr[0]);
        g_scanning = 0;
        (void)memcpy_s(&g_server_addr, sizeof(sle_addr_t),
                       &r->addr, sizeof(sle_addr_t));
        sle_stop_seek();
        /* 停止扫描后才在 seek_disable_cbk 中发起连接 */
    }
}

static void sle_seek_disable_cbk(errcode_t status)
{
    (void)status;
    if (!g_scanning && !g_connected) {
        /* 先清除旧配对信息，避免重连失败 */
        sle_remove_paired_remote_device(&g_server_addr);
        sle_connect_remote_device(&g_server_addr);
    }
}

static void sle_seek_enable_cbk(errcode_t status)
{
    osal_printk("[CLI] Seek started st=%x\r\n", status);
}

/* ── 发送控制包 ───────────────────────────────────────────────── */
static void send_ctrl(uint8_t cmd, uint16_t uptime)
{
    if (!g_connected || g_prop_hdl == 0u) { return; }
    ctrl_pkt_t pkt = {.cmd = cmd, .seq = g_ctrl_seq++, .uptime_s = uptime};
    ssapc_write_param_t wr = {0};
    wr.handle   = g_prop_hdl;
    wr.type     = 0;
    wr.data_len = (uint16_t)sizeof(ctrl_pkt_t);
    wr.data     = (uint8_t *)&pkt;
    ssapc_write_cmd(g_client_id, g_conn_id, &wr);
}

/* ── ECG 流式数据标记（与 sensor_hub.c 一致）──────────────────── */
#define ECG_STREAM_TAG  0xECu

/* ── SSAPC 通知回调 ───────────────────────────────────────────── */
static void sle_notification_cbk(uint8_t client_id, uint16_t conn_id,
                                   ssapc_handle_value_t *data, errcode_t status)
{
    (void)client_id; (void)conn_id; (void)status;
    if (data == NULL || data->data == NULL || data->data_len < 2u) { return; }

    /* ECG 批量包：[0xEC] [count] [int16 samples...] */
    if (data->data[0] == ECG_STREAM_TAG) {
        uint8_t cnt = data->data[1];
        if (cnt > 0u && data->data_len >= (uint16_t)(2u + cnt * 2u)) {
            const int16_t *samples = (const int16_t *)&data->data[2];
            for (uint8_t i = 0u; i < cnt; i++) {
                osal_printk("E,%d\r\n", (int)samples[i]);
            }
        }
        return;
    }

    /* bio_pkt_t 数据包 */
    if (data->data_len >= (uint16_t)sizeof(bio_pkt_t)) {
        (void)memcpy_s(&g_latest, sizeof(bio_pkt_t),
                       data->data, sizeof(bio_pkt_t));
        g_new_data = 1;
    }
}

/* ── SSAPC 其他回调 ───────────────────────────────────────────── */
static void sle_find_prop_cbk(uint8_t cid, uint16_t conn,
                               ssapc_find_property_result_t *r, errcode_t st)
{
    (void)cid; (void)conn; (void)st;
    if (r != NULL) {
        g_prop_hdl = r->handle;
        osal_printk("[CLI] prop_hdl=0x%x\r\n", g_prop_hdl);
    }
}

static void sle_find_svc_cbk(uint8_t cid, uint16_t conn,
                               ssapc_find_service_result_t *r, errcode_t st)
{ (void)cid; (void)conn; (void)r; (void)st; }

static void sle_find_svc_cmp_cbk(uint8_t cid, uint16_t conn,
                                   ssapc_find_structure_result_t *r, errcode_t st)
{ (void)cid; (void)conn; (void)r;
  osal_printk("[CLI] svc_cmp st=%x\r\n", st); }

static void sle_write_cbk(uint8_t cid, uint16_t conn,
                           ssapc_write_result_t *r, errcode_t st)
{ (void)cid; (void)conn; (void)r; (void)st; }

/* ── MTU 交换完成回调 → 再做属性发现 ─────────────────────────── */
static void sle_exchange_info_cbk(uint8_t client_id, uint16_t conn_id,
                                   ssap_exchange_info_t *param, errcode_t status)
{
    (void)client_id;
    osal_printk("[CLI] MTU=%u st=%x\r\n", param ? param->mtu_size : 0u, status);
    /* MTU 交换完成后发起属性发现（与 Farsight 保持一致）*/
    ssapc_find_structure_param_t fp = {0};
    fp.type      = SSAP_FIND_TYPE_PROPERTY;
    fp.start_hdl = 0x01;
    fp.end_hdl   = 0xFFFF;
    sle_uuid_setu2(SLE_UUID_SERVER_NTF, &fp.uuid);
    ssapc_find_structure(g_client_id, conn_id, &fp);
}

static errcode_t sle_ssapc_register_cbks(void)
{
    ssapc_callbacks_t cbk = {0};
    cbk.exchange_info_cb        = sle_exchange_info_cbk;
    cbk.find_structure_cb       = sle_find_svc_cbk;
    cbk.find_structure_cmp_cb   = sle_find_svc_cmp_cbk;
    cbk.ssapc_find_property_cbk = sle_find_prop_cbk;
    cbk.write_cfm_cb            = sle_write_cbk;
    cbk.notification_cb         = sle_notification_cbk;
    cbk.indication_cb           = sle_notification_cbk;
    return ssapc_register_callbacks(&cbk);
}

/* ── SLE 连接回调 ─────────────────────────────────────────────── */
static void sle_connect_state_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                   sle_acb_state_t state,
                                   sle_pair_state_t pair_state,
                                   sle_disc_reason_t disc_reason)
{
    (void)addr; (void)disc_reason;
    if (state == SLE_ACB_STATE_CONNECTED) {
        g_conn_id = conn_id; g_connected = 1;
        osal_printk("[CLI] Connected conn_id=%u pair_state=%u\r\n",
                    conn_id, pair_state);
        /* 按 Farsight 标准流程：连接后先配对，配对完再做 MTU + 属性发现 */
        if (pair_state == SLE_PAIR_NONE) {
            sle_pair_remote_device(&g_server_addr);
        }
    } else if (state == SLE_ACB_STATE_DISCONNECTED) {
        g_connected = 0; g_conn_id = 0; g_prop_hdl = 0;
        osal_printk("[CLI] Disconnected → re-scan\r\n");
        osal_msleep(1000);
        g_scanning = 1;
        sle_start_seek();
    }
}

static void sle_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                   errcode_t status)
{
    (void)addr;
    osal_printk("[CLI] Pair conn=%u st=%x\r\n", conn_id, status);
    if (status == ERRCODE_SUCC) {
        /* 配对成功后请求 MTU 交换，MTU 回调中再做属性发现 */
        ssap_exchange_info_t info = {0};
        info.mtu_size = SLE_MTU_SIZE_DEFAULT;
        info.version  = 1;
        ssapc_exchange_info_req(g_client_id, conn_id, &info);
    }
}

/* ── 启动扫描 ─────────────────────────────────────────────────── */
static void start_scan(void)
{
    sle_seek_param_t sp = {0};
    sp.own_addr_type       = 0;
    sp.filter_duplicates   = 0;
    sp.seek_filter_policy  = 0;
    sp.seek_phys           = 1;
    sp.seek_type[0]        = 1;   /* 主动扫描，可收 scan response（含设备名）*/
    sp.seek_interval[0]    = 0x80;
    sp.seek_window[0]      = 0x40;
    sle_set_seek_param(&sp);

    sle_announce_seek_callbacks_t seek_cbk = {0};
    seek_cbk.seek_enable_cb  = sle_seek_enable_cbk;
    seek_cbk.seek_disable_cb = sle_seek_disable_cbk;
    seek_cbk.seek_result_cb  = sle_seek_result_cbk;
    sle_announce_seek_register_callbacks(&seek_cbk);

    g_scanning = 1;
    sle_start_seek();
    osal_printk("[CLI] Scanning for '%s'...\r\n", SLE_TARGET_NAME);
}

/* ── SLE Client 初始化 ────────────────────────────────────────── */
static errcode_t sle_client_init(void)
{
    if (enable_sle() != ERRCODE_SUCC) { return ERRCODE_FAIL; }

    sle_connection_callbacks_t conn_cbk = {0};
    conn_cbk.connect_state_changed_cb = sle_connect_state_cbk;
    conn_cbk.pair_complete_cb         = sle_pair_complete_cbk;
    sle_connection_register_callbacks(&conn_cbk);
    sle_ssapc_register_cbks();

    char app_uuid_val[UUID_LEN_2] = {0};
    sle_uuid_t app_uuid = {.len = UUID_LEN_2};
    (void)memcpy_s(app_uuid.uuid, UUID_LEN_2, app_uuid_val, UUID_LEN_2);
    ssapc_register_client(&app_uuid, &g_client_id);

    osal_printk("[CLI] SLE client init OK, client_id=%u\r\n", g_client_id);
    return ERRCODE_SUCC;
}

/* ── OLED 辅助 ────────────────────────────────────────────────── */
static void draw_hline(uint8_t y)
{
    for (uint8_t x = 0u; x < 128u; x++) { ssd1306_DrawPixel(x, y, White); }
}

/* ── 串口格式化输出 ────────────────────────────────────────────── */
static uint32_t g_bio_ms_relay = 0;   /* 简易毫秒计时（每包+500ms） */

static void print_bio_pkt(const bio_pkt_t *p, uint32_t pkt_idx)
{
    uint8_t fin = (p->flags & 0x04u) ? 1u : 0u;
    g_bio_ms_relay += 500u;

    /* D-line CSV：与自绘板格式完全一致，raw_capture.py 可直接解析 */
    osal_printk("D,%u,%u,0,%d,%u,%u,%u,%u,%u\r\n",
                (unsigned)g_bio_ms_relay,
                (unsigned)p->ir, (int)p->ecg_raw,
                p->hr, p->spo2,
                p->ecg_hr, (unsigned)p->ecg_sig, fin);

    /* BMD101 诊断：sig_upd(高8位) / hr_upd(低8位) 来自 rr_ms 字段 */
    uint8_t sig_upd = (uint8_t)(p->rr_ms >> 8u);
    uint8_t hr_upd  = (uint8_t)(p->rr_ms & 0xFFu);

    /* 每 10 包输出一次可读摘要 */
    if ((pkt_idx % 10u) == 0u) {
        osal_printk("[BIO#%lu] HR=%u SpO2=%u eHR=%u eSIG=%u "
                    "sig_upd=%u hr_upd=%u ecg_raw=%d\r\n",
                    (unsigned long)pkt_idx,
                    p->hr, p->spo2, p->ecg_hr, p->ecg_sig,
                    sig_upd, hr_upd, (int)p->ecg_raw);
    }
}

/* ── OLED 刷新 ────────────────────────────────────────────────── */
static void update_oled(const bio_pkt_t *p, uint32_t pkt_idx)
{
    char buf[22];
    uint8_t fin = (p->flags & 0x04u) ? 1u : 0u;

    ssd1306_Fill(Black);

    /* Row 0: 标题 */
    (void)snprintf(buf, sizeof(buf), "BIO #%lu", (unsigned long)(pkt_idx % 10000u));
    ssd1306_SetCursor(0, 0);
    ssd1306_DrawString(buf, Font_7x10, White);
    draw_hline(11);

    /* Row 1: HR + SpO2 */
    if (fin && p->hr > 0u) {
        (void)snprintf(buf, sizeof(buf), "HR:%3u SpO2:%2u%%", p->hr, p->spo2);
    } else if (!fin) {
        (void)snprintf(buf, sizeof(buf), "HR:--- [NoFinger]");
    } else {
        (void)snprintf(buf, sizeof(buf), "HR:cal SpO2:%2u%%", p->spo2);
    }
    ssd1306_SetCursor(0, 16);
    ssd1306_DrawString(buf, Font_7x10, White);

    /* Row 2: 血压 */
    if (p->sbp > 0u) {
        (void)snprintf(buf, sizeof(buf), "BP:%u/%u PTT:%u",
                       p->sbp, p->dbp, p->ptt_ms);
    } else {
        (void)snprintf(buf, sizeof(buf), "BP:---/---");
    }
    ssd1306_SetCursor(0, 27);
    ssd1306_DrawString(buf, Font_7x10, White);

    /* Row 3: ECG */
    const char *sig_str;
    if      (p->ecg_sig == 0u)   { sig_str = "EXC"; }
    else if (p->ecg_sig <= 25u)  { sig_str = "GD "; }
    else if (p->ecg_sig <= 100u) { sig_str = "FAR"; }
    else if (p->ecg_sig < 200u)  { sig_str = "POR"; }
    else                          { sig_str = "NC "; }
    (void)snprintf(buf, sizeof(buf), "ECG:%3u SIG:%s", p->ecg_hr, sig_str);
    ssd1306_SetCursor(0, 40);
    ssd1306_DrawString(buf, Font_7x10, White);

    /* Row 4: 温度 + PI */
    uint32_t tf = (uint32_t)(p->temp_f & 0x0Fu) * 625u / 1000u;
    (void)snprintf(buf, sizeof(buf), "T:%d.%u PI:%u.%u%%",
                   p->temp_i, (unsigned)tf,
                   p->pi_x10 / 10u, p->pi_x10 % 10u);
    ssd1306_SetCursor(0, 53);
    ssd1306_DrawString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}

/* ── 主任务 ───────────────────────────────────────────────────── */
static int sle_bio_client_task(const char *arg)
{
    (void)arg;
    osal_msleep(500);

    osal_printk("\r\n[CLI] ===== 大板 SLE 生理数据接收端 (COM7) =====\r\n");

    /* 1. SLE 初始化（必须在 I2C 之前）*/
    sle_client_init();
    osal_msleep(200);

    /* 2. I2C1 + SSD1306 初始化 */
    uapi_pin_set_mode(I2C_SCL_PIN, I2C_PIN_MODE);
    uapi_pin_set_mode(I2C_SDA_PIN, I2C_PIN_MODE);
    uapi_pin_set_pull(I2C_SCL_PIN, PIN_PULL_TYPE_UP);
    uapi_pin_set_pull(I2C_SDA_PIN, PIN_PULL_TYPE_UP);
    uapi_i2c_master_init(I2C_BUS, I2C_SPEED, 0);

    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_DrawString("BIO Scanning..", Font_7x10, White);
    ssd1306_SetCursor(0, 16);
    ssd1306_DrawString("Find:" SLE_TARGET_NAME, Font_7x10, White);
    ssd1306_UpdateScreen();

    /* 3. 开始扫描 */
    start_scan();

    uint32_t uptime   = 0;
    uint32_t hb_timer = 0;
    uint32_t pkt_idx  = 0;

    while (1) {
        uptime++;
        hb_timer++;

        /* 心跳（每 2s）*/
        if (hb_timer >= 20u && g_connected) {
            send_ctrl(CMD_HEARTBEAT, (uint16_t)(uptime / 10u));
            hb_timer = 0;
        }

        /* 未连接时 OLED 提示 */
        if (!g_connected) {
            if ((uptime % 20u) == 0u) {
                ssd1306_Fill(Black);
                ssd1306_SetCursor(0, 0);
                ssd1306_DrawString("BIO Scanning..", Font_7x10, White);
                ssd1306_SetCursor(0, 16);
                ssd1306_DrawString("Find:" SLE_TARGET_NAME, Font_7x10, White);
                ssd1306_UpdateScreen();
                osal_printk("[CLI] Scanning for '%s'...\r\n", SLE_TARGET_NAME);
            }
            osal_msleep(100);
            continue;
        }

        /* 有新数据 */
        if (g_new_data) {
            g_new_data = 0;
            bio_pkt_t pkt;
            (void)memcpy_s(&pkt, sizeof(bio_pkt_t), &g_latest, sizeof(bio_pkt_t));
            pkt_idx++;
            print_bio_pkt(&pkt, pkt_idx);
            update_oled(&pkt, pkt_idx);
        } else {
            /* 已连接但迟迟没数据 → 每 5s 打一次等待日志，便于诊断 */
            static uint32_t s_wait_cnt = 0;
            s_wait_cnt++;
            if (s_wait_cnt >= 50u) {
                s_wait_cnt = 0;
                osal_printk("[CLI] Waiting notify... conn=%u prop=0x%x\r\n",
                            g_conn_id, g_prop_hdl);
            }
        }

        osal_msleep(100);
    }
    return 0;
}

/* ── 入口 ─────────────────────────────────────────────────────── */
static void sle_bio_client_entry(void)
{
    uapi_watchdog_disable();
    osal_kthread_lock();
    osal_task *task = osal_kthread_create(
        (osal_kthread_handler)sle_bio_client_task, 0,
        "sle_bio_cli", TASK_STACK);
    if (task != NULL) {
        osal_kthread_set_priority(task, TASK_PRIO);
        osal_kfree(task);
    }
    osal_kthread_unlock();
}
app_run(sle_bio_client_entry);
