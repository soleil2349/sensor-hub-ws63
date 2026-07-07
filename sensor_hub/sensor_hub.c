/**
 * sensor_hub.c — 自绘板生理数据多模式服务端
 *
 * 硬件配置（V3 PCB，引脚已交换，无需飞线）：
 *   MAX30102  硬件I2C1  GPIO16(SCL/IRD), GPIO15(SDA/RD)  PCB 直连模块 IRD/RD
 *   BMD101    硬件UART2  GPIO7(RXD/Pin12), GPIO8(TXD/Pin13)  PIN_MODE_2
 *
 * BMD101 引脚说明（V3 PCB 已交换 Pin12/Pin13）：
 *   BMD101 TX → GPIO7 (Pin12) = UART2_RXD  硬件接收
 *   BMD101 RX → GPIO8 (Pin13) = UART2_TXD  硬件发送
 *
 * 多模式并行通信架构：
 *   ① SLE Server  广播名 "sle_test"  地址 01:02:03:04:05:07
 *      → 大板（FS-Hi3863-V2）SLE Client 接收，OLED 显示
 *   ② BLE GATT Server  广播名 "BIO_HUB"  地址 11:22:33:44:55:AB
 *      → PC / 手机 / 平板 标准 BLE 接收（nRF Connect 或 bio_ble_recv.py）
 *      → 用于独立调试，无需大板即可验证传感器数据
 *
 * 初始化顺序（关键）：
 *   SLE init → 200ms → BLE init → 300ms → HW-I2C1(MAX30102) → HW-UART2(BMD101)
 */

#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "watchdog.h"
#include "securec.h"
#include "sle_ssap_server.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_server_adv.h"
#include "bio_pkt.h"
#include "max30102_drv.h"
#include "bmd101_drv.h"
#include "ble_bio_server.h"
#include "uart.h"
#include "pinctrl.h"
#include <stdio.h>
#include <string.h>

/*
 * ── 调试模式开关 ──────────────────────────────────────────────────
 * SKIP_BMD101 = 1:
 *   跳过 bmd101_init()，UART0 保持 115200 baud，CH340 串口可见所有日志。
 *   同时开启 UART0 RX 命令接口（跳线必须在 CH340 侧）。
 *   命令字符：i=I2C扫描  r=寄存器转储  m=强制读FIFO  c=切换连续打印  ?=帮助
 *
 * SKIP_BMD101 = 0:
 *   正常运行：bmd101_init() 启动硬件 UART2（不修改 UART0），
 *   CH340 保持 115200 即可查看所有日志。
 */
#define SKIP_BMD101  0

/* ── 全局毫秒计时器（bio_pkt.h 声明，各驱动共享）───────────────── */
volatile uint32_t g_bio_ms = 0;

/* ── 任务配置 ─────────────────────────────────────────────────── */
#define TASK_PRIO       17
#define TASK_STACK      0x3000

/* ── SLE UUID 定义（与大板客户端完全一致）────────────────────── */
#define UUID_LEN_2              2u
#define SLE_UUID_SERVER_SERVICE 0xABCDu
#define SLE_UUID_SERVER_NTF     0xBCDEu
#define SLE_ADV_HANDLE          1u
#define SLE_MTU_DEFAULT         512u

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

/* ── SLE 服务端状态 ───────────────────────────────────────────── */
static uint8_t  g_server_id   = 0;
static uint16_t g_service_hdl = 0;
static uint16_t g_prop_hdl    = 0;
static uint16_t g_conn_id     = 0;
static volatile uint8_t g_connected  = 0;
static volatile uint8_t g_srv_paused = 0;
static sle_addr_t g_client_addr = {0};  /* 已连接客户端地址（断连时用）*/

/* 客户端存活检测：每次收到写包就刷新计数；超时则主动断连重广播 */
static volatile uint32_t g_last_rx_tick = 0;   /* 最后收到客户端数据的时间（100ms 单位）*/
#define CLIENT_TIMEOUT_TICKS  100u              /* 10s = 100 × 100ms */

/* ── 注册 SSAPS 回调（最小集合，只处理写入控制包）─────────────── */
static void ssaps_write_cbk(uint8_t server_id, uint16_t conn_id,
                             ssaps_req_write_cb_t *w, errcode_t status)
{
    (void)server_id; (void)conn_id; (void)status;
    /* 收到任何写包 → 刷新存活计数 */
    g_last_rx_tick = 0;
    if (w == NULL || w->length < (uint16_t)sizeof(ctrl_pkt_t)) { return; }
    const ctrl_pkt_t *pkt = (const ctrl_pkt_t *)w->value;
    if (pkt->cmd == CMD_TOGGLE) {
        g_srv_paused = !g_srv_paused;
        osal_printk("[SRV] Toggle: paused=%u\r\n", g_srv_paused);
    }
}

static void ssaps_mtu_cbk(uint8_t server_id, uint16_t conn_id,
                           ssap_exchange_info_t *mtu, errcode_t status)
{
    (void)server_id; (void)conn_id; (void)status;
    osal_printk("[SRV] MTU=%u\r\n", mtu ? mtu->mtu_size : 0u);
}

static errcode_t sle_ssaps_register_cbks(void)
{
    ssaps_callbacks_t cbk = {0};
    cbk.mtu_changed_cb    = ssaps_mtu_cbk;
    cbk.write_request_cb  = ssaps_write_cbk;
    return ssaps_register_callbacks(&cbk);
}

/* ── SLE 连接回调 ─────────────────────────────────────────────── */
static void sle_connect_state_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                   sle_acb_state_t state,
                                   sle_pair_state_t pair_state,
                                   sle_disc_reason_t disc_reason)
{
    (void)pair_state; (void)disc_reason;
    if (state == SLE_ACB_STATE_CONNECTED) {
        g_conn_id = conn_id;
        g_connected = 1;
        g_last_rx_tick = 0;
        if (addr != NULL) {
            (void)memcpy_s(&g_client_addr, sizeof(sle_addr_t), addr, sizeof(sle_addr_t));
        }
        osal_printk("[SRV] Connected conn_id=%u\r\n", conn_id);
    } else if (state == SLE_ACB_STATE_DISCONNECTED) {
        g_connected = 0;
        osal_printk("[SRV] Disconnected, restart adv\r\n");
        sle_start_announce(SLE_ADV_HANDLE);
    }
}

static void sle_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                   errcode_t status)
{
    (void)addr;
    osal_printk("[SRV] Pair conn=%u st=%x\r\n", conn_id, status);
}

/* ── SLE Server 初始化（同步方式，复用 Farsight 广播模块）──────── */
static errcode_t sle_server_setup(void)
{
    /* 使能 SLE 射频栈 */
    errcode_t ret = enable_sle();
    if (ret != ERRCODE_SUCC) {
        osal_printk("[SRV] enable_sle fail: %x\r\n", ret);
        return ERRCODE_FAIL;
    }

    /* 注册广播回调（sle_server_adv.c 中定义）*/
    sle_announce_register_cbks();

    /* 注册连接回调 */
    sle_connection_callbacks_t conn_cbk = {0};
    conn_cbk.connect_state_changed_cb = sle_connect_state_cbk;
    conn_cbk.pair_complete_cb         = sle_pair_complete_cbk;
    sle_connection_register_callbacks(&conn_cbk);

    /* 注册 SSAPS 回调 */
    sle_ssaps_register_cbks();

    /* 注册服务器 */
    char app_uuid_val[UUID_LEN_2] = {0};
    sle_uuid_t app_uuid = {.len = UUID_LEN_2};
    (void)memcpy_s(app_uuid.uuid, UUID_LEN_2, app_uuid_val, UUID_LEN_2);
    ssaps_register_server(&app_uuid, &g_server_id);

    /* 添加 GATT 服务（同步）*/
    sle_uuid_t svc_uuid = {0};
    sle_uuid_setu2(SLE_UUID_SERVER_SERVICE, &svc_uuid);
    ret = ssaps_add_service_sync(g_server_id, &svc_uuid, 1, &g_service_hdl);
    if (ret != ERRCODE_SUCC) {
        osal_printk("[SRV] add_service fail: %x\r\n", ret);
        return ERRCODE_FAIL;
    }

    /* 添加属性（Property / Characteristic）*/
    static uint8_t prop_val[sizeof(bio_pkt_t)];
    memset(prop_val, 0, sizeof(prop_val));

    ssaps_property_info_t prop = {0};
    sle_uuid_setu2(SLE_UUID_SERVER_NTF, &prop.uuid);
    prop.permissions         = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    prop.operate_indication  = SSAP_OPERATE_INDICATION_BIT_READ |
                               SSAP_OPERATE_INDICATION_BIT_WRITE_NO_RSP |
                               SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    prop.value     = prop_val;
    prop.value_len = (uint16_t)sizeof(bio_pkt_t);

    ret = ssaps_add_property_sync(g_server_id, g_service_hdl, &prop, &g_prop_hdl);
    if (ret != ERRCODE_SUCC) {
        osal_printk("[SRV] add_property fail: %x\r\n", ret);
        return ERRCODE_FAIL;
    }

    /* 添加 CCC 描述符（CCCD）—— SLE SSAP 通知必须有此描述符，初始值 0x0001 = 通知默认开启 */
    static uint8_t cccd_val[2] = {0x01, 0x00};
    ssaps_desc_info_t desc = {0};
    desc.permissions        = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    desc.type               = SSAP_DESCRIPTOR_CLIENT_CONFIGURATION;
    desc.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ |
                              SSAP_OPERATE_INDICATION_BIT_WRITE;
    desc.value     = cccd_val;
    desc.value_len = (uint16_t)sizeof(cccd_val);
    ret = ssaps_add_descriptor_sync(g_server_id, g_service_hdl, g_prop_hdl, &desc);
    if (ret != ERRCODE_SUCC) {
        osal_printk("[SRV] add_descriptor fail: %x\r\n", ret);
        /* 非致命错误，继续运行 */
    } else {
        osal_printk("[SRV] CCCD added OK\r\n");
    }

    /* 启动服务 */
    ret = ssaps_start_service(g_server_id, g_service_hdl);
    if (ret != ERRCODE_SUCC) {
        osal_printk("[SRV] start_service fail: %x\r\n", ret);
        return ERRCODE_FAIL;
    }

    /* 设置 MTU */
    ssap_exchange_info_t ex_info = {0};
    ex_info.mtu_size = SLE_MTU_DEFAULT;
    ex_info.version  = 1;
    ssaps_set_info(g_server_id, &ex_info);

    /* 开始广播（sle_server_adv.c 中实现）*/
    ret = sle_server_adv_init();
    if (ret != ERRCODE_SUCC) {
        osal_printk("[SRV] adv_init fail: %x\r\n", ret);
        return ERRCODE_FAIL;
    }

    osal_printk("[SRV] SLE server OK, server_id=%u svc=%x prop=%x\r\n",
                g_server_id, g_service_hdl, g_prop_hdl);
    return ERRCODE_SUCC;
}

/* ── PTT → 血压映射（分段线性近似，文献默认系数）───────────────── */
static uint8_t ptt_to_sbp(uint16_t ptt)
{
    int32_t sbp;
    if      (ptt < 150u) { sbp = 180; }
    else if (ptt <= 250u) { sbp = 180 - (int32_t)(ptt - 150u) * 60 / 100; }
    else if (ptt <= 400u) { sbp = 120 - (int32_t)(ptt - 250u) * 40 / 150; }
    else                  { sbp = 80;  }
    if (sbp > 220) { sbp = 220; }
    if (sbp < 60)  { sbp = 60;  }
    return (uint8_t)sbp;
}

static uint32_t g_prev_rpeak_ms = 0;
static uint32_t g_prev_foot_ms  = 0;
#define PTT_BUF_N  5u
static uint16_t g_ptt_buf[PTT_BUF_N];
static uint8_t  g_ptt_cnt = 0;
static uint8_t  g_ptt_idx = 0;

static uint16_t ptt_median(void)
{
    if (g_ptt_cnt == 0u) { return 0; }
    uint16_t tmp[PTT_BUF_N];
    uint8_t n = g_ptt_cnt;
    for (uint8_t i = 0; i < n; i++) { tmp[i] = g_ptt_buf[i]; }
    for (uint8_t i = 0; i < n - 1u; i++) {
        for (uint8_t j = (uint8_t)(i + 1u); j < n; j++) {
            if (tmp[j] < tmp[i]) { uint16_t t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
        }
    }
    return tmp[n / 2u];
}

/* ── 发送 bio_pkt_t Notify ────────────────────────────────────── */
static uint32_t g_ntf_ok  = 0;
static uint32_t g_ntf_err = 0;

static void send_bio_notify(const bio_pkt_t *pkt)
{
    if (!g_connected) { return; }
    ssaps_ntf_ind_t param = {0};
    param.handle    = g_prop_hdl;
    param.type      = SSAP_PROPERTY_TYPE_VALUE;
    param.value     = (uint8_t *)pkt;
    param.value_len = (uint16_t)sizeof(bio_pkt_t);
    errcode_t ret = ssaps_notify_indicate(g_server_id, g_conn_id, &param);
    if (ret == ERRCODE_SUCC) {
        g_ntf_ok++;
    } else {
        g_ntf_err++;
        osal_printk("[SRV] ntf FAIL ret=0x%x ok=%u err=%u srv=%u conn=%u hdl=0x%x\r\n",
                    ret, g_ntf_ok, g_ntf_err,
                    g_server_id, g_conn_id, g_prop_hdl);
    }
}

/* ── UART0 命令接口（SKIP_BMD101=1 时有效，跳线在 CH340 侧）──── */
#if SKIP_BMD101

#define CMD_UART_BUS    UART_BUS_0
#define CMD_HW_BUF_SIZE 64u

static uint8_t  g_cmd_hw_buf[CMD_HW_BUF_SIZE];
static uart_buffer_config_t g_cmd_buf_cfg = {
    .rx_buffer      = g_cmd_hw_buf,
    .rx_buffer_size = CMD_HW_BUF_SIZE
};

/* 单字节命令队列（最多 8 个待处理命令）*/
#define CMD_QUEUE_SIZE 8u
static volatile uint8_t g_cmd_q[CMD_QUEUE_SIZE];
static volatile uint8_t g_cmd_head = 0;
static volatile uint8_t g_cmd_tail = 0;

static void cmd_uart_rx_cb(const void *buffer, uint16_t length, bool error)
{
    if (error || buffer == NULL || length == 0u) { return; }
    const uint8_t *data = (const uint8_t *)buffer;
    for (uint16_t i = 0; i < length; i++) {
        uint8_t next = (uint8_t)((g_cmd_head + 1u) % CMD_QUEUE_SIZE);
        if (next != g_cmd_tail) {
            g_cmd_q[g_cmd_head] = data[i];
            g_cmd_head = next;
        }
    }
}

static void cmd_init(void)
{
    /* 以 115200 重新初始化 UART0（与启动波特率相同），配置 RX buffer */
    uapi_pin_set_mode(S_MGPIO17, PIN_MODE_1);  /* UART0_TX */
    uapi_pin_set_mode(S_MGPIO18, PIN_MODE_1);  /* UART0_RX */
    uart_attr_t ua = {
        .baud_rate = 115200u,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity    = UART_PARITY_NONE
    };
    uart_pin_config_t up = {
        .tx_pin  = S_MGPIO17,
        .rx_pin  = S_MGPIO18,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    uapi_uart_deinit(CMD_UART_BUS);
    uapi_uart_init(CMD_UART_BUS, &up, &ua, NULL, &g_cmd_buf_cfg);
    uapi_uart_register_rx_callback(CMD_UART_BUS,
        UART_RX_CONDITION_FULL_OR_IDLE, 1, cmd_uart_rx_cb);
}

static void cmd_print_help(void)
{
    osal_printk("\r\n[CMD] ===== MAX30102 Soft-I2C Debug =====\r\n");
    osal_printk("[CMD]  i  -- I2C probe + FIFO/结果状态\r\n");
    osal_printk("[CMD]  r  -- 寄存器全转储 (0x00-0x0D, 0xFF)\r\n");
    osal_printk("[CMD]  m  -- FIFO 强制读取 + 缓冲区状态\r\n");
    osal_printk("[CMD]  b  -- 重新初始化 MAX30102\r\n");
    osal_printk("[CMD]  c  -- 切换连续原始值打印 (500ms)\r\n");
    osal_printk("[CMD]  s  -- 当前 IR/Red/finger/HR/SpO2 快照\r\n");
    osal_printk("[CMD]  ?  -- 本帮助\r\n");
    osal_printk("[CMD] 硬件: I2C1 GPIO15=SDA GPIO16=SCL addr=0x57\r\n");
    osal_printk("[CMD] ========================================\r\n");
}

static volatile uint8_t g_continuous = 0;  /* 连续打印原始值模式 */

static void cmd_process(uint8_t c, int *p_max_ok)
{
    int max_ok = *p_max_ok;
    switch (c) {
    case 'i': case 'I':
        osal_printk("[CMD] >> I2C scan\r\n");
        max30102_i2c_scan();
        break;

    case 'r': case 'R':
        osal_printk("[CMD] >> Register dump\r\n");
        if (max_ok) { max30102_dump_regs(); }
        else { osal_printk("[CMD]   MAX30102 not initialized!\r\n"); }
        break;

    case 'm': case 'M':
        osal_printk("[CMD] >> Force read FIFO\r\n");
        if (max_ok) { max30102_force_read(); }
        else { osal_printk("[CMD]   MAX30102 not initialized!\r\n"); }
        break;

    case 'c': case 'C':
        g_continuous = !g_continuous;
        osal_printk("[CMD] Continuous raw print: %s\r\n",
                    g_continuous ? "ON (every 500ms)" : "OFF");
        break;

    case 's': case 'S':
        if (max_ok) {
            max30102_result_t mx = {0};
            max30102_get_result(&mx);
            osal_printk("[CMD] >> Snapshot: IR=%u RED=%u finger=%u HR=%u SpO2=%u\r\n",
                        (unsigned)mx.ir_raw, (unsigned)mx.red_raw,
                        mx.finger, mx.hr, mx.spo2);
            osal_printk("[CMD]   total_samples=%u\r\n",
                        (unsigned)s_total_samples_get());
        }
        break;

    case 't': case 'T':
        /* 强制 poll MAX30102 FIFO，立即更新缓冲区并打印 */
        if (max_ok) {
            int got = max30102_poll();
            max30102_result_t mx = {0};
            max30102_get_result(&mx);
            osal_printk("[CMD] >> Poll: got=%d  IR=%u RED=%u finger=%u total=%u\r\n",
                        got, (unsigned)mx.ir_raw, (unsigned)mx.red_raw,
                        mx.finger, (unsigned)s_total_samples_get());
        } else {
            osal_printk("[CMD]   MAX30102 未初始化，按 'b' 重新初始化\r\n");
        }
        break;

    case 'b': case 'B':
        /* 重新初始化 MAX30102（软件 I2C，不需要波特率切换）*/
        osal_printk("[CMD] >> Re-init MAX30102\r\n");
        *p_max_ok = (max30102_init() == 0);
        osal_printk("[CMD]   MAX30102 re-init: %s\r\n", *p_max_ok ? "OK" : "FAIL");
        break;

    case 'w': case 'W':
        /* Watch：I2C 状态快照 */
        max30102_i2c_scan();
        break;

    case '?': case 'h': case 'H':
        cmd_print_help();
        break;

    case '\r': case '\n': case ' ':
        break;  /* 忽略回车空格 */

    default:
        osal_printk("[CMD] Unknown: 0x%02X (press ? for help)\r\n", c);
        break;
    }
}

#endif /* SKIP_BMD101 */

/* ── 主任务 ───────────────────────────────────────────────────── */
static int sensor_hub_task(const char *arg)
{
    (void)arg;
    osal_msleep(500);

    osal_printk("\r\n[HUB] ===== 自绘板生理数据 多模式 Server =====\r\n");
    osal_printk("[HUB] MAX30102: HW-I2C1 GPIO15=SDA GPIO16=SCL (PCB)\r\n");
    osal_printk("[HUB] BMD101  : HW-UART2 GPIO7=RXD(Pin12) GPIO8=TXD(Pin13)\r\n");
    osal_printk("[HUB] SLE addr: 01:02:03:04:05:07  name=sle_test\r\n");
    osal_printk("[HUB] BLE addr: 11:22:33:44:55:AB  name=BIO_HUB\r\n");

    /* 1. SLE 初始化（必须在传感器之前）*/
    if (sle_server_setup() != ERRCODE_SUCC) {
        osal_printk("[HUB] SLE init FAIL\r\n");
    }

    /* 2. SLE 射频稳定等待 */
    osal_msleep(200);

    /* 3. BLE GATT Server 初始化（与 SLE 并行，供 PC/手机直连调试）*/
    if (ble_bio_server_init() != ERRCODE_SUCC) {
        osal_printk("[HUB] BLE init FAIL (SLE 仍正常运行)\r\n");
    }

    /* 4. 给 BLE 栈一点时间处理回调（广播会在 on_service_start 中异步启动）*/
    osal_msleep(100);

    /* 5. MAX30102 初始化（等待 1s 让 3.3V 在 SLE/BLE RF 启动后充分稳定）*/
    osal_msleep(1000);
    int max_ok = 0;
    for (int _attempt = 0; _attempt < 8 && !max_ok; _attempt++) {
        if (_attempt > 0) {
            osal_printk("[HUB] MAX30102 init retry %d/7 (500ms)...\r\n", _attempt);
            osal_msleep(500);
        }
        max_ok = (max30102_init() == 0);
    }
    if (!max_ok) {
        osal_printk("[HUB] MAX30102 init FAILED after 8 attempts\r\n");
    }
    osal_printk("[HUB] MAX30102 %s\r\n", max_ok ? "OK" : "FAIL");
    if (max_ok) { max30102_dump_regs(); }

#if SKIP_BMD101
    /* 6. 跳过 BMD101（UART0 保持 115200，CH340 串口可见）*/
    int bmd_ok = 0;
    osal_printk("[HUB] BMD101 SKIPPED (debug mode, UART0 stays 115200)\r\n");

    /* 7. 初始化 UART0 命令接口（需要在 MAX30102 init 之后）*/
    cmd_init();
    cmd_print_help();
#else
    /* 6. BMD101 硬件 UART2 初始化（不修改 UART0，串口保持 115200） */
    int bmd_ok = (bmd101_init() == 0);
    osal_printk("[HUB] BMD101 %s (HW-UART2 GPIO7=RXD GPIO8=TXD)\r\n",
                bmd_ok ? "OK" : "FAIL");
#endif

    osal_printk("[HUB] Sensor init done. Waiting SLE connection...\r\n");

    uint32_t loop     = 0;
    uint32_t notify_t = 0;   /* 通知计时（500ms）*/
    uint8_t  flags    = (uint8_t)((max_ok ? 1u : 0u) | (bmd_ok ? 2u : 0u));

    while (1) {
        loop++;
        g_bio_ms += 100u;

        /* 客户端存活检测：连接后超过 10s 没收到心跳写包 → 主动断连重广播 */
        if (g_connected) {
            g_last_rx_tick++;
            if (g_last_rx_tick >= CLIENT_TIMEOUT_TICKS) {
                g_last_rx_tick = 0;
                osal_printk("[SRV] Client timeout → disconnect & restart adv\r\n");
                sle_disconnect_remote_device(&g_client_addr);
                /* disconnect_cbk 会触发 sle_start_announce 重新广播 */
            }
        }

        /* MAX30102 启动失败时每 5s 自动重试（处理上电慢的情况）*/
        if (!max_ok && (loop % 50u) == 0u) {
            max_ok = (max30102_init() == 0);
            if (max_ok) {
                flags |= 1u;
                osal_printk("[HUB] MAX30102 auto-recovery OK loop=%u\r\n", loop);
            }
        }

        /* 轮询 MAX30102 */
        static int poll_ret_last = 0;
        if (max_ok) {
            poll_ret_last = max30102_poll();
        }

        /* 轮询 BMD101 ring buffer → ThinkGear 解析 */
        if (bmd_ok) {
            bmd101_poll();
        }

        /* 每 100ms 读取 ECG 流 + SLE 通知（E, 串口输出已移至 bmd101_drv） */
        if (bmd_ok) {
            int16_t ecg_buf[24];
            uint8_t ecg_n = bmd101_read_ecg_stream(ecg_buf, 24u);

            if (ecg_n > 0u && g_connected && !g_srv_paused) {
                uint8_t ecg_pkt[2u + 24u * 2u];
                ecg_pkt[0] = 0xECu;
                ecg_pkt[1] = ecg_n;
                (void)memcpy_s(&ecg_pkt[2], sizeof(ecg_pkt) - 2u,
                               ecg_buf, ecg_n * 2u);
                ssaps_ntf_ind_t ecg_ntf = {0};
                ecg_ntf.handle    = g_prop_hdl;
                ecg_ntf.type      = SSAP_PROPERTY_TYPE_VALUE;
                ecg_ntf.value     = ecg_pkt;
                ecg_ntf.value_len = (uint16_t)(2u + ecg_n * 2u);
                ssaps_notify_indicate(g_server_id, g_conn_id, &ecg_ntf);
            }
        }

#if SKIP_BMD101
        /* 处理 UART0 命令队列 */
        while (g_cmd_tail != g_cmd_head) {
            uint8_t c = g_cmd_q[g_cmd_tail];
            g_cmd_tail = (uint8_t)((g_cmd_tail + 1u) % CMD_QUEUE_SIZE);
            cmd_process(c, &max_ok);
        }
#endif

        /* 每 500ms 发一次 SLE Notify */
        notify_t++;
        if (notify_t >= 5u) {
            notify_t = 0;

            max30102_result_t mx = {0};
            bmd101_result_t   bm = {0};
            max30102_get_result(&mx);
            bmd101_get_result(&bm);

            /* PTT 计算：匹配最近的 ECG R-peak 和 PPG foot */
            uint16_t ptt_val = 0;
            uint8_t  ptt_ok  = 0;
            if (bm.rpeak_new && mx.ppg_foot_new) {
                int32_t ptt = (int32_t)(mx.ppg_foot_ms - bm.rpeak_tick_ms);
                if (ptt >= 50 && ptt <= 500) {
                    g_ptt_buf[g_ptt_idx] = (uint16_t)ptt;
                    g_ptt_idx = (uint8_t)((g_ptt_idx + 1u) % PTT_BUF_N);
                    if (g_ptt_cnt < PTT_BUF_N) { g_ptt_cnt++; }
                    ptt_val = ptt_median();
                    ptt_ok  = 1;
                }
            } else if (bm.rpeak_new && bm.rpeak_tick_ms != g_prev_rpeak_ms) {
                g_prev_rpeak_ms = bm.rpeak_tick_ms;
            } else if (mx.ppg_foot_new && mx.ppg_foot_ms != g_prev_foot_ms) {
                g_prev_foot_ms = mx.ppg_foot_ms;
                if (g_prev_rpeak_ms > 0u) {
                    int32_t ptt = (int32_t)(mx.ppg_foot_ms - g_prev_rpeak_ms);
                    if (ptt >= 50 && ptt <= 500) {
                        g_ptt_buf[g_ptt_idx] = (uint16_t)ptt;
                        g_ptt_idx = (uint8_t)((g_ptt_idx + 1u) % PTT_BUF_N);
                        if (g_ptt_cnt < PTT_BUF_N) { g_ptt_cnt++; }
                        ptt_val = ptt_median();
                        ptt_ok  = 1;
                    }
                }
            }

            bio_pkt_t pkt = {0};
            /* MAX30102 PPG 算法输出 */
            pkt.hr      = mx.hr;
            pkt.spo2    = mx.spo2;
            pkt.ir      = mx.ir_raw;
            pkt.temp_i  = mx.temp_int;
            pkt.temp_f  = mx.temp_frac;
            pkt.pi_x10  = mx.pi_x10;
            /* 血压 */
            pkt.ptt_ms  = ptt_val;
            if (ptt_ok && ptt_val > 0u) {
                pkt.sbp = ptt_to_sbp(ptt_val);
                pkt.dbp = (uint8_t)((uint16_t)pkt.sbp * 2u / 3u);
            }
            /* 状态标志 */
            pkt.flags = (uint8_t)(flags | (mx.finger ? 4u : 0u)
                                  | (ptt_ok ? 8u : 0u));
            /* BMD101 ECG */
            pkt.ecg_hr  = bm.heart_rate;
            pkt.ecg_sig = bm.poor_signal;
            pkt.ecg_raw = bm.raw_ecg;
            /* 临时：rr_ms 高8位=sig_update, 低8位=hr_update（BMD101诊断） */
            pkt.rr_ms   = (uint16_t)((bm.sig_update & 0xFFu) << 8u)
                        | (uint16_t)(bm.hr_update & 0xFFu);

            if (!g_srv_paused) {
                /* ① SLE Notify → 大板（SLE 通道，标准路径）*/
                send_bio_notify(&pkt);

                /* ② BLE Notify → PC（共享 GATT 层：SLE notify API + BLE conn_id）
                 *    Hi3863 GATT 数据库 SLE/BLE 共用，ssaps_notify_indicate 可通过
                 *    BLE conn_id 向 BLE 客户端路由通知。
                 *    PC 已订阅 SLE 注册的 char（0xCDEF / handle=g_prop_hdl）。 */
                if (ble_bio_is_connected()) {
                    ssaps_ntf_ind_t ble_ntf = {0};
                    ble_ntf.handle    = g_prop_hdl;
                    ble_ntf.type      = SSAP_PROPERTY_TYPE_VALUE;
                    ble_ntf.value     = (uint8_t *)&pkt;
                    ble_ntf.value_len = (uint16_t)sizeof(bio_pkt_t);
                    ssaps_notify_indicate(g_server_id,
                                         ble_bio_get_conn_id(),
                                         &ble_ntf);
                }
            }

            /* 每 500ms 输出一行 CSV 原始数据（10 个字段）
             * D,ms,ir,red,ecg_raw,ppg_hr,spo2,ecg_hr,ecg_sig,finger */
            osal_printk("D,%u,%u,%u,%d,%u,%u,%u,%u,%u\r\n",
                        (unsigned)g_bio_ms,
                        (unsigned)mx.ir_raw, (unsigned)mx.red_raw,
                        (int)bm.raw_ecg,
                        mx.hr, mx.spo2,
                        bm.heart_rate, (unsigned)bm.poor_signal,
                        mx.finger);

            /* 每 30s 打印一次状态摘要 */
            if ((loop % 300u) == 0u) {
                osal_printk("[HUB] loop=%u samp=%u poll=%d ntf=%u/%u SLE=%u\r\n",
                            loop, (unsigned)s_total_samples_get(),
                            poll_ret_last, g_ntf_ok, g_ntf_err, g_connected);
                if (bmd_ok) { bmd101_print_diag(); }
            }
        }

        osal_msleep(100);
    }
    return 0;
}

/* ── 入口 ─────────────────────────────────────────────────────── */
static void sensor_hub_entry(void)
{
    uapi_watchdog_disable();
    osal_kthread_lock();
    osal_task *task = osal_kthread_create(
        (osal_kthread_handler)sensor_hub_task, 0,
        "bio_server", TASK_STACK);
    if (task != NULL) {
        osal_kthread_set_priority(task, TASK_PRIO);
        osal_kfree(task);
    }
    osal_kthread_unlock();
}
app_run(sensor_hub_entry);
