/**
 * ble_bio_server.c — BLE 连接跟踪 + 广播（最终版）
 *
 * Hi3863 SLE 和 BLE 共用 GATT 数据库，运行 SLE 时无法通过
 * gatts_add_service 注册新 BLE 服务。
 *
 * 策略：
 *   - 不注册新 GATT 服务
 *   - 只注册 GAP BLE 回调（跟踪 BLE 连接/断开）
 *   - 广播名 "BIO_HUB" 让 PC 找到板子
 *   - sensor_hub.c 获取 BLE conn_id 后，自行用
 *     ssaps_notify_indicate(g_server_id, ble_conn_id, g_prop_hdl)
 *     向 BLE 客户端发通知（SLE notify API + BLE conn_id → 共享 GATT 层路由）
 */

#include "ble_bio_server.h"

#include "securec.h"
#include "errcode.h"
#include "soc_osal.h"
#include "bts_def.h"
#include "bts_le_gap.h"
#include <string.h>

/* ── 状态 ─────────────────────────────────────────────────────── */
#define BLE_INVALID_CONN  0xFFFFu

static uint16_t         g_ble_conn_id = BLE_INVALID_CONN;
static volatile uint8_t g_ble_conn    = 0;

/* ── 广播配置 ──────────────────────────────────────────────────── */
#define BLE_BIO_NAME      "BIO_HUB"
#define BLE_BIO_NAME_LEN  7u
#define BLE_ADV_HANDLE    1u
#define ADV_BUF_LEN       64u

static uint8_t g_ble_addr[BD_ADDR_LEN] =
    {0x11, 0x22, 0x33, 0x44, 0x55, 0xAB};

/* ── 广播启动 ──────────────────────────────────────────────────── */
static void ble_bio_start_adv(void)
{
    gap_ble_adv_params_t par = {0};
    par.min_interval      = 0x30u;
    par.max_interval      = 0x60u;
    par.adv_type          = 0u;   /* ADV_IND */
    par.adv_filter_policy = 0u;
    par.channel_map       = 7u;
    par.duration          = 0u;
    (void)memset_s(&par.peer_addr, sizeof(par.peer_addr), 0, sizeof(par.peer_addr));
    gap_ble_set_adv_param(BLE_ADV_HANDLE, &par);

    uint8_t adv[ADV_BUF_LEN] = {0};
    uint8_t rsp[ADV_BUF_LEN] = {0};
    uint8_t ai = 0, ri = 0;

    /* Flags */
    adv[ai++] = 2u; adv[ai++] = 0x01u; adv[ai++] = 0x06u;
    /* Shortened Local Name */
    adv[ai++] = (uint8_t)(BLE_BIO_NAME_LEN + 1u);
    adv[ai++] = 0x08u;
    (void)memcpy_s(&adv[ai], ADV_BUF_LEN - ai, BLE_BIO_NAME, BLE_BIO_NAME_LEN);
    ai += BLE_BIO_NAME_LEN;

    /* Scan Response: Complete Local Name */
    rsp[ri++] = (uint8_t)(BLE_BIO_NAME_LEN + 1u);
    rsp[ri++] = 0x09u;
    (void)memcpy_s(&rsp[ri], ADV_BUF_LEN - ri, BLE_BIO_NAME, BLE_BIO_NAME_LEN);
    ri += BLE_BIO_NAME_LEN;

    gap_ble_config_adv_data_t cfg = {0};
    cfg.adv_data        = adv;  cfg.adv_length        = ai;
    cfg.scan_rsp_data   = rsp;  cfg.scan_rsp_length   = ri;
    gap_ble_set_adv_data(BLE_ADV_HANDLE, &cfg);

    errcode_t ret = gap_ble_start_adv(BLE_ADV_HANDLE);
    osal_printk("[BLE] adv ret=%d  name=%s\r\n", ret, BLE_BIO_NAME);
}

/* ── GAP 回调 ─────────────────────────────────────────────────── */
static void on_ble_enable(errcode_t status)
{
    osal_printk("[BLE] stack enable st=%d\r\n", status);
}

static void on_adv_start(uint8_t id, adv_status_t st)
{
    osal_printk("[BLE] adv_start id=%u st=%u\r\n", id, st);
}

static void on_conn_state_change(uint16_t conn_id, bd_addr_t *addr,
                                  gap_ble_conn_state_t state,
                                  gap_ble_pair_state_t pair,
                                  gap_ble_disc_reason_t reason)
{
    (void)addr; (void)pair; (void)reason;
    if (state == GAP_BLE_STATE_CONNECTED) {
        g_ble_conn_id = conn_id;
        g_ble_conn    = 1;
        osal_printk("[BLE] PC connected  conn_id=%u\r\n", conn_id);
    } else if (state == GAP_BLE_STATE_DISCONNECTED) {
        g_ble_conn    = 0;
        g_ble_conn_id = BLE_INVALID_CONN;
        osal_printk("[BLE] PC disconnected → re-adv\r\n");
        ble_bio_start_adv();
    }
}

/* ── 公共接口 ──────────────────────────────────────────────────── */

errcode_t ble_bio_server_init(void)
{
    gap_ble_callbacks_t gap_cb = {0};
    gap_cb.ble_enable_cb        = on_ble_enable;
    gap_cb.start_adv_cb         = on_adv_start;
    gap_cb.conn_state_change_cb = on_conn_state_change;
    errcode_t ret = gap_ble_register_callbacks(&gap_cb);
    if (ret != ERRCODE_BT_SUCCESS) {
        osal_printk("[BLE] gap_cb reg fail %x\r\n", ret);
        return ERRCODE_FAIL;
    }

    /* 启动 BLE 射频（异步）*/
    enable_ble();

    /* 设置广播地址（11:22:33:44:55:AB）*/
    bd_addr_t addr = {0};
    addr.type = 0;
    (void)memcpy_s(addr.addr, BD_ADDR_LEN, g_ble_addr, BD_ADDR_LEN);
    gap_ble_set_local_addr(&addr);

    /* 开始广播，供 PC 搜索 */
    ble_bio_start_adv();

    osal_printk("[BLE] init OK  addr=11:22:33:44:55:AB\r\n");
    return ERRCODE_SUCC;
}

uint16_t ble_bio_get_conn_id(void)
{
    return g_ble_conn_id;
}

uint8_t ble_bio_is_connected(void)
{
    return g_ble_conn;
}
