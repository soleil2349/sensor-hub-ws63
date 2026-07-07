/**
 * ble_bio_server.h — BLE 连接跟踪模块
 *
 * 在 Hi3863 上 SLE 和 BLE 共用同一个 GATT 数据库，gatts_add_service
 * 无法在 SLE 运行时注册新服务。
 *
 * 本模块仅做两件事：
 *   1. 监听 BLE 连接/断开事件（gap_ble_register_callbacks），
 *      记录 BLE 客户端的 conn_id。
 *   2. 暴露 conn_id 给 sensor_hub.c，由 sensor_hub.c 用
 *      ssaps_notify_indicate(g_server_id, ble_conn_id, g_prop_hdl)
 *      向 BLE 客户端发数据。
 *
 * 广播：设备名 "BIO_HUB"，BLE 地址 11:22:33:44:55:AB
 */
#ifndef BLE_BIO_SERVER_H
#define BLE_BIO_SERVER_H

#include "errcode.h"
#include <stdint.h>

/**
 * 初始化 BLE GAP 回调并开始广播 "BIO_HUB"。
 * 须在 SLE 初始化（sle_server_setup）之后调用。
 */
errcode_t ble_bio_server_init(void);

/** 返回当前 BLE 客户端的连接 ID（未连接时返回 0xFFFF）*/
uint16_t ble_bio_get_conn_id(void);

/** 返回 1 = BLE 客户端已连接；0 = 未连接 */
uint8_t ble_bio_is_connected(void);

#endif /* BLE_BIO_SERVER_H */
