
#===============================================================================
# @brief  cmake file - 自绘板生理数据 SLE Server
#
# 传感器：
#   MAX30102  硬件 I2C1  GPIO15=SDA  GPIO16=SCL  (PCB 直连 + 2.2K上拉)
#   BMD101    硬件 UART2  GPIO7=RX   GPIO8=TX   (PCB 直连)
#
# SLE：复用 Farsight sle_01_trans_server 的广播模块（sle_server_adv.c）
#       SSAP Server 服务注册直接在 sensor_hub.c 中使用同步 API
#
# 注意：不引用 sle_server.c（含 app_run 会冲突）
#===============================================================================

set(SLE_SERVER_ADV_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Farsight/sle_01_trans_server")

set(SOURCES "${SOURCES}"
    "${CMAKE_CURRENT_SOURCE_DIR}/sensor_hub.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/max30102_drv.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/bmd101_drv.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/ble_bio_server.c"
    "${SLE_SERVER_ADV_DIR}/sle_server_adv.c"
    PARENT_SCOPE)

set(PUBLIC_HEADER "${PUBLIC_HEADER}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${SLE_SERVER_ADV_DIR}"
    PARENT_SCOPE)
