
#===============================================================================
# @brief  cmake file - 大板 SLE 生理数据接收端 (COM7)
#
# 功能：
#   接收自绘板（SLE Server）发来的 bio_pkt_t，通过 COM7 串口输出 + SSD1306 显示
#
# 依赖：
#   SSD1306 驱动（peripheral/helloworld_oled）
#   SLE 系统头文件由 SDK 全局 include 路径提供
#
# 注意：不引用 Farsight sle_client.c（含 app_run，会冲突）
#===============================================================================

set(SSD1306_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../peripheral/helloworld_oled")

set(SOURCES "${SOURCES}"
    "${CMAKE_CURRENT_SOURCE_DIR}/sle_light_server.c"
    "${SSD1306_DIR}/ssd1306.c"
    "${SSD1306_DIR}/ssd1306_fonts.c"
    PARENT_SCOPE)

set(PUBLIC_HEADER "${PUBLIC_HEADER}"
    "${SSD1306_DIR}"
    PARENT_SCOPE)
