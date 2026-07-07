#ifndef MAX30102_DRV_H
#define MAX30102_DRV_H

#include <stdint.h>

/*
 * 自绘板 U2 MAX30102 软件 I2C 驱动（飞线）
 *
 * 飞线接线：
 *   MCU GPIO_0 (U6 引脚 4)  →  U2 SCL (引脚 3)  [飞线]
 *   MCU GPIO_1 (U6 引脚 5)  →  U2 SDA (引脚 2)  [飞线]
 *   U2 VIN (引脚 1)         =  3.3V (R7 已移除)
 *   U2 INT (引脚 5)         =  3.3V (禁用中断)
 *
 * 注意：模块 IRD/RD 引脚（pin6/7）经实测未连通 SDA/SCL，不可用于 I2C
 *
 * I2C 地址：0x57 (7-bit)
 * 通信方式：软件 bit-bang（GPIO0/GPIO1，PIN_MODE_0）
 */

/* 传感器测量结果（含算法输出） */
typedef struct {
    uint8_t  hr;           /* PPG 心率 BPM, 0=无效 */
    uint8_t  spo2;         /* 血氧 %, 0=无效 */
    uint8_t  pi_x10;       /* 灌注指数 × 10 */
    uint8_t  finger;       /* 1=检测到手指 */
    uint32_t ir_raw;       /* IR 原始均值 */
    uint32_t red_raw;      /* Red 原始均值 */
    int8_t   temp_int;     /* 芯片温度整数部分 ℃ */
    uint8_t  temp_frac;    /* 芯片温度小数（低 4 位 × 0.0625℃）*/
    uint16_t rr_interval;  /* PPG R-R 间期 ms（最近一次心跳间隔）*/
    uint32_t ppg_foot_ms;  /* 最近 PPG 脉搏脚点时间戳 ms */
    uint8_t  ppg_foot_new; /* 新脚点检测标志（get_result 后自动清零）*/
} max30102_result_t;

/* 核心 API */
int      max30102_init(void);
int      max30102_poll(void);
void     max30102_get_result(max30102_result_t *out);
uint32_t s_total_samples_get(void);

/* 诊断 API */
void     max30102_i2c_scan(void);                          /* I2C 总线探测 + 状态打印 */
void     max30102_dump_regs(void);                         /* 寄存器转储 */
void     max30102_force_read(void);                        /* 强制读 FIFO 并打印 */
void     max30102_set_baud(uint32_t baud);                 /* 兼容存根（软 I2C 不使用）*/
void     max30102_send(const uint8_t *data, uint16_t len); /* 兼容存根 */

#endif /* MAX30102_DRV_H */
