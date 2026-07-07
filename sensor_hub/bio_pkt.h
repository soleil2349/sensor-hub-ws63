/**
 * bio_pkt.h — 生理数据包定义 V2（自绘板 → 大板，双端必须完全一致）
 *
 * bio_pkt_t (20 字节, packed)：
 *   MAX30102 PPG 算法输出 + PTT 血压 + BMD101 ECG
 *
 * ctrl_pkt_t (4 字节)：
 *   大板 → 自绘板控制命令（全双工心跳/暂停）
 */
#ifndef BIO_PKT_H
#define BIO_PKT_H

#include <stdint.h>

#define BIO_PKT_VER  2u

/* ── 服务端 → 客户端：生理数据包（20 字节）────────────────────── */
typedef struct {
    /* MAX30102 PPG 算法输出 */
    uint8_t  hr;        /* PPG 心率 BPM（过零检测，0=无效/计算中）        */
    uint8_t  spo2;      /* 血氧饱和度 %（R 值查表，0=无效）               */
    uint32_t ir;        /* 原始 IR 均值（手指检测阈值 50000）             */
    int8_t   temp_i;    /* 芯片温度整数部分（℃）                          */
    uint8_t  temp_f;    /* 温度小数（低 4 位 × 0.0625℃）                  */
    uint8_t  pi_x10;    /* 灌注指数 × 10（25 → 2.5%）                     */
    /* PTT 血压估算 */
    uint8_t  sbp;       /* 收缩压 mmHg（0=无效）                          */
    uint8_t  dbp;       /* 舒张压 mmHg（0=无效）                          */
    uint8_t  flags;     /* bit0=max_ok bit1=bmd_ok bit2=finger bit3=ptt_valid */
    uint16_t ptt_ms;    /* 脉搏传导时间 ms（调试/校准用）                 */
    /* BMD101 ECG */
    uint8_t  ecg_hr;    /* BMD101 心率 BPM（0=无效）                       */
    uint8_t  ecg_sig;   /* ECG 信号质量（0=最好, 200=无接触）              */
    int16_t  ecg_raw;   /* ECG 原始采样值（int16，512SPS）                 */
    /* 元数据 */
    uint16_t rr_ms;     /* PPG R-R 间期 ms（用于 HRV 分析，0=无效）       */
} __attribute__((packed)) bio_pkt_t;   /* 共 20 字节 */

/* 全局毫秒计时器（sensor_hub.c 主循环每 100ms 递增，各驱动共享） */
extern volatile uint32_t g_bio_ms;

/* ── 客户端 → 服务端：控制包（4 字节）─────────────────────────── */
#define CMD_HEARTBEAT  0u
#define CMD_TOGGLE     1u
typedef struct {
    uint8_t  cmd;
    uint8_t  seq;
    uint16_t uptime_s;
} __attribute__((packed)) ctrl_pkt_t;

#endif /* BIO_PKT_H */
