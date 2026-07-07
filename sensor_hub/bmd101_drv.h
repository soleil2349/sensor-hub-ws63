#ifndef BMD101_DRV_H
#define BMD101_DRV_H

#include <stdint.h>

typedef struct {
    uint8_t  poor_signal;   /* 0=best, 200=no signal */
    uint8_t  heart_rate;    /* BPM from BMD101 */
    uint8_t  hr_avg8s;      /* 8-second average HR */
    int16_t  raw_ecg;       /* raw ECG sample (+-32768) */
    uint32_t pkt_count;     /* total valid packets */
    uint32_t byte_count;    /* total bytes received by ThinkGear parser */
    uint32_t uart_rx_total; /* total raw bytes received from UART DMA */
    uint32_t uart_rx_error; /* UART error callback count */
    uint32_t chk_fail;      /* ThinkGear checksum failures */
    uint32_t rpeak_tick_ms; /* R-peak timestamp (g_bio_ms based) */
    uint8_t  rpeak_new;     /* new R-peak flag (cleared by get_result) */
    uint16_t sig_update;    /* code 0x02 收到次数（诊断用） */
    uint16_t hr_update;     /* code 0x03 收到次数（诊断用） */
} bmd101_result_t;

int  bmd101_init(void);
void bmd101_poll(void);
void bmd101_get_result(bmd101_result_t *out);
/* 向 BMD101 发送数据（硬件 UART2 TX） */
void bmd101_send(const uint8_t *data, uint16_t len);
/* 打印诊断信息（first 16 bytes raw hex） */
void bmd101_print_diag(void);

/* ECG 流式读取：从内部缓冲区取出滤波后的 ECG 样本，返回实际读取个数 */
uint8_t bmd101_read_ecg_stream(int16_t *buf, uint8_t max_count);

#endif
