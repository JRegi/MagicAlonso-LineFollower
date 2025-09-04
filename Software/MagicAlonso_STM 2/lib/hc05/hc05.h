//// FILE: include/hc05.h
#ifndef HC05_H
#define HC05_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t usart;        // USART1/2/3
    uint32_t rcc_usart;    // RCC_USARTx
    uint32_t tx_port;      // GPIOx
    uint16_t tx_pin;       // GPIO10, etc.
    uint32_t rx_port;      // GPIOx
    uint16_t rx_pin;       // GPIO11, etc.
    uint32_t baud;         // 9600..115200
    bool     use_rx;       // true para RX
} hc05_cfg_t;

typedef struct { hc05_cfg_t cfg; } hc05_t;

void hc05_init(hc05_t *h, const hc05_cfg_t *cfg);
void hc05_send_str(hc05_t *h, const char *s);
void hc05_printf(hc05_t *h, const char *fmt, ...);
bool hc05_rx_ready(hc05_t *h);
int  hc05_getc(hc05_t *h);

#endif // HC05_H