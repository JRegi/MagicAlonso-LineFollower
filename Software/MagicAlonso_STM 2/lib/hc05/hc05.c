//// FILE: src/hc05.c
#include "hc05.h"
#include <stdarg.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void _uart_write(uint32_t usart, const char *s){ while(*s){ usart_send_blocking(usart, *s++); } }

void hc05_init(hc05_t *h, const hc05_cfg_t *cfg){
    h->cfg=*cfg;
    rcc_periph_clock_enable(h->cfg.rcc_usart);
    gpio_set_mode(h->cfg.tx_port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, h->cfg.tx_pin);
    if (h->cfg.use_rx){ gpio_set_mode(h->cfg.rx_port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, h->cfg.rx_pin); }
    usart_set_baudrate(h->cfg.usart, h->cfg.baud);
    usart_set_databits(h->cfg.usart, 8);
    usart_set_parity(h->cfg.usart, USART_PARITY_NONE);
    usart_set_stopbits(h->cfg.usart, USART_STOPBITS_1);
    usart_set_flow_control(h->cfg.usart, USART_FLOWCONTROL_NONE);
    usart_set_mode(h->cfg.usart, h->cfg.use_rx?USART_MODE_TX_RX:USART_MODE_TX);
    usart_enable(h->cfg.usart);
}

void hc05_send_str(hc05_t *h, const char *s){ _uart_write(h->cfg.usart, s); }

void hc05_printf(hc05_t *h, const char *fmt, ...){
    char buf[192]; va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap); _uart_write(h->cfg.usart, buf);
}

bool hc05_rx_ready(hc05_t *h){ return usart_get_flag(h->cfg.usart, USART_SR_RXNE); }
int  hc05_getc(hc05_t *h){ if (!hc05_rx_ready(h)) return -1; return (int)usart_recv(h->cfg.usart); }