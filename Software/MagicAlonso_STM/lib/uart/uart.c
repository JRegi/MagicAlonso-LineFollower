#include "uart.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <stdarg.h>

// --- Si ya tenés delay_ms en otra lib, podés declarar extern y usarla aquí.
// extern void delay_ms(uint32_t ms);

void uart_init_115200(void) {
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    // PB10 = TX (AF Push-Pull 50 MHz), PB11 = RX (Input Floating)
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO11);

    usart_set_baudrate(USART3, 115200);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_enable(USART3);
}

void uart_putc(char c) {
    usart_send_blocking(USART3, (uint16_t)c);
}

void uart_write(const char *s) {
    while (*s) uart_putc(*s++);
}

void uart_printf(const char *fmt, ...) {
    char buf[64];  // ajustá si necesitás líneas más largas
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_write(buf);
}

// --- Opcional: lectura bloqueante con timeout rudimentario ---
static int _uart_rx_ready(void) {
    return usart_get_flag(USART3, USART_SR_RXNE) != 0;
}

// Implementación simple de timeout (en "vueltas"); si tenés delay_ms, cambiá por tiempo real.
int uart_getc_timeout(int timeout_ms) {
    // Si ya tenés delay_ms real, hacé:
    // int t = timeout_ms;
    // while (t-- > 0) { if (_uart_rx_ready()) return usart_recv(USART3); delay_ms(1); }
    // return -1;

    // Fallback: bucle aproximado (no usa delay_ms)
    volatile int spins = timeout_ms * 6000; // aprox a 72MHz; ajustar si querés
    while (spins-- > 0) {
        if (_uart_rx_ready())
            return (int)usart_recv(USART3);
        __asm__("nop");
    }
    return -1;
}
