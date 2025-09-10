#include "qre_array.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

// ====== UART (USART3) ======
static void usart3_setup_115200(void) {
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    // PB10 TX AF PP 50MHz, PB11 RX Floating
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,         GPIO_CNF_INPUT_FLOAT,            GPIO11);

    usart_set_baudrate(USART3, 115200);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_enable(USART3);
}

static void uart_putc(char c) { usart_send_blocking(USART3, (uint16_t)c); }
static void uart_write(const char *s) { while (*s) uart_putc(*s++); }
static void uart_printf(const char *fmt, ...) {
    char buf[64];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_write(buf);
}

// ====== Clocks ======
static void clocks_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // 72 MHz
}

static const uint8_t QRE_CH[8] = {7, 6, 5, 4, 3, 2, 0, 1};
qre_array_t qre;

int main(void) {
    clocks_setup();
    usart3_setup_115200();
    // Inicializar
    qre_init(&qre, QRE_CH, 8);

    // Calibrar (mueve el robot sobre línea y fondo)
    uart_write("CAL: mover sobre linea/fondo...\r\n");
    qre_calibrate(&qre, 400, 1000);
    uart_write("\r\nCAL OK\r\n");

    while (1) {
        uint16_t pos = qre_read_position_black(&qre); // false = línea negra
        uart_printf("%d\r\n", pos);
    }
}