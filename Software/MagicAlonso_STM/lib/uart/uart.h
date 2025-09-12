#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdarg.h>  // por si querés exponer uart_vprintf

// Inicializa USART3 en PB10 (TX) / PB11 (RX) a 115200-8N1 (bloqueante).
void uart_init_115200(void);

// Envío bloqueante
void uart_putc(char c);
void uart_write(const char *s);
void uart_printf(const char *fmt, ...);

// (Opcional) Lectura bloqueante simple con timeout en ms; devuelve -1 si expira.
// Si no la querés, podés no implementarla ni declararla.
int uart_getc_timeout(int timeout_ms);

#endif // UART_H
