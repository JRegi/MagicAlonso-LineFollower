#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ===== Defaults (overridable por build_flags) ===== */
#ifndef SERIALBT_USART
#define SERIALBT_USART USART1
#endif

#ifndef SERIALBT_TX_PORT
#define SERIALBT_TX_PORT GPIOA
#endif
#ifndef SERIALBT_TX_PIN
#define SERIALBT_TX_PIN  GPIO9
#endif

#ifndef SERIALBT_RX_PORT
#define SERIALBT_RX_PORT GPIOA
#endif
#ifndef SERIALBT_RX_PIN
#define SERIALBT_RX_PIN  GPIO10
#endif

/* KEY opcional para modo AT (0 = no usado) */
#ifndef SERIALBT_KEY_PORT
#define SERIALBT_KEY_PORT GPIOB
#endif
#ifndef SERIALBT_KEY_PIN
#define SERIALBT_KEY_PIN  GPIO12
#endif

#ifndef SERIALBT_RX_BUFSZ
#define SERIALBT_RX_BUFSZ 256
#endif

/* ===== API estilo Arduino, sin parámetros de pines/USART ===== */
void    SerialBT_begin(uint32_t baud);
void    SerialBT_end(void);

int     SerialBT_available(void);
int     SerialBT_peek(void);
int     SerialBT_read(void);
size_t  SerialBT_readBytes(uint8_t *buf, size_t len);
int     SerialBT_readBytesUntil(char term, char *buf, size_t len);
void    SerialBT_setTimeout(uint32_t ms);

size_t  SerialBT_write(const uint8_t *data, size_t len);
int     SerialBT_print(const char *s);
int     SerialBT_println(const char *s);
int     SerialBT_printf(const char *fmt, ...);

/* “Programar” el HC-05 por AT (nombre, pin, baud) usando KEY por defecto.
   Devuelve true si se pudo entrar a AT y setear todo. */
bool    SerialBT_program(const char *name, const char *pin4, uint32_t new_baud);

/* Inicialización básica del MCU (clocks a 72 MHz y pines USART1 PA9/PA10).
   Llamala una vez al inicio. */
void    board_clock_setup_72mhz(void);
void    board_usart_pins_setup_default(void);
