#ifndef BT_HC05_H
#define BT_HC05_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Inicializa USART3 (PB10=TX, PB11=RX) al baudrate indicado, 8N1, sin flow control.
 * No bloqueante: habilita RXNE/TXE por interrupción y usa buffers circulares. */
void bt_init(uint32_t baudrate);

/* Cola datos para transmitir (no bloqueante). Devuelve cuántos bytes aceptó. */
size_t bt_write(const void *data, size_t length);

/* Azúcar para strings (no bloqueante). Devuelve bytes aceptados. */
size_t bt_write_string(const char *text);

/* ¿Cuántos bytes hay disponibles para leer (RX)? */
size_t bt_available(void);

/* Lee un byte si hay datos (no bloqueante). Devuelve true si leyó uno. */
bool bt_read_byte(uint8_t *out_byte);

/* Lee hasta 'max_length' bytes disponibles. Devuelve cantidad leída. */
size_t bt_read(uint8_t *out_buffer, size_t max_length);

#endif /* BT_HC05_H */
