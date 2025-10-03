#ifndef HC05_H
#define HC05_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ============================================================
   Config por defecto (Blue Pill F103 + USART3 en PB10/PB11)
   Si querés otro USART/pines/IRQ, redefiní estos macros ANTES
   de incluir este header.
   ============================================================ */
#ifndef BT_USART
  #include <libopencm3/stm32/usart.h>
  #define BT_USART            USART3
#endif
#ifndef BT_RCC_USART
  #include <libopencm3/stm32/rcc.h>
  #define BT_RCC_USART        RCC_USART3
#endif
#ifndef BT_NVIC_USART_IRQ
  #include <libopencm3/stm32/f1/nvic.h>
  #define BT_NVIC_USART_IRQ   NVIC_USART3_IRQ
#endif
#ifndef BT_USART_ISR
  /* Nombre de la ISR real en libopencm3 para este USART */
  #define BT_USART_ISR        usart3_isr
#endif
#ifndef BT_PORT
  #include <libopencm3/stm32/gpio.h>
  #define BT_PORT             GPIOB
  #define BT_TX_PIN           GPIO10   /* PB10: USART3_TX */
  #define BT_RX_PIN           GPIO11   /* PB11: USART3_RX */
  #define BT_RCC_GPIO         RCC_GPIOB
#endif

/* Tamaño de ring-buffers (potencias de 2) */
#ifndef BT_TX_RB_SIZE
  #define BT_TX_RB_SIZE 256
#endif
#ifndef BT_RX_RB_SIZE
  #define BT_RX_RB_SIZE 256
#endif

/* ================== API pública ================== */

/** Inicializa GPIO/USART y habilita RX por interrupciones.
 *  Llamá antes en tu main() a rcc_clock_setup_in_* para configurar el clock.
 *  @param baud 9600 / 38400 / 115200 ... (según tu HC-05)
 *  @param nvic_prio prioridad NVIC (0 = alta, 15 = baja). Ej: 12 para UART.
 */
void bt_init(uint32_t baud, uint8_t nvic_prio);

/** Encola datos para TX (no bloqueante). Devuelve cuántos bytes aceptó. */
size_t bt_write(const void *data, size_t len);

/** Encola un byte. Devuelve true si lo encoló. */
bool bt_write_byte(uint8_t b);

/** Encola una cadena terminada en '\0'. Devuelve bytes aceptados (sin el '\0'). */
size_t bt_write_string(const char *s);

/** Variante bloqueante simple: reintenta hasta encolar todo. */
void bt_write_all(const void *data, size_t len);

/** Lee un byte si hay (no bloqueante). Devuelve true si leyó. */
bool bt_read_byte(uint8_t *out);

/** Lee hasta maxlen bytes disponibles. Devuelve cantidad leída. */
size_t bt_read(void *dst, size_t maxlen);

/** Bytes disponibles en RX (aprox, no atómico frente a ISR). */
size_t bt_rx_available(void);

/** Espacio libre en TX (aprox). */
size_t bt_tx_free(void);

/** Cuenta y limpia overrun de RX (software). */
uint32_t bt_get_and_clear_rx_overruns(void);

/* -------- Helpers binarios (little-endian) -------- */
void bt_send_u16_le(uint16_t v);
void bt_send_i16_le(int16_t v);
void bt_send_u32_le(uint32_t v);
void bt_send_i32_le(int32_t v);
void bt_send_float_le(float f);

/* -------- Helpers de texto livianos -------- */
void bt_print_uint(uint32_t v);    /* decimal */
void bt_print_int(int32_t v);      /* decimal con signo */
void bt_println(const char *s);    /* s + "\r\n" */

#ifdef __cplusplus
}
#endif
#endif /* HC05_H */