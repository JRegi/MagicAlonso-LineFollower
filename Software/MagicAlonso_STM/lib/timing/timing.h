#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>
#include <stdbool.h>

extern volatile bool control_tick_400hz;

void control_timer_init_400hz(void);

/** Inicializa SysTick (1 kHz) y DWT para micros.
 *  Debe llamarse después de clocks_init(). */
void timing_init(void);

/** Retorna ms y us desde arranque. */
uint32_t millis(void);
uint32_t micros(void);

/** Delays bloqueantes de precisión (usar solo en init o pruebas). */
void delay_ms_blocking(uint32_t ms);
void delay_us_blocking(uint32_t us);

/** No-blocking: timeout con micros; si expira actualiza *last_us. */
bool timeout_elapsed(uint32_t *last_us, uint32_t interval_us);

#endif // TIMING_H