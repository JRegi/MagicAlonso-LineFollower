#ifndef CLOCKS_H
#define CLOCKS_H

#include <stdint.h>

/** Inicializa el sistema: PLL 72MHz, HSE 8MHz, y habilita periféricos usados por el robot.
 *  Esta función agrupa las acciones que FujitoraBot hace en su setup.c. */
void clocks_init(void);

/** Habilita clocks periféricos adicionales si un driver lo necesita. */
void clocks_enable_periph(uint32_t rcc_periph);

/** Devuelve la frecuencia AHB (SystemCoreClock) en Hz. */
uint32_t clocks_get_hz(void);

/** Configura prioridades NVIC comunes (SysTick, timers críticos) */
void clocks_default_nvic_priorities(void);

#endif // CLOCKS_H
