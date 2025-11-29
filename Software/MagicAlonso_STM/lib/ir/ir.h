#ifndef IR_SIRC_H
#define IR_SIRC_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Driver IR Sony SIRC (12 bits) para STM32F103 + libopencm3
 *
 * - Receptor VS1838 en PB7.
 * - Usa EXTI7 + NVIC_EXTI9_5_IRQ para capturar flancos descendentes.
 * - Usa micros() de timing.c para medir tiempos entre flancos.
 * - Guarda UN solo código en Flash (última página, 0x0800FC00 en F103C8).
 *
 * Requisitos:
 *   - Haber llamado antes a:
 *       clocks_init();
 *       timing_init();
 *   - No tener otra implementación de exti9_5_isr en el proyecto
 *     (si la tenés, hay que fusionar a mano la lógica).
 */

/** Inicializa GPIO PB7, EXTI7, NVIC y carga el código guardado en Flash (si lo hay). */
void ir_sirc_init(void);

/** Devuelve true si hay un código válido guardado en Flash. */
bool ir_sirc_has_saved_code(void);

/** Devuelve el código guardado (12 bits) en Flash (no valida magic). */
uint16_t ir_sirc_get_saved_code(void);

/** Borra la página de Flash usada y vuelve a modo “aprender primer código”. */
void ir_sirc_clear_flash_and_reset(void);

/**
 * Servicio no bloqueante del IR.
 *
 * Debe llamarse frecuentemente en el main loop.
 *
 * Params (pueden ser NULL si no los necesitás):
 *   - learned_now:
 *       *true* si EN ESTA LLAMADA se aprendió y guardó un código en Flash
 *       (es decir, pasó de modo APRENDER -> MODO NORMAL).
 *
 *   - match_saved:
 *       *true* si EN ESTA LLAMADA se recibió un frame que coincide exactamente
 *       con el código guardado en Flash.
 *
 *   - last_code:
 *       si no es NULL, se escribe el último código IR decodificado (12 bits),
 *       independientemente de si coincide o no con el guardado.
 *
 * Retorno:
 *   - true  => se procesó al menos un frame IR en esta llamada.
 *   - false => no había frames pendientes.
 */
bool ir_sirc_service(bool *learned_now, bool *match_saved, uint16_t *last_code);

#endif /* IR_SIRC_H */
