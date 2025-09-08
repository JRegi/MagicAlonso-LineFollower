#pragma once
#include <stdint.h>
#include <libopencm3/stm32/timer.h>  // enum tim_oc_id

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t tim;          // TIM1..TIM4 base (p.ej. TIM1)
  enum tim_oc_id ch;     // TIM_OC1..TIM_OC4
  uint32_t gpio_port;    // GPIOA..GPIOE
  uint16_t gpio_pin;     // GPIOx
  uint16_t freq_hz;      // 50/400/2000 etc.
  uint16_t min_us;       // 1000 (PWM) o 125 (OneShot125)
  uint16_t max_us;       // 2000 (PWM) o 250 (OneShot125)
} esc_config_t;

typedef struct {
  esc_config_t cfg;
  uint16_t arr;      // ARR precalculado (periodo_us - 1)
  uint8_t  is_adv;   // TIM1/TIM8
} esc_handle_t;

/* Inicializa GPIO + Timer + Canal. Base de tiempo a 1 MHz (1 µs/tick).
 * Supone SYSCLK=72 MHz ya configurado.
 */
void esc_init(esc_handle_t* h, const esc_config_t* cfg);

/* Cambia el ancho en µs (saturado a [min_us, max_us]). */
void esc_write_us(esc_handle_t* h, uint16_t us);

#ifdef __cplusplus
}
#endif
