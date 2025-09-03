#pragma once
#include <stdint.h>
#include <libopencm3/stm32/timer.h>  // enum tim_oc_id

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ESC_OK = 0,
  ESC_ERR_BADARG,
  ESC_ERR_UNSUPPORTED
} esc_result_t;

typedef enum {
  ESC_STATE_DISARMED = 0,
  ESC_STATE_ARMING,
  ESC_STATE_ARMED
} esc_state_t;

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
  uint16_t arr;          // ARR actual (cacheado)
  uint8_t  is_adv;       // TIM1/TIM8 -> 1
  esc_state_t state;
  uint32_t arm_until_ms; // para arming no bloqueante
} esc_handle_t;

/* Inicializa GPIO + Timer + Canal, ARR según freq, duty en min_us.
 * Requiere que HSE->72MHz ya esté configurado (p. ej. en tu system_init).
 */
esc_result_t esc_init(esc_handle_t* h, const esc_config_t* cfg);

/* Cambia el ancho en μs (saturado al rango [min_us, max_us]). */
void esc_write_us(esc_handle_t* h, uint16_t us);

/* Normalizado 0..1 → mapea a [min_us, max_us]. */
void esc_write_norm(esc_handle_t* h, float n01);

/* Arranque seguro: mantiene min_us por 'duration_ms' de forma no bloqueante. */
void esc_begin_arming(esc_handle_t* h, uint32_t duration_ms, uint32_t now_ms);

/* Avanza el estado de arming (llamar periódicamente con tu tick ms). */
void esc_update(esc_handle_t* h, uint32_t now_ms);

/* Consulta de estado. */
static inline esc_state_t esc_state(const esc_handle_t* h) { return h->state; }

#ifdef __cplusplus
}
#endif