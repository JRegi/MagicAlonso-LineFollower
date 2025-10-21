#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/dwt.h>
#include "esc.h"
#include "timing.h"

/* -------- Helpers privados -------- */
static int is_adv_timer(uint32_t tim) { return (tim == TIM1) || (tim == TIM8); }

static void set_oc_mode(uint32_t tim, enum tim_oc_id ch) {
  timer_set_oc_mode(tim, ch, TIM_OCM_PWM1);
  timer_enable_oc_preload(tim, ch);
  timer_enable_oc_output(tim, ch);
}

/* -------- API mínima -------- */
void esc_init(esc_handle_t* h, const esc_config_t* cfg)
{
  if (!h || !cfg || !cfg->freq_hz) return; // guardia mínima
  uint32_t period_us = 1000000UL / cfg->freq_hz; // base 1 MHz → µs

  h->cfg   = *cfg;
  h->is_adv = is_adv_timer(cfg->tim);
  h->arr   = (uint16_t)(period_us - 1);

  /* Pin AF push-pull (F1) */
  gpio_set_mode(cfg->gpio_port, GPIO_MODE_OUTPUT_10_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, cfg->gpio_pin);

  /* Timer: base 1 MHz (1 µs/tick), edge-aligned, up */
  timer_set_mode(cfg->tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  // Suponiendo TIM1 en APB2 @72 MHz: obtenerlo desde libopencm3 si disponible
  uint32_t timclk_hz = rcc_apb2_frequency; // en F1 el timer clock = apb2 * 1 (o *2 si prescalado >1)
  uint16_t psc = (uint16_t)((timclk_hz / 1000000UL) - 1UL);
  timer_set_prescaler(cfg->tim, psc);

  timer_set_period(cfg->tim, h->arr);         // ARR = periodo_us - 1
  timer_enable_preload(cfg->tim);
  timer_continuous_mode(cfg->tim);

  /* Canal PWM */
  set_oc_mode(cfg->tim, cfg->ch);
  timer_set_oc_value(cfg->tim, cfg->ch, 0);

  /* TIM1/TIM8 requieren habilitar MOE para sacar PWM */
  if (h->is_adv) { timer_enable_break_main_output(cfg->tim); }

  timer_generate_event(cfg->tim, TIM_EGR_UG); // aplica PSC/ARR/CCR
  timer_enable_counter(cfg->tim);
}

void esc_write_us(esc_handle_t* h, uint16_t us)
{
  if (!h) return;
  // if (us < h->cfg.min_us) us = h->cfg.min_us;
  // if (us > h->cfg.max_us) us = h->cfg.max_us;
  timer_set_oc_value(h->cfg.tim, h->cfg.ch, us);
}

void esc_calibrate(esc_handle_t* h)
{
  if (!h) return;

  esc_write_us(h, 0);
  delay_ms_blocking(200);

  esc_write_us(h, 1000);
  delay_ms_blocking(800);

  esc_write_us(h, 2000);
  delay_ms_blocking(5100);

  esc_write_us(h, 1000);
  delay_ms_blocking(4100);
}

void esc_arm(esc_handle_t* h)
{
  if (!h) return;

  // Min 500ms
  esc_write_us(h, 1000);
  delay_ms_blocking(1000);

    // 1500us 500ms
  esc_write_us(h, 1400);
  delay_ms_blocking(800);

    // Min 500ms
  esc_write_us(h, 1000);
  delay_ms_blocking(1000);
}