#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "esc/esc.h"

static int is_adv_timer(uint32_t tim) { return (tim == TIM1) || (tim == TIM8); }

static void enable_rcc_gpio(uint32_t port) {
  if (port == GPIOA) rcc_periph_clock_enable(RCC_GPIOA);
  else if (port == GPIOB) rcc_periph_clock_enable(RCC_GPIOB);
  else if (port == GPIOC) rcc_periph_clock_enable(RCC_GPIOC);
  else if (port == GPIOD) rcc_periph_clock_enable(RCC_GPIOD);
  else if (port == GPIOE) rcc_periph_clock_enable(RCC_GPIOE);
}

static esc_result_t enable_rcc_tim(uint32_t tim) {
  if (tim == TIM1) rcc_periph_clock_enable(RCC_TIM1);
  else if (tim == TIM2) rcc_periph_clock_enable(RCC_TIM2);
  else if (tim == TIM3) rcc_periph_clock_enable(RCC_TIM3);
  else if (tim == TIM4) rcc_periph_clock_enable(RCC_TIM4);
  else return ESC_ERR_UNSUPPORTED;
  return ESC_OK;
}

static void set_oc_mode(uint32_t tim, enum tim_oc_id ch) {
  timer_set_oc_mode(tim, ch, TIM_OCM_PWM1);
  timer_enable_oc_preload(tim, ch);
  timer_enable_oc_output(tim, ch);
}

esc_result_t esc_init(esc_handle_t* h, const esc_config_t* cfg)
{
  if (!h || !cfg || !cfg->freq_hz || cfg->min_us >= cfg->max_us) return ESC_ERR_BADARG;
  uint32_t period_us = 1000000UL / cfg->freq_hz;        // base 1 MHz → μs
  if (period_us < cfg->max_us) return ESC_ERR_BADARG;   // no entra el pulso max

  h->cfg = *cfg;
  h->is_adv = is_adv_timer(cfg->tim);
  h->arr = (uint16_t)(period_us - 1);
  h->state = ESC_STATE_DISARMED;
  h->arm_until_ms = 0;

  enable_rcc_gpio(cfg->gpio_port);
  if (enable_rcc_tim(cfg->tim) != ESC_OK) return ESC_ERR_UNSUPPORTED;

  // Pin AF push-pull 50 MHz
  gpio_set_mode(cfg->gpio_port, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, cfg->gpio_pin);

  // Timer: base 1 MHz (1 μs por tick), edge-aligned, upcounting
  timer_set_mode(cfg->tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(cfg->tim, 71);                   // 72 MHz / 72 = 1 MHz
  timer_set_period(cfg->tim, h->arr);                  // ARR = period_us - 1
  timer_enable_preload(cfg->tim);
  timer_continuous_mode(cfg->tim);

  // Canal PWM
  set_oc_mode(cfg->tim, cfg->ch);
  timer_set_oc_value(cfg->tim, cfg->ch, cfg->min_us);

  // TIM1/TIM8 requieren MOE
  if (h->is_adv) { timer_enable_break_main_output(cfg->tim); }

  timer_generate_event(cfg->tim, TIM_EGR_UG);          // aplica PSC/ARR/CCR
  timer_enable_counter(cfg->tim);
  return ESC_OK;
}

void esc_write_us(esc_handle_t* h, uint16_t us)
{
  if (!h) return;
  if (us < h->cfg.min_us) us = h->cfg.min_us;
  if (us > h->cfg.max_us) us = h->cfg.max_us;
  timer_set_oc_value(h->cfg.tim, h->cfg.ch, us);
}

void esc_write_norm(esc_handle_t* h, float n01)
{
  if (!h) return;
  if (n01 < 0.f) n01 = 0.f;
  if (n01 > 1.f) n01 = 1.f;
  uint16_t us = (uint16_t)(h->cfg.min_us + (n01 * (h->cfg.max_us - h->cfg.min_us)));
  esc_write_us(h, us);
}

void esc_begin_arming(esc_handle_t* h, uint32_t duration_ms, uint32_t now_ms)
{
  if (!h) return;
  h->state = ESC_STATE_ARMING;
  h->arm_until_ms = now_ms + duration_ms;
  esc_write_us(h, h->cfg.min_us);
}

void esc_update(esc_handle_t* h, uint32_t now_ms)
{
  if (!h) return;
  if (h->state == ESC_STATE_ARMING && (int32_t)(now_ms - h->arm_until_ms) >= 0) {
    h->state = ESC_STATE_ARMED;
  }
}
