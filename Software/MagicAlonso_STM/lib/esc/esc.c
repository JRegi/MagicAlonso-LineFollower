#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/dwt.h>
#include "esc.h"

/* -------- Helpers privados -------- */
static int is_adv_timer(uint32_t tim) { return (tim == TIM1) || (tim == TIM8); }

void delay_init_ms(void) {
    dwt_enable_cycle_counter();  // libopencm3 habilita DEMCR.TRCENA y DWT_CYCCNT
}

// Espera 'cycles' ciclos de CPU (maneja wrap-around de 32 bits).
static inline void _delay_cycles(uint32_t cycles) {
    const uint32_t start = DWT_CYCCNT;
    while ((uint32_t)(DWT_CYCCNT - start) < cycles) {
        __asm__("nop");
    }
}

// Delay en milisegundos (bloqueante).
void delay_ms(uint32_t ms) {
    // ticks por ms según el clock efectivo de AHB (72e6 => 72000 ticks/ms)
    const uint32_t ticks_per_ms = rcc_ahb_frequency / 1000U;

    // Para soportar delays largos sin overflow, lo hacemos en "chunks".
    while (ms) {
        // máx 1000 ms por chunk (=> <= ~72e6 ciclos, cabe en 32 bits con margen)
        uint32_t chunk = (ms > 1000U) ? 1000U : ms;
        _delay_cycles(ticks_per_ms * chunk);
        ms -= chunk;
    }
}

static void enable_rcc_gpio(uint32_t port) {
  if (port == GPIOA) rcc_periph_clock_enable(RCC_GPIOA);
  else if (port == GPIOB) rcc_periph_clock_enable(RCC_GPIOB);
  else if (port == GPIOC) rcc_periph_clock_enable(RCC_GPIOC);
  else if (port == GPIOD) rcc_periph_clock_enable(RCC_GPIOD);
  else if (port == GPIOE) rcc_periph_clock_enable(RCC_GPIOE);
}

static void enable_rcc_tim(uint32_t tim) {
  if (tim == TIM1) rcc_periph_clock_enable(RCC_TIM1);
  else if (tim == TIM2) rcc_periph_clock_enable(RCC_TIM2);
  else if (tim == TIM3) rcc_periph_clock_enable(RCC_TIM3);
  else if (tim == TIM4) rcc_periph_clock_enable(RCC_TIM4);
}

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

  /* Clocks */
  enable_rcc_gpio(cfg->gpio_port);
  enable_rcc_tim(cfg->tim);

  /* Pin AF push-pull (F1) */
  gpio_set_mode(cfg->gpio_port, GPIO_MODE_OUTPUT_10_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, cfg->gpio_pin);

  /* Timer: base 1 MHz (1 µs/tick), edge-aligned, up */
  timer_set_mode(cfg->tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(cfg->tim, 71);          // 72 MHz / (71+1) = 1 MHz
  timer_set_period(cfg->tim, h->arr);         // ARR = periodo_us - 1
  timer_enable_preload(cfg->tim);
  timer_continuous_mode(cfg->tim);

  /* Canal PWM */
  set_oc_mode(cfg->tim, cfg->ch);
  timer_set_oc_value(cfg->tim, cfg->ch, cfg->min_us);

  /* TIM1/TIM8 requieren habilitar MOE para sacar PWM */
  if (h->is_adv) { timer_enable_break_main_output(cfg->tim); }

  timer_generate_event(cfg->tim, TIM_EGR_UG); // aplica PSC/ARR/CCR
  timer_enable_counter(cfg->tim);
}

void esc_write_us(esc_handle_t* h, uint16_t us)
{
  if (!h) return;
  if (us < h->cfg.min_us) us = h->cfg.min_us;
  if (us > h->cfg.max_us) us = h->cfg.max_us;
  timer_set_oc_value(h->cfg.tim, h->cfg.ch, us);
}

void esc_calibrate(esc_handle_t* h)
{
  if (!h) return;

  delay_init_ms();

  esc_write_us(h, 0);
  delay_ms(200);

  esc_write_us(h, 1000);
  delay_ms(500);

  esc_write_us(h, 2000);
  delay_ms(5100);

  esc_write_us(h, 1000);
  delay_ms(4100);
}

void esc_arm(esc_handle_t* h)
{
  if (!h) return;

  delay_init_ms();

  // Min 500ms
  esc_write_us(h, h->cfg.min_us);
  delay_ms(500);

    // 1500us 500ms
  esc_write_us(h, 1500);
  delay_ms(500);

    // Min 500ms
  esc_write_us(h, h->cfg.min_us);
  delay_ms(500);
}