#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

/* OneShot125 típico: 125–250 us */
#define ONESHOT_MIN_US 125
#define ONESHOT_MAX_US 250

static void clock_setup(void)
{
    /* HSE 8 MHz → SYSCLK 72 MHz */
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM1);  /* PA8/PA9 = TIM1_CH1/CH2 */
}

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 7200; j++) __asm__("nop");
    }
}

/* TIM1_CH1 → PA8  y  TIM1_CH2 → PA9  | OneShot125 @ ~2 kHz */
static void pwm_setup_tim1_ch1_ch2_oneshot125(void)
{
    /* Pone PA8 y PA9 como Alternate Function Push-Pull 50 MHz */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8 | GPIO9);

    /* Base timer: 1 MHz (1 us por tick) */
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 71);      /* 72 MHz / (71+1) = 1 MHz */

    /* Periodo ~2 kHz. Usamos ARR=500 → 500 us (≈1996 Hz, más que suficiente) */
    timer_set_period(TIM1, 500);

    timer_enable_preload(TIM1);         /* ARPE */
    timer_continuous_mode(TIM1);        /* Free-running */

    /* CH1 (PA8) */
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_preload(TIM1, TIM_OC1);
    timer_set_oc_value(TIM1, TIM_OC1, ONESHOT_MIN_US);
    timer_enable_oc_output(TIM1, TIM_OC1);

    /* CH2 (PA9) */
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_preload(TIM1, TIM_OC2);
    timer_set_oc_value(TIM1, TIM_OC2, ONESHOT_MIN_US);
    timer_enable_oc_output(TIM1, TIM_OC2);

    /* TIM1 es “advanced”: habilitar salida principal (BDTR.MOE) */
    timer_enable_break_main_output(TIM1);

    /* Aplica preload y arranca */
    timer_generate_event(TIM1, TIM_EGR_UG);
    timer_enable_counter(TIM1);
}

/* Helpers de escritura (en microsegundos reales) */
static inline void esc_pa8_write_us(uint16_t us)
{
    if (us < ONESHOT_MIN_US) us = ONESHOT_MIN_US;
    if (us > ONESHOT_MAX_US) us = ONESHOT_MAX_US;
    timer_set_oc_value(TIM1, TIM_OC1, us);
}

static inline void esc_pa9_write_us(uint16_t us)
{
    if (us < ONESHOT_MIN_US) us = ONESHOT_MIN_US;
    if (us > ONESHOT_MAX_US) us = ONESHOT_MAX_US;
    timer_set_oc_value(TIM1, TIM_OC2, us);
}

static inline void esc_write_both_us(uint16_t us)
{
    if (us < ONESHOT_MIN_US) us = ONESHOT_MIN_US;
    if (us > ONESHOT_MAX_US) us = ONESHOT_MAX_US;
    timer_set_oc_value(TIM1, TIM_OC1, us); /* PA8 */
    timer_set_oc_value(TIM1, TIM_OC2, us); /* PA9 */
}

int main(void)
{
    clock_setup();
    pwm_setup_tim1_ch1_ch2_oneshot125();

    /* Armado típico: 2 s al mínimo en ambos ESC */
    for (int i = 0; i < 2000; i++) {
        esc_write_both_us(ONESHOT_MIN_US); /* 125 us */
        delay_ms(1);
    }

    /* Demo: rampa conjunta 140–220 us */
    while (1) {
        for (uint16_t u = 140; u <= 220; u += 2) {
            esc_write_both_us(u);
            delay_ms(10);
        }
        for (uint16_t u = 220; u >= 140; u -= 2) {
            esc_write_both_us(u);
            delay_ms(10);
        }
    }
}
