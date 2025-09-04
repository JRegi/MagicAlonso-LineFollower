#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

static void clock_setup(void)
{
    /* HSE 8 MHz -> SYSCLK 72 MHz */
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM1);   /* PA9 = TIM1_CH2 */
}

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 7200; j++) __asm__("nop");
    }
}

/* PWM servo clásico en PA9 (TIM1_CH2). freq_hz: 50 por defecto */
static void pwm_setup_pa9_tim1_ch2(uint16_t freq_hz)
{
    /* PA9 en Alternate Function Push-Pull 50 MHz */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

    /* Timer base: 1 MHz (1 us por tick) */
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 71); /* 72 MHz / (71+1) = 1 MHz */

    /* Periodo en microsegundos → 50 Hz => 20000 us */
    uint32_t period_us = 1000000UL / freq_hz; /* p.ej. 20000 para 50 Hz */
    timer_set_period(TIM1, period_us);

    timer_enable_preload(TIM1);  /* ARPE */
    timer_continuous_mode(TIM1);

    /* Canal 2 (PA9) en PWM1 */
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_preload(TIM1, TIM_OC2);

    /* Arranque seguro al mínimo (1000 us) */
    timer_set_oc_value(TIM1, TIM_OC2, ESC_MIN_US);
    timer_enable_oc_output(TIM1, TIM_OC2);

    /* TIM1 es “advanced”: habilitar salida principal (BDTR.MOE) */
    timer_enable_break_main_output(TIM1);

    /* Aplicar preloads y arrancar */
    timer_generate_event(TIM1, TIM_EGR_UG);
    timer_enable_counter(TIM1);
}

static void esc_pa9_write_us(uint16_t us)
{
    if (us < ESC_MIN_US) us = ESC_MIN_US;
    if (us > ESC_MAX_US) us = ESC_MAX_US;
    timer_set_oc_value(TIM1, TIM_OC2, us); /* Duty en microsegundos reales */
}

int main(void)
{
    clock_setup();
    pwm_setup_pa9_tim1_ch2(50); /* 50 Hz como en Fujitora1 */

    /* Armado típico del ESC: ~2 s al mínimo */
    for (int i = 0; i < 2000; i++) {
        esc_pa9_write_us(ESC_MIN_US); /* 1000 us */
        delay_ms(1);
    }

    /* Demo: rampa suave 1100–1500 us */
    while (1) {
        for (uint16_t u = 1100; u <= 1500; u += 5) {
            esc_pa9_write_us(u);
            delay_ms(10);
        }
        for (uint16_t u = 1500; u >= 1100; u -= 5) {
            esc_pa9_write_us(u);
            delay_ms(10);
        }
    }
}