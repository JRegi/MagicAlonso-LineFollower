#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include "esc/esc.h"

static void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++)
    for (volatile uint32_t j = 0; j < 7200; j++) __asm__("nop");
}

/* ---- Tick en ms con SysTick ---- */
static volatile uint32_t sys_ms = 0;
static inline uint32_t millis(void) { return sys_ms; }
void sys_tick_handler(void) { sys_ms++; }
static void systick_setup_1ms(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);   // 72 MHz
    systick_set_reload(72000 - 1);                    // 1 kHz -> 1 ms
    systick_interrupt_enable();
    systick_counter_enable();
}

/* ---- LED PC13 (opcional para errores) ---- */
static void led_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}
static void led_blink_blocking(uint32_t ms) {
    for (volatile uint32_t i=0; i<ms*7200; ++i) __asm__("nop");
    gpio_toggle(GPIOC, GPIO13);
}

int main(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    systick_setup_1ms();
    led_setup();

    esc_handle_t ml, mr;

    // Izquierdo: PA10 = TIM1_CH3
    esc_config_t cfgl = {
        .tim      = TIM1,
        .ch       = TIM_OC3,
        .gpio_port= GPIOA,
        .gpio_pin = GPIO10,
        .freq_hz  = 50,
        .min_us   = 1000,
        .max_us   = 2000
    };

    // Derecho: PA8 = TIM1_CH1
    esc_config_t cfgr = {
        .tim      = TIM1,
        .ch       = TIM_OC1,
        .gpio_port= GPIOA,
        .gpio_pin = GPIO8,
        .freq_hz  = 50,
        .min_us   = 1000,
        .max_us   = 2000
    };

    if (esc_init(&ml, &cfgl) != ESC_OK) {
        timer_disable_oc_output(cfgl.tim, cfgl.ch);
        if (cfgl.tim == TIM1 || cfgl.tim == TIM8) {
            timer_disable_break_main_output(cfgl.tim);
        }
        timer_disable_counter(cfgl.tim);
        gpio_set_mode(cfgl.gpio_port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, cfgl.gpio_pin);
        while (1) { led_blink_blocking(200); }
    }

    if (esc_init(&mr, &cfgr) != ESC_OK) {
        timer_disable_oc_output(cfgr.tim, cfgr.ch);
        if (cfgr.tim == TIM1 || cfgr.tim == TIM8) {
            timer_disable_break_main_output(cfgr.tim);
        }
        timer_disable_counter(cfgr.tim);
        gpio_set_mode(cfgr.gpio_port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, cfgr.gpio_pin);
        while (1) { led_blink_blocking(200); }
    }

    // Arming no bloqueante (~2 s a mínimo)
    esc_begin_arming(&ml, 2000, millis());
    esc_begin_arming(&mr, 2000, millis());

    while (1) {
        uint32_t now = millis();
        esc_update(&ml, now);
        esc_update(&mr, now);

        // Mantener ambos al mínimo hasta que los dos estén ARMADOS
        if (esc_state(&ml) != ESC_STATE_ARMED || esc_state(&mr) != ESC_STATE_ARMED) {
            esc_write_us(&ml, cfgl.min_us);
            esc_write_us(&mr, cfgr.min_us);
            continue;
        }

        // Rampa ML
        for (uint16_t u = 1100; u <= 1500; u += 20) {
            esc_write_us(&ml, u);
            delay_ms(10);
        }
        esc_write_us(&ml, cfgl.min_us);

        // Rampa MR
        for (uint16_t u = 1100; u <= 1500; u += 20) {
            esc_write_us(&mr, u);
            delay_ms(10);
        }
        esc_write_us(&mr, cfgr.min_us);

        // Rampa conjunta
        for (uint16_t u = 1100; u <= 1500; u += 20) {
            esc_write_us(&ml, u);
            esc_write_us(&mr, u);
            delay_ms(10);
        }
        esc_write_us(&ml, cfgl.min_us);
        esc_write_us(&mr, cfgr.min_us);
    }
}
