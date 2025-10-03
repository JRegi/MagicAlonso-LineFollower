#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>        // API genérica del NVIC
#include <libopencm3/stm32/f1/nvic.h> 
#include <libopencm3/stm32/timer.h>
#include "receptor_ir.h"

#define IR_PORT GPIOA
#define IR_PIN  GPIO0

static volatile uint32_t last_time = 0;
static volatile uint32_t code = 0;
static volatile int bitcount = 0;
static volatile int available = 0;

void ir_init(void) {
    // Clocks
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_TIM2);

    // Reset corto del TIM2 y configuración
    rcc_periph_reset_pulse(RST_TIM2);

    // PA0 como entrada con pull-up
    gpio_set_mode(IR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, IR_PIN);
    gpio_set(IR_PORT, IR_PIN);

    // TIM2 a 1 MHz (1 tick = 1 us) y contador a 0
    timer_disable_counter(TIM2);
    timer_set_prescaler(TIM2, (rcc_apb1_frequency/1000000) - 1);
    timer_set_period(TIM2, 0xFFFF);
    timer_set_counter(TIM2, 0);
    timer_enable_counter(TIM2);

    // EXTI0 en PA0 flanco de bajada
    exti_select_source(EXTI0, IR_PORT);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI0);

    // NVIC
    nvic_enable_irq(NVIC_EXTI0_IRQ);
}

void exti0_isr(void) {
    uint32_t now = timer_get_counter(TIM2);
    uint32_t delta = now - last_time;
    last_time = now;

    if(delta > 8500 && delta < 9500) {   // start pulse ~9ms
        code = 0;
        bitcount = 0;
    } else if(delta > 4000 && delta < 5000) {
        // 4.5ms space, espera bits
        bitcount = 0;
        code = 0;
    } else if(delta > 400 && delta < 700) {
        // possible bit 0 (560us)
    } else if(delta > 1500 && delta < 1800) {
        // bit 1 (1690us)
        code |= (1UL << bitcount);
    }

    if(bitcount++ >= 32) {
        available = 1;
        bitcount = 0;
    }

    exti_reset_request(EXTI0);
}

int ir_available(void) {
    return available;
}

uint32_t ir_read(void) {
    available = 0;
    return code;
}

