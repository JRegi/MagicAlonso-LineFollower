#include "timing.h"
#include "clocks.h"

#include <stdbool.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#ifndef SYSTICK_HZ
#define SYSTICK_HZ 1000U
#endif

volatile bool control_tick_400hz = false;

static volatile uint32_t ticks_ms = 0;

void sys_tick_handler(void) { ticks_ms++; }

static inline void dwt_enable_if_possible(void) {
    #if defined(DWT_BASE)
        /* Habilita el contador de ciclos si existe DWT en este core */
        dwt_enable_cycle_counter();
    #endif
}

static uint32_t tim2_get_clk_hz(void) {
    if (rcc_apb1_frequency != rcc_ahb_frequency) {
        return rcc_apb1_frequency * 2u;   // típico: 36 MHz * 2 = 72 MHz
    } else {
        return rcc_apb1_frequency;        // sin prescaler, iguales
    }
}

void control_timer_init_400hz(void) {
    /* Secuencia de init sin timer_reset(): */
    timer_disable_counter(TIM2);

    nvic_enable_irq(NVIC_TIM2_IRQ);
    
    /* CR1: clock interno, edge-aligned, upcounting */
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    uint32_t tclk = tim2_get_clk_hz();     // ~72 MHz en F103 “bluepill” típico
    uint16_t psc  = (uint16_t)((tclk / 1000000UL) - 1UL); // -> base 1 MHz
    timer_set_prescaler(TIM2, psc);
    timer_set_period(TIM2, 2500 - 1);      // 1e6 / 400 = 2500 → 2.5 ms

    /* Asegurar contador en 0 y flags limpios */
    timer_set_counter(TIM2, 0);
    timer_clear_flag(TIM2, TIM_SR_UIF);

    timer_enable_irq(TIM2, TIM_DIER_UIE);  // Interrupción por update
    timer_generate_event(TIM2, TIM_EGR_UG);// Cargar PSC/ARR ya mismo
    timer_enable_counter(TIM2);
}

/* ISR: marcar el “tick” y salir rápido */
void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        timer_clear_flag(TIM2, TIM_SR_UIF);
        control_tick_400hz = true;
    }
}

void timing_init(void) {
    uint32_t cpu_hz = clocks_get_hz();

    /* SysTick: configurar reload para SYSTICK_HZ */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload((cpu_hz / SYSTICK_HZ) - 1U);
    systick_interrupt_enable();
    systick_counter_enable();

    /* DWT (alta resolución para micros) */
    dwt_enable_if_possible();
}

uint32_t millis(void) { return ticks_ms; }

uint32_t micros(void)
{
    uint32_t ms_before, cycles;
    uint32_t cpu_hz = clocks_get_hz();
    const uint32_t cycles_per_us = cpu_hz / 1000000U;
    const uint32_t cycles_per_ms = cpu_hz / SYSTICK_HZ;

    do {
        ms_before = ticks_ms;
        
        #if defined(DWT_CYCCNT)
            cycles = DWT_CYCCNT;
        #else
            cycles = 0;
        #endif
    } while (ms_before != ticks_ms);

    #if defined(DWT_CYCCNT)
        uint32_t cycles_in_ms = cycles % cycles_per_ms;
        uint32_t sub_us = cycles_in_ms / cycles_per_us;
        return ms_before * 1000U + sub_us;
    #else
        return ms_before * 1000U;
    #endif
}

void delay_ms_blocking(uint32_t ms)
{
    uint32_t start = micros();
    uint32_t wait = ms * 1000U;
    while ((uint32_t)(micros() - start) < wait) {
        __asm__("nop");
    }
}

void delay_us_blocking(uint32_t us)
{
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) {
        __asm__("nop");
    }
}

bool timeout_elapsed(uint32_t *last_us, uint32_t interval_us)
{
    uint32_t now = micros();
    if ((uint32_t)(now - *last_us) >= interval_us) {
        *last_us = now;
        return true;
    }
    return false;
}
