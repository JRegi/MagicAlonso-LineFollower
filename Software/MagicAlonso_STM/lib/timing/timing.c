#include "timing.h"
#include "clocks.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>

#ifndef SYSTICK_HZ
#define SYSTICK_HZ 1000U
#endif

static volatile uint32_t ticks_ms = 0;

void sys_tick_handler(void) { ticks_ms++; }

static inline void dwt_enable_if_possible(void) {
    #if defined(DWT_BASE)
        /* Habilita el contador de ciclos si existe DWT en este core */
        dwt_enable_cycle_counter();
    #endif
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
