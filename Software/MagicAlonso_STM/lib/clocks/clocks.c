#include "clocks.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

void clocks_init(void)
{
    /* Configura reloj HSE 8MHz -> PLL -> 72MHz (STM32F1 classic) */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    /* Habilitar solo los periféricos detectados en el proyecto */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    
    rcc_periph_clock_enable(RCC_AFIO);

    /* Timers que aparecen en tu código: TIM1 y TIM2 */
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_TIM2);

    /* ADC1 (usado por regleta de sensores QRE) */
    rcc_periph_clock_enable(RCC_ADC1);

    /* USART3 (telemetría/console en tu proyecto) */
    rcc_periph_clock_enable(RCC_USART3);

    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

    /* Configurar prioridades NVIC por defecto */
    clocks_default_nvic_priorities();
}

void clocks_enable_periph(uint32_t rcc_periph)
{
    rcc_periph_clock_enable(rcc_periph);
}

uint32_t clocks_get_hz(void)
{
    return rcc_ahb_frequency; /* libopencm3 mantiene esta variable */
}

void clocks_default_nvic_priorities(void)
{
    /* SysTick: prioridad media */
    #ifdef NVIC_SYSTICK
        nvic_set_priority(NVIC_SYSTICK, 64);
    #elif defined(NVIC_SYSTICK_IRQ)
        nvic_set_priority(NVIC_SYSTICK_IRQ, 64);
    #else
        #warning "SysTick NVIC identifier not defined for this target."
    #endif

    /* Timers críticos: TIM1 (PWM) y TIM2 (loop de control) */
    #ifdef NVIC_TIM1_UP_IRQ
        nvic_set_priority(NVIC_TIM1_UP_IRQ, 32);
    #endif

    #ifdef NVIC_TIM2_IRQ
        nvic_set_priority(NVIC_TIM2_IRQ, 48);
    #endif

    /* USART3: baja prioridad (telemetría no crítica) */
    #ifdef NVIC_USART3_IRQ
        nvic_set_priority(NVIC_USART3_IRQ, 96);
    #endif
}