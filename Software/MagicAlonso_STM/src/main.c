#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include "esc.h"
#include "qtr_array.h"

#define MOTOR_LEFT_PIN  GPIO10   // TIM1_CH3
#define MOTOR_RIGHT_PIN GPIO8  // TIM1_CH1

#define TIM_LEFT_MOTOR  TIM_OC3
#define TIM_RIGHT_MOTOR TIM_OC1

#define US_MIN          1000u
#define US_MAX          1350u

#define PWM_HZ          400u

#define NUM_SENS        8
#define SETPOINT        0

void sys_tick_handler(void);
static volatile uint32_t _ms = 0;
void sys_tick_handler(void){ _ms++; }
static inline uint32_t millis(void){ return _ms; }

static void delay_us(uint32_t us){ for (volatile uint32_t i=0;i<us*12;++i) __asm__("nop"); }
static void delay_ms(uint32_t ms){ while(ms--) delay_us(1000); }

static void clock_setup(void){
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_CAL_PIN | LED_RUN_PIN);
    LED_OFF(LED_CAL_PORT, LED_CAL_PIN);
    LED_OFF(LED_RUN_PORT, LED_RUN_PIN);
}

static void systick_setup(void){
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(72000 - 1); // 1 ms
    systick_interrupt_enable();
    systick_counter_enable();
}

static void arm_esc(uint16_t delay){
    esc_write_us(&ml, US_MIN);
    esc_write_us(&mr, US_MIN);
    delay_ms(delay);

    esc_write_us(&ml, US_MAX);
    esc_write_us(&mr, US_MAX);
    delay_ms(delay);

    esc_write_us(&ml, US_MIN);
    esc_write_us(&mr, US_MIN);
    delay_ms(delay);
}

void main(void) {
    clock_setup();
    systick_setup();

    esc_handle_t escL, mr;
    
    const esc_config_t ml = {
        .tim=TIM1,
        .ch=TIM_LEFT_MOTOR,
        .gpio_port=GPIOA,
        .gpio_pin=MOTOR_LEFT_PIN,
        .freq_hz=PWM_HZ,
        .min_us=US_MIN,
        .max_us=US_MAX
    };
    const esc_config_t cfgR = {.tim=TIM1,
        .ch=TIM_RIGHT_MOTOR,
        .gpio_port=GPIOA,
        .gpio_pin=MOTOR_RIGHT_PIN,
        .freq_hz=PWM_HZ,
        .min_us=US_MIN,
        .max_us=US_MAX
    };

    esc_init(&escL, &ml);
    esc_init(&escR, &mr);

    arm_esc(1000);

    while (1) {
        esc_write_us(&ml, US_MAX);
        delay(2000);
        
        esc_write_us(&ml, US_MIN);
        delay(2000);

        esc_write_us(&mr, US_MAX);
        delay(2000);

        esc_write_us(&mr, US_MIN);
        delay(4000);
    }
}



