#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <stdint.h>
#include <stdbool.h>

/* ================== Pines ================== */

#define IR_PORT        GPIOB
#define IR_PIN         GPIO7

#define LED_PORT       GPIOB
#define LED_R_PIN      GPIO15   // rojo
#define LED_G_PIN      GPIO14   // verde
#define LED_B_PIN      GPIO13   // azul

/* ================== Estados ================== */

typedef enum {
    IR_LEARN_FIRST = 0,   // aprendiendo botón 1
    IR_LEARN_SECOND,      // aprendiendo botón 2
    IR_LEARN_DONE         // modo normal
} ir_learn_state_t;

static ir_learn_state_t learn_state = IR_LEARN_FIRST;

/* Códigos aprendidos (Sony SIRC 12 bits) */
static uint16_t code_btn1 = 0;
static uint16_t code_btn2 = 0;
static bool     btn1_learned = false;
static bool     btn2_learned = false;

/* ================== Decodificación Sony SIRC ==================
 * Medimos tiempo entre flancos descendentes (delta_us).
 *
 * - Si delta_us > ~10000us => asumimos que empieza una nueva trama.
 * - Bits:
 *      ~1.2ms (ej. 600..1500us)  => bit 0
 *      ~2.4ms (ej. 1500..3200us) => bit 1
 *
 * Recibimos 12 bits (SIRC clásico).
 * ============================================================= */

static volatile int      ir_bit_index    = -1;   // -1 = no estamos en trama
static volatile uint16_t ir_code         = 0;    // máx 16 bits, usamos 12
static volatile bool     ir_frame_ready  = false;
static volatile bool     ir_error        = false;

/* ================== Helpers ================== */

static void delay(volatile uint32_t t)
{
    while (t--) __asm__("nop");
}

/* LED helpers (activos en alto, independientes por color) */

static void led_all_off(void)
{
    gpio_clear(LED_PORT, LED_R_PIN | LED_G_PIN | LED_B_PIN);
}

static void led_red_on(void)
{
    gpio_set(LED_PORT, LED_R_PIN);
}

static void led_red_off(void)
{
    gpio_clear(LED_PORT, LED_R_PIN);
}

static void led_red_toggle(void)
{
    if (gpio_get(LED_PORT, LED_R_PIN)) {
        led_red_off();
    } else {
        led_red_on();
    }
}

static void led_blue_on(void)
{
    gpio_set(LED_PORT, LED_B_PIN);
}

static void led_blue_off(void)
{
    gpio_clear(LED_PORT, LED_B_PIN);
}

static void led_blue_toggle(void)
{
    if (gpio_get(LED_PORT, LED_B_PIN)) {
        led_blue_off();
    } else {
        led_blue_on();
    }
}

static void led_green_on(void)
{
    gpio_set(LED_PORT, LED_G_PIN);
}

static void led_green_off(void)
{
    gpio_clear(LED_PORT, LED_G_PIN);
}

/* ================== Init ================== */

static void clock_setup(void)
{
    /* HSE 8MHz -> PLL -> 72MHz */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

static void led_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_set_mode(LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  LED_R_PIN | LED_G_PIN | LED_B_PIN);

    led_all_off();
}

static void ir_gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);

    /* PB7 como entrada con pull-up interno */
    gpio_set(IR_PORT, IR_PIN);  // ODR=1 => PULL-UP
    gpio_set_mode(IR_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN,
                  IR_PIN);
}

static void timer2_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM2);

    timer_disable_counter(TIM2);
    /* 72MHz / 72 = 1MHz => 1us por tick */
    timer_set_prescaler(TIM2, 72 - 1);
    timer_set_period(TIM2, 0xFFFF);
    timer_set_counter(TIM2, 0);
    timer_enable_counter(TIM2);
}

static void ir_exti_setup(void)
{
    rcc_periph_clock_enable(RCC_AFIO);

    exti_select_source(EXTI7, IR_PORT);
    exti_set_trigger(EXTI7, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI7);

    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

/* ================== ISR EXTI: Sony SIRC ================== */

void exti9_5_isr(void)
{
    if (!exti_get_flag_status(EXTI7)) return;
    exti_reset_request(EXTI7);

    uint16_t delta_us = timer_get_counter(TIM2);
    timer_set_counter(TIM2, 0);

    /* GAP largo entre tramas => reset y empezar nueva */
    if (delta_us > 10000) {   // >10ms
        ir_bit_index   = 0;
        ir_code        = 0;
        ir_frame_ready = false;
        ir_error       = false;
        return;
    }

    if (ir_bit_index < 0 || ir_bit_index >= 12) {
        /* Todavía no vimos un gap largo para empezar trama,
           o ya pasamos 12 bits. Ignorar. */
        return;
    }

    /* Decodificar bit según delta_us
     *  - ~1.2ms => 0
     *  - ~2.4ms => 1
     */
    if (delta_us > 600 && delta_us < 1500) {
        /* bit 0 */
        ir_code |= (0U << ir_bit_index);
        ir_bit_index++;
    } else if (delta_us > 1500 && delta_us < 3200) {
        /* bit 1 */
        ir_code |= (1U << ir_bit_index);
        ir_bit_index++;
    } else {
        /* timing raro => error */
        ir_error     = true;
        ir_bit_index = -1;
        return;
    }

    if (ir_bit_index == 12) {
        /* recibimos 12 bits */
        ir_frame_ready = true;
        ir_bit_index   = -1;
    }
}

/* ================== main ================== */

int main(void)
{
    clock_setup();
    led_init();
    ir_gpio_setup();
    timer2_setup();
    ir_exti_setup();

    /* Al inicio: aprender botón 1 -> LED verde encendido */
    learn_state   = IR_LEARN_FIRST;
    btn1_learned  = false;
    btn2_learned  = false;
    led_all_off();
    led_green_on();

    while (1) {
        if (ir_frame_ready) {
            ir_frame_ready = false;

            if (ir_error) {
                ir_error = false;
                continue;
            }

            uint16_t code = ir_code;   // 12 bits válidos

            /* ======== MODO APRENDIZAJE ======== */
            if (learn_state == IR_LEARN_FIRST) {
                /* Primer botón */
                code_btn1     = code;
                btn1_learned  = true;

                /* Apagar verde, pequeña pausa y volver a encenderlo
                   para indicar que ahora espera el segundo botón */
                led_green_off();
                delay(60000);
                led_green_on();

                learn_state = IR_LEARN_SECOND;

            } else if (learn_state == IR_LEARN_SECOND) {
                /* Segundo botón */
                code_btn2     = code;
                btn2_learned  = true;

                /* Ya no estamos aprendiendo: apagar verde */
                led_green_off();
                learn_state = IR_LEARN_DONE;

                /* Pequeño flash rojo+azul al terminar el aprendizaje */
                led_red_on();
                led_blue_on();
                delay(80000);
                led_red_off();
                led_blue_off();

            /* ======== MODO NORMAL ======== */
            } else if (learn_state == IR_LEARN_DONE) {
                if (btn1_learned && code == code_btn1) {
                    /* Botón 1 -> toggle rojo */
                    led_red_toggle();
                } else if (btn2_learned && code == code_btn2) {
                    /* Botón 2 -> toggle azul */
                    led_blue_toggle();
                }
            }
        }
    }

    return 0;
}
