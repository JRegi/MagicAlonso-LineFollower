#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
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

/* ESC en PA8 / PA9 / PA10 (TIM1_CH1/2/3) */

/* ================== Flash ================== */

#define IR_FLASH_ADDR  0x0800FC00U  /* última página en un F103C8 (64KB) */
#define IR_MAGIC       0x51C051C0U  /* constante arbitraria válida */

typedef struct {
    uint32_t magic;        // IR_MAGIC si válido
    uint16_t sirc_code;    // 12 bits Sony SIRC
    uint16_t reserved;     // padding
    uint32_t checksum;     // magic ^ sirc_code
} ir_flash_t;

static ir_flash_t ir_flash_data;

/* ================== Estados ================== */

typedef enum {
    IR_STATE_LEARN = 0,  // esperando primer código para guardar
    IR_STATE_DONE        // ya tiene código, modo normal
} ir_state_t;

static ir_state_t ir_state = IR_STATE_LEARN;

/* ================== Sony SIRC decoder ================== */
/* Medimos tiempo entre flancos descendentes (delta_us).
 * - Si delta_us > ~10000us => nueva trama
 * - 600..1500 us  => bit 0
 * - 1500..3200 us => bit 1
 * 12 bits totales
 */

static volatile int      ir_bit_index    = -1;
static volatile uint16_t ir_code         = 0;
static volatile bool     ir_frame_ready  = false;
static volatile bool     ir_error        = false;

/* ================== Helpers ================== */

static void delay(volatile uint32_t t)
{
    while (t--) __asm__("nop");
}

/* LED helpers (activos en alto) */

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

static void led_green_on(void)
{
    gpio_set(LED_PORT, LED_G_PIN);
}

static void led_green_off(void)
{
    gpio_clear(LED_PORT, LED_G_PIN);
}

static void led_blue_on(void)
{
    gpio_set(LED_PORT, LED_B_PIN);
}

static void led_blue_off(void)
{
    gpio_clear(LED_PORT, LED_B_PIN);
}

/* ================== Clock 72 MHz ================== */

static void clock_setup(void)
{
    /* HSE 8MHz -> PLL -> 72MHz */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

/* ================== ESC: PWM seguro 400 Hz, 1000us ================== */

static void esc_pwm_safe_400hz_1000us(void)
{
    /* 1) Clocks */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM1);

    /* 2) GPIO: PA8/9/10 como alternate function push-pull */
    gpio_set_mode(GPIOA,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO8 | GPIO9 | GPIO10);

    /* 3) Timer1 a 1 MHz (1 us por tick), periodo 2500 us -> 400 Hz */
    timer_disable_counter(TIM1);

    /* 72 MHz / 72 = 1 MHz */
    timer_set_prescaler(TIM1, 72 - 1);

    /* Periodo: 2500 ticks -> 2500 us -> 400 Hz */
    timer_set_period(TIM1, 2500 - 1);

    /* 4) Configurar canales en PWM1 */
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

    timer_enable_oc_preload(TIM1, TIM_OC1);
    timer_enable_oc_preload(TIM1, TIM_OC2);
    timer_enable_oc_preload(TIM1, TIM_OC3);

    timer_set_oc_polarity_high(TIM1, TIM_OC1);
    timer_set_oc_polarity_high(TIM1, TIM_OC2);
    timer_set_oc_polarity_high(TIM1, TIM_OC3);

    /* 5) Pulsos a 1000 us (mínimo) */
    timer_set_oc_value(TIM1, TIM_OC1, 1000);
    timer_set_oc_value(TIM1, TIM_OC2, 1000);
    timer_set_oc_value(TIM1, TIM_OC3, 1000);

    /* 6) Habilitar salida canales */
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC2);
    timer_enable_oc_output(TIM1, TIM_OC3);

    /* 7) Habilitar salida principal de TIM1 (advanced timer) */
    timer_enable_break_main_output(TIM1);

    /* 8) Actualizar y arrancar */
    timer_generate_event(TIM1, TIM_EGR_UG);
    timer_enable_counter(TIM1);
}

/* ================== Init GPIO LED e IR ================== */

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

/* ================== Timer2 para tiempos IR (1us) ================== */

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

/* ================== EXTI PB7 ================== */

static void ir_exti_setup(void)
{
    rcc_periph_clock_enable(RCC_AFIO);

    exti_select_source(EXTI7, IR_PORT);
    exti_set_trigger(EXTI7, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI7);

    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

/* ================== Flash: load / save ================== */

static void ir_load_from_flash(void)
{
    const ir_flash_t *pf = (const ir_flash_t *)IR_FLASH_ADDR;
    ir_flash_data = *pf;

    uint32_t expected = ir_flash_data.magic ^ ir_flash_data.sirc_code;

    if (ir_flash_data.magic != IR_MAGIC || ir_flash_data.checksum != expected) {
        /* No hay datos válidos */
        ir_flash_data.magic     = 0;
        ir_flash_data.sirc_code = 0;
        ir_flash_data.reserved  = 0;
        ir_flash_data.checksum  = 0;
    }
}

static void ir_save_to_flash(uint16_t code)
{
    ir_flash_data.magic     = IR_MAGIC;
    ir_flash_data.sirc_code = code;
    ir_flash_data.reserved  = 0;
    ir_flash_data.checksum  = ir_flash_data.magic ^ ir_flash_data.sirc_code;

    flash_unlock();
    flash_erase_page(IR_FLASH_ADDR);

    uint32_t *src  = (uint32_t *)&ir_flash_data;
    uint32_t  addr = IR_FLASH_ADDR;

    for (unsigned i = 0; i < sizeof(ir_flash_data)/4; i++) {
        flash_program_word(addr, src[i]);
        addr += 4;
    }

    flash_lock();
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
    esc_pwm_safe_400hz_1000us();  /* Señal segura a los ESC */
    led_init();
    ir_gpio_setup();
    timer2_setup();
    ir_exti_setup();

    ir_load_from_flash();

    if (ir_flash_data.magic == IR_MAGIC) {
        /* Ya hay un código guardado en flash */
        ir_state = IR_STATE_DONE;

        /* Flash blanco rápido para decir "ya tenía código" */
        led_red_on();
        led_green_on();
        led_blue_on();
        delay(80000);
        led_all_off();
    } else {
        /* No hay código -> modo aprendizaje */
        ir_state = IR_STATE_LEARN;
        /* LED verde fijo: esperando primer código */
        led_green_on();
    }

    while (1) {
        if (ir_frame_ready) {
            ir_frame_ready = false;

            if (ir_error) {
                ir_error = false;
                continue;
            }

            uint16_t code = ir_code;  // 12 bits SIRC válidos

            if (ir_state == IR_STATE_LEARN) {
                /* Primer código válido que llega -> lo guardamos en Flash */
                ir_save_to_flash(code);

                /* Apagar verde, dar feedback y pasar a modo normal */
                led_green_off();

                /* Flash rojo breve: "grabado" */
                led_red_on();
                delay(80000);
                led_red_off();

                ir_state = IR_STATE_DONE;

            } else if (ir_state == IR_STATE_DONE) {
                /* Modo normal: si llega el mismo código, toggle rojo (para test) */
                if (ir_flash_data.magic == IR_MAGIC &&
                    code == ir_flash_data.sirc_code) {

                    led_red_toggle();
                }
            }
        }
    }

    return 0;
}
