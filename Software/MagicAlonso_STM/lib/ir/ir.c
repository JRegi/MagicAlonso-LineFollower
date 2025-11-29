#include "ir.h"
#include "timing.h"      // micros()
#include "clocks.h"      // clocks_enable_periph() si quisieras usarla

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>

/* ================== Configuración HW ================== */

#define IR_PORT        GPIOB
#define IR_PIN         GPIO7

/* F103C8: última página de Flash en 0x0800FC00 (64 kB) */
#define IR_FLASH_ADDR  0x0800FC00U
#define IR_MAGIC       0x51C051C0U   /* valor mágico arbitrario válido */

typedef struct {
    uint32_t magic;        /* IR_MAGIC si es válido */
    uint16_t sirc_code;    /* 12 bits Sony SIRC */
    uint16_t reserved;     /* padding (sin usar) */
    uint32_t checksum;     /* magic ^ sirc_code */
} ir_flash_t;

static ir_flash_t ir_flash_data;

/* ================== Estado de alto nivel ================== */

typedef enum {
    IR_STATE_LEARN = 0,   /* esperando primer código para guardar */
    IR_STATE_DONE         /* ya hay código en Flash, modo normal */
} ir_state_t;

static ir_state_t ir_state = IR_STATE_LEARN;

/* ================== Decoder Sony SIRC (12 bits) ==================
 *
 * Usamos micros() para medir delta_us entre flancos descendentes:
 *
 *  - Si delta_us > ~10000us => asumimos que empieza una nueva trama.
 *  - Bits:
 *      600..1500 us  => bit 0   (~1.2 ms)
 *      1500..3200 us => bit 1   (~2.4 ms)
 *
 * Se acumulan exactamente 12 bits por trama (SIRC clásico).
 * ================================================================ */

static volatile int      ir_bit_index   = -1;    /* -1 = no estamos en trama */
static volatile uint16_t ir_code        = 0;     /* acumulador de 12 bits */
static volatile bool     ir_frame_ready = false;
static volatile bool     ir_error       = false;

/* Marca de tiempo del último flanco (en us, vía micros()) */
static volatile uint32_t ir_last_edge_us = 0;

/* ================== Helpers internos ================== */

static void ir_delay(volatile uint32_t t)
{
    while (t--) __asm__("nop");
}

static void ir_load_from_flash(void)
{
    const ir_flash_t *pf = (const ir_flash_t *)IR_FLASH_ADDR;
    ir_flash_data = *pf;

    uint32_t expected = ir_flash_data.magic ^ ir_flash_data.sirc_code;

    if (ir_flash_data.magic != IR_MAGIC || ir_flash_data.checksum != expected) {
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
    uint32_t addr  = IR_FLASH_ADDR;

    for (unsigned i = 0; i < sizeof(ir_flash_data)/4; i++) {
        flash_program_word(addr, src[i]);
        addr += 4;
    }

    flash_lock();
}

/* ================== Init hardware ================== */

static void ir_gpio_setup(void)
{
    /* clocks_init() ya habilita GPIOB y AFIO, así que en principio
       no haría falta clocks_enable_periph(), pero es seguro llamarla. */

    // clocks_enable_periph(RCC_GPIOB);  // opcional
    gpio_set(IR_PORT, IR_PIN);  /* PULL-UP interno: ODR=1 */
    gpio_set_mode(IR_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN,
                  IR_PIN);
}

static void ir_exti_setup(void)
{
    // clocks_enable_periph(RCC_AFIO);   // clocks_init() ya lo hace

    exti_select_source(EXTI7, IR_PORT);
    exti_set_trigger(EXTI7, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI7);

    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
    /* Si quisieras, acá podrías fijar prioridad explícita:
       nvic_set_priority(NVIC_EXTI9_5_IRQ, 64);
    */
}

/* ================== API pública ================== */

void ir_sirc_init(void)
{
    ir_gpio_setup();
    ir_exti_setup();

    ir_load_from_flash();

    if (ir_flash_data.magic == IR_MAGIC) {
        ir_state = IR_STATE_DONE;
    } else {
        ir_state = IR_STATE_LEARN;
    }

    ir_bit_index    = -1;
    ir_code         = 0;
    ir_frame_ready  = false;
    ir_error        = false;
    ir_last_edge_us = micros();   /* punto de partida */
}

bool ir_sirc_has_saved_code(void)
{
    return (ir_flash_data.magic == IR_MAGIC);
}

uint16_t ir_sirc_get_saved_code(void)
{
    return ir_flash_data.sirc_code;
}

void ir_sirc_clear_flash_and_reset(void)
{
    flash_unlock();
    flash_erase_page(IR_FLASH_ADDR);
    flash_lock();

    ir_flash_data.magic     = 0;
    ir_flash_data.sirc_code = 0;
    ir_flash_data.reserved  = 0;
    ir_flash_data.checksum  = 0;

    ir_state = IR_STATE_LEARN;
}

/* Servicio no bloqueante: se llama desde el main loop */
bool ir_sirc_service(bool *learned_now, bool *match_saved, uint16_t *last_code)
{
    if (learned_now) *learned_now = false;
    if (match_saved) *match_saved = false;

    if (!ir_frame_ready) {
        return false;  /* nada que hacer */
    }

    ir_frame_ready = false;

    if (ir_error) {
        ir_error = false;
        return true;   /* hubo frame, pero inválido */
    }

    uint16_t code = ir_code;  /* 12 bits válidos */

    if (last_code) {
        *last_code = code;
    }

    if (ir_state == IR_STATE_LEARN) {
        /* Primer código válido que llega -> guardar en Flash */
        ir_save_to_flash(code);
        ir_state = IR_STATE_DONE;

        if (learned_now) {
            *learned_now = true;
        }

    } else if (ir_state == IR_STATE_DONE) {
        if (ir_flash_data.magic == IR_MAGIC &&
            code == ir_flash_data.sirc_code) {

            if (match_saved) {
                *match_saved = true;
            }
        }
    }

    return true;
}

/* ================== ISR EXTI: Sony SIRC ================== */

void exti9_5_isr(void)
{
    if (!exti_get_flag_status(EXTI7)) return;
    exti_reset_request(EXTI7);

    uint32_t now_us   = micros();
    uint32_t delta_us = now_us - ir_last_edge_us;
    ir_last_edge_us   = now_us;

    /* GAP largo entre tramas => reset y empezar nueva */
    if (delta_us > 10000U) {   /* >10 ms */
        ir_bit_index   = 0;
        ir_code        = 0;
        ir_frame_ready = false;
        ir_error       = false;
        return;
    }

    if (ir_bit_index < 0 || ir_bit_index >= 12) {
        /* Sin gap de inicio, o ya superamos 12 bits */
        return;
    }

    /* Decodificar bit según delta_us:
     *  - ~1.2ms => 0
     *  - ~2.4ms => 1
     */
    if (delta_us > 600U && delta_us < 1500U) {
        /* bit 0 */
        ir_code |= (0U << ir_bit_index);
        ir_bit_index++;
    } else if (delta_us > 1500U && delta_us < 3200U) {
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
