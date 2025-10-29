#include "ui.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "timing.h"   /* para micros() en el debounce */

static inline void rgb_write(bool r_on, bool g_on, bool b_on)
{
    if (r_on) gpio_set(RGB_PORT, RGB_RED_PIN);   else gpio_clear(RGB_PORT, RGB_RED_PIN);
    if (g_on) gpio_set(RGB_PORT, RGB_GREEN_PIN); else gpio_clear(RGB_PORT, RGB_GREEN_PIN);
    if (b_on) gpio_set(RGB_PORT, RGB_BLUE_PIN);  else gpio_clear(RGB_PORT, RGB_BLUE_PIN);
}

void ui_init(void)
{
    /* Liberar PB3/PB4/PB5 si hizo falta (JTAG off, SWD on) */
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

    /* RGB: PB12..PB14 como push-pull */
    gpio_set_mode(RGB_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  RGB_RED_PIN | RGB_GREEN_PIN | RGB_BLUE_PIN);
    rgb_off();

    /* Botón: PC13 input pull-up (típico Bluepill) */
    gpio_set_mode(BUTTON1_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN, BUTTON1_PIN);
    gpio_set(BUTTON1_PORT, BUTTON1_PIN); /* pull-up */
}

void rgb_set(bool r, bool g, bool b)     { rgb_write(r, g, b); }
void rgb_off(void)                       { rgb_write(false, false, false); }
void rgb_red(void)                       { rgb_write(false , true, false); }
void rgb_green(void)                     { rgb_write(true, false, false); }
void rgb_blue(void)                      { rgb_write(false, false, true ); }
void rgb_yellow(void)                    { rgb_write(true , true , false); }
void rgb_magenta(void)                   { rgb_write(true , false, true ); }
void rgb_cyan(void)                      { rgb_write(false, true , true ); }
void rgb_white(void)                     { rgb_write(true , true , true ); }

bool button1_read(void)
{
    /* PC13 en Bluepill es activo en 0 (a masa al presionar). */
    return !gpio_get(BUTTON1_PORT, BUTTON1_PIN);
}

bool button1_was_pressed(uint32_t debounce_us)
{
    static bool pressed_last = false;
    static uint32_t t_last = 0;

    bool now = button1_read();

    if (now != pressed_last) {
        /* cambió estado → arrancar/chequear debounce */
        uint32_t t = micros();
        if ((uint32_t)(t - t_last) >= debounce_us) {
            /* estado estable por > debounce → actualizar */
            pressed_last = now;
            t_last = t;

            /* flanco de bajada a subida (o subida a bajada).
               Queremos “una vez por pulsación”: retornar true
               cuando pasa a PRESIONADO. */
            if (now) return true;
        }
    }
    return false;
}
