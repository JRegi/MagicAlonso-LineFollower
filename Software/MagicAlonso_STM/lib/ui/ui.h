#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Config (pines) ----------
   Ajustá si hace falta según TU cableado real.
   Por lo que vi en tu proyecto, los RGB van en PB12..PB14
   y el botón en PC13 (típico Bluepill).                    */

#include <libopencm3/stm32/gpio.h>

#ifndef RGB_PORT
#define RGB_PORT        GPIOB
#endif
#ifndef RGB_RED_PIN
#define RGB_RED_PIN     GPIO12  /* PB12 */
#endif
#ifndef RGB_GREEN_PIN
#define RGB_GREEN_PIN   GPIO13  /* PB13 */
#endif
#ifndef RGB_BLUE_PIN
#define RGB_BLUE_PIN    GPIO14  /* PB14 */
#endif

#ifndef BUTTON1_PORT
#define BUTTON1_PORT    GPIOC
#endif
#ifndef BUTTON1_PIN
#define BUTTON1_PIN     GPIO13  /* PC13, activo en 0 */
#endif

/* ---------- API ---------- */

/* Inicializa AFIO (deshabilita JTAG, deja SWD), GPIO de LED RGB y botón */
void ui_init(void);

/* LED RGB: helpers de colores “sintéticos” (compatibles con tu naming) */
void rgb_off(void);
void rgb_red(void);
void rgb_green(void);
void rgb_blue(void);
void rgb_yellow(void);
void rgb_magenta(void);
void rgb_cyan(void);
void rgb_white(void);

/* LED RGB: control fino por canal (true = encendido) */
void rgb_set(bool r, bool g, bool b);

/* Botón (lectura inmediata) */
bool button1_read(void);          /* true = presionado (nivel lógico invertido si PC13) */

/* Botón con flanco + debounce no bloqueante (recomendado) */
bool button1_was_pressed(uint32_t debounce_us); /* devuelve true UNA vez por pulsación */

#ifdef __cplusplus
}
#endif