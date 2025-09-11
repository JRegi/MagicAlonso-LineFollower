#include "qre_array.h"
#include "esc.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <stdint.h>
#include <stdbool.h>

#define CTRL_HZ      400                  // 400 Hz → PWM “servo”
#define DT_SEC       (1.0f / CTRL_HZ)

// Centro para N=8 (0..7000)
#define SETPOINT     3500

// Ahora en μs de servo:
#define BASE_SPEED   1075                // neutral
#define MAX_SPEED    1150                 // tope alto seguro
#define MIN_SPEED    1000                 // tope bajo seguro
#define PWM_HZ       400u                 // servo @ 400 Hz

#define MOTOR_LEFT_PIN   GPIO10           // TIM1_CH3 (PA10 si corresponde a tu mapeo)
#define MOTOR_RIGHT_PIN  GPIO8            // TIM1_CH1 (PA8)

#define TIM_LEFT_MOTOR   TIM_OC3
#define TIM_RIGHT_MOTOR  TIM_OC1

// Ganancias “por muestra” (dt=2.5 ms). Punto de arranque conservador.
static float KP = 0.008f;
static float KD = 0.0f;
static int   last_error = 0;

// (dejado como lo tenías)
static const uint8_t QRE_CH[8] = {7, 6, 5, 4, 3, 2, 0, 1};
qre_array_t qre;

static volatile uint32_t ticks = 0;

esc_handle_t ml, mr;

const esc_config_t escL = {
    .tim       = TIM1,
    .ch        = TIM_LEFT_MOTOR,
    .gpio_port = GPIOA,
    .gpio_pin  = MOTOR_LEFT_PIN,
    .freq_hz   = PWM_HZ,        // 400 Hz
    .min_us    = MIN_SPEED,     // 1100
    .max_us    = MAX_SPEED      // 1900
};
const esc_config_t escR = {
    .tim       = TIM1,
    .ch        = TIM_RIGHT_MOTOR,
    .gpio_port = GPIOA,
    .gpio_pin  = MOTOR_RIGHT_PIN,
    .freq_hz   = PWM_HZ,        // 400 Hz
    .min_us    = MIN_SPEED,     // 1100
    .max_us    = MAX_SPEED      // 1900 (arreglado)
};

void sys_tick_handler(void);
void sys_tick_handler(void) { ticks++; }

static void clock_and_systick_setup(void) {
    // 72 MHz desde HSE 8 MHz
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    // SysTick a 400 Hz → 2.5 ms
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload((72000000 / CTRL_HZ) - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

static inline void pid_step_and_output(uint16_t position) {
    int error      = (int)position - SETPOINT;
    int derivative = error - last_error;

    float pid = (error * KP) + (derivative * KD);
    last_error = error;

    float us_right = (float)BASE_SPEED + pid;
    float us_left  = (float)BASE_SPEED - pid;

    if (us_right > MAX_SPEED) us_right = MAX_SPEED;
    else if (us_right < MIN_SPEED) us_right = MIN_SPEED;
    if (us_left  > MAX_SPEED) us_left  = MAX_SPEED;
    else if (us_left  < MIN_SPEED) us_left  = MIN_SPEED;

    esc_write_us(&mr, (uint16_t)us_right);
    esc_write_us(&ml, (uint16_t)us_left);
}

int main(void) {
    clock_and_systick_setup();

    esc_init(&ml, &escL);
    esc_init(&mr, &escR);

    esc_arm(&ml);
    esc_arm(&mr);

    // Sensores
    qre_init(&qre, QRE_CH, 8);

    // Calibración (mover la regleta por línea y fondo)
    qre_calibrate(&qre, 2000, 1000);

    // Promedio (arrancá con 1 si querés tunear KP primero)
    qre_set_averaging(&qre, 4);

    uint32_t next = ticks; // arranca ya
    while (1) {
        // esperar el próximo tick exacto (2.5 ms)
        while ((int32_t)(ticks - next) < 0) { __asm__("nop"); }
        next += 1;

        // 1) Lectura en fase
        uint16_t pos = qre_read_position_black(&qre);

        // 2) PID + salida
        pid_step_and_output(pos);
    }
}