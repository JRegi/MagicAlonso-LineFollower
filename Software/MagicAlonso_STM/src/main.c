#include "qre_array.h"
#include "esc.h"
#include "uart.h"
#include "timing.h"
#include "ui.h"
#include "clocks.h"
//#include "hc05.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <stdint.h>
#include <stdbool.h>

// Centro para N=8 (0..7000)
#define SETPOINT     3500

#define ONESHOT125


#ifdef ONESHOT125
    // Ahora en μs de servo:
    #define BASE_SPEED   140                // neutral
    #define MAX_SPEED    160                 // tope alto seguro
    #define MIN_SPEED    125     
                                             // tope bajo seguro
    #define PWM_MOTOR_HZ 2000u               // frecuencia PWM de los motores
#else // PWM estándar
    // Ahora en μs de servo:
    #define BASE_SPEED   1500                // neutral
    #define MAX_SPEED    1900                 // tope alto seguro
    #define MIN_SPEED    1100                 // tope bajo seguro
    #define PWM_MOTOR_HZ 400u                 // frecuencia PWM de los motores
#endif


#define CONTROL_HZ   400u                 // servo @ 400 Hz


#define FAN_SPEED    200                 // velocidad fija del ventilador

#define MOTOR_LEFT_PIN   GPIO10           // TIM1_CH3 (PA10 si corresponde a tu mapeo)
#define MOTOR_RIGHT_PIN  GPIO8            // TIM1_CH1 (PA8)
#define MOTOR_FAN_PIN    GPIO9            // TIM1_CH2 (PA9)

#define TIM_LEFT_MOTOR   TIM_OC3
#define TIM_RIGHT_MOTOR  TIM_OC1
#define TIM_FAN_MOTOR    TIM_OC2

#define JS40_PIN GPIO6
#define JS40_PORT GPIOB

//static BTSerial bt;

// Ganancias “por muestra” (dt=2.5 ms). Punto de arranque conservador.
static float KP = 0.03f;
static float KD = 0.12f;
static int   last_error = 0;

uint16_t control_period = 1000000 / CONTROL_HZ;

// (dejado como lo tenías)
static const uint8_t QRE_CH[8] = {7, 6, 5, 4, 3, 2, 0, 1};
qre_array_t qre;

esc_handle_t ml, mr, mf;

const esc_config_t escL = {
    .tim       = TIM1,
    .ch        = TIM_LEFT_MOTOR,
    .gpio_port = GPIOA,
    .gpio_pin  = MOTOR_LEFT_PIN,
    .freq_hz   = PWM_MOTOR_HZ,        // 400 Hz
    .min_us    = MIN_SPEED,     // 1100
    .max_us    = MAX_SPEED      // 1900
};

const esc_config_t escR = {
    .tim       = TIM1,
    .ch        = TIM_RIGHT_MOTOR,
    .gpio_port = GPIOA,
    .gpio_pin  = MOTOR_RIGHT_PIN,
    .freq_hz   = PWM_MOTOR_HZ,        // 400 Hz
    .min_us    = MIN_SPEED,     // 1100
    .max_us    = MAX_SPEED      // 1900 (arreglado)
};

const esc_config_t escF = {
    .tim       = TIM1,
    .ch        = TIM_FAN_MOTOR,
    .gpio_port = GPIOA,
    .gpio_pin  = MOTOR_FAN_PIN,
    .freq_hz   = PWM_MOTOR_HZ,        // 400 Hz
    .min_us    = MIN_SPEED,     // 1100
    .max_us    = MAX_SPEED      // 1900 (arreglado)
};

static inline void pid_step_and_output(uint16_t position) {
    int error      = (int)position - SETPOINT;
    int derivative = error - last_error;

    int pid = (error * KP) + (derivative * KD);
    last_error = error;

    int us_right = BASE_SPEED - pid;
    int us_left  = BASE_SPEED + pid;

    if (us_right > MAX_SPEED) us_right = MAX_SPEED;
    if (us_right < MIN_SPEED) us_right = MIN_SPEED;
    if (us_left  > MAX_SPEED) us_left  = MAX_SPEED;
    if (us_left  < MIN_SPEED) us_left  = MIN_SPEED;

    esc_write_us(&mr, (uint16_t)us_right);
    esc_write_us(&ml, (uint16_t)us_left);
    //uart_printf("ML: %4u MR: %4u\n", (uint16_t)us_right, (uint16_t)us_left);
}

void init_js40 (void) {
    gpio_set_mode(JS40_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, JS40_PIN);
}

bool read_js40 (void) {
    return gpio_get(JS40_PORT, JS40_PIN);
}

int main(void) {
    clocks_init();
    timing_init();
    ui_init();
    control_timer_init_400hz();
    init_js40();
    // SerialBT_begin(115200);
    // SerialBT_println("SerialBT listo. Enviá PING"); // sin control de KEY
    //uart_init_115200();

    delay_ms_blocking(200);

    esc_init(&ml, &escL);
    esc_init(&mr, &escR);
    esc_init(&mf, &escF);

    delay_ms_blocking(200); 

    // Armado
    // esc_write_us(&ml, 1000);
    // esc_write_us(&mr, 1000);
    // esc_calibrate(&ml);
    // delay_ms_blocking(4000);
    // esc_calibrate(&mr);

    // esc_arm(&ml);
    // esc_arm(&mr);
    // esc_arm(&mf);
    

    /*      Armado      */
    esc_write_us(&mf, MIN_SPEED); // ventilador ON
    esc_write_us(&mr, MIN_SPEED);
    esc_write_us(&ml, MIN_SPEED);

    delay_ms_blocking(2000);

    //esc_arm(&ml);
    //esc_arm(&mr);

    // Sensores
    qre_init(&qre, QRE_CH, 8);

    //uart_printf("Calibrating QRE...\n");

    // Calibración (mover la regleta por línea y fondo)
    //rgb_off();
    rgb_red();
    delay_ms_blocking(500);
    qre_calibrate(&qre, 2000, 500);

    //uart_printf("Calibration done.\n");

    // Promedio (arrancá con 1 si querés tunear KP primero)
    qre_set_averaging(&qre, 1);

    //uint16_t raw_qre[8];
    rgb_off();
    rgb_blue();

    //bool fan_state = false;

    bool modo_activo = false;
    
    bool js40_read = false;
    bool led_on = false;

    while (1) {

        if(read_js40()) {
            rgb_off();     // modo OFF
            esc_write_us(&mf, MIN_SPEED);
            esc_write_us(&mr, MIN_SPEED);
            esc_write_us(&ml, MIN_SPEED);
        } else if (modo_activo) {
            esc_write_us(&mf, FAN_SPEED);
        }

        if (button1_was_pressed(100)) { // 15 ms de debounce
            modo_activo = !modo_activo;
            rgb_blue();

            if (modo_activo) {
                rgb_cyan();    // modo ON
                esc_write_us(&mf, FAN_SPEED);
            } else {
                rgb_off();     // modo OFF
                esc_write_us(&mf, MIN_SPEED);
                esc_write_us(&mr, MIN_SPEED);
                esc_write_us(&ml, MIN_SPEED);
            }
        }

        // Si el modo está activo, corré el lazo a 400 Hz
        if (modo_activo && control_tick_400hz ) {
            control_tick_400hz = false;

            uint16_t pos = qre_read_position_white(&qre);

            if (pos == (uint16_t)-1) {
                // No se ve línea: detener motores
                esc_write_us(&mr, MIN_SPEED);
                esc_write_us(&ml, MIN_SPEED);
            } else {
                //pid_step_and_output(pos);
            }
        }

    }
}