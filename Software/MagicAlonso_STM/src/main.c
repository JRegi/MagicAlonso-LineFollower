#include "qre_array.h"
#include "esc.h"
#include "uart.h"
#include "timing.h"
#include "ui.h"
#include "clocks.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ir.h>

// Centro para N=8 (0..7000)
#define SETPOINT     3500

// Elegí protocolo
#define ONESHOT125

#ifdef ONESHOT125
    // Rango físico del protocolo Oneshot125 en µs
    #define ESC_MIN_US      125
    #define ESC_MAX_US      250
    #define PWM_MOTOR_HZ    2000u
#else
    // Rango físico típico PWM "servo" en µs
    #define ESC_MIN_US      1000
    #define ESC_MAX_US      2000
    #define PWM_MOTOR_HZ    400u
#endif

// ===================== Velocidades en PORCENTAJE =====================
// Todos estos son relativos al rango [ESC_MIN_US, ESC_MAX_US].

// Velocidad "base" del robot (crucero / neutro del PID)
#define BASE_SPEED_PCT    20u   // TODO: ajustar a gusto

// Velocidad fija del ventilador
#define FAN_SPEED_PCT    75u   // TODO: ajustar a gusto

// Velocidad de "freno" / apagado de motores (0% = ESC_MIN_US)
#define STOP_SPEED_PCT     0u

// ====================================================================

#define CONTROL_HZ   400u                 //  400 Hz

#define MOTOR_LEFT_PIN   GPIO10           // TIM1_CH3 (PA10 si corresponde a tu mapeo)
#define MOTOR_RIGHT_PIN  GPIO8            // TIM1_CH1 (PA8)
#define MOTOR_FAN_PIN    GPIO9            // TIM1_CH2 (PA9)

#define TIM_LEFT_MOTOR   TIM_OC3
#define TIM_RIGHT_MOTOR  TIM_OC1
#define TIM_FAN_MOTOR    TIM_OC2

#define JS40_PIN GPIO6
#define JS40_PORT GPIOB

// Ganancias “por muestra” (dt=2.5 ms). Siguen trabajando en µs.
static float KP = 0.05f;
static float KD = 0.05f;
static int   last_error = 0;

uint16_t control_period = 1000000 / CONTROL_HZ;

// (dejado como lo tenías)
static const uint8_t QRE_CH[8] = {7, 6, 5, 4, 3, 2, 0, 1};
qre_array_t qre;

esc_handle_t ml, mr, mf;

// --- Velocidades efectivas en µs (derivadas de porcentajes) ---
static uint16_t MIN_SPEED_US;    // siempre = ESC_MIN_US
static uint16_t BASE_SPEED_US;   // desde BASE_SPEED_PCT
static uint16_t MAX_SPEED_US;    // simétrico respecto de BASE_SPEED_US
static uint16_t FAN_SPEED_US;    // desde FAN_SPEED_PCT

// =================== Helper: % → µs ===================
static inline uint16_t esc_percent_to_us(float pct)
{
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    float span = (float)(ESC_MAX_US - ESC_MIN_US);
    float us   = (float)ESC_MIN_US + (pct / 100.0f) * span;

    return (uint16_t)(us + 0.5f);
}
// ======================================================

// Config ESC: rango físico completo del protocolo
const esc_config_t escL = {
    .tim       = TIM1,
    .ch        = TIM_LEFT_MOTOR,
    .gpio_port = GPIOA,
    .gpio_pin  = MOTOR_LEFT_PIN,
    .freq_hz   = PWM_MOTOR_HZ,
    .min_us    = ESC_MIN_US,
    .max_us    = ESC_MAX_US
};

const esc_config_t escR = {
    .tim       = TIM1,
    .ch        = TIM_RIGHT_MOTOR,
    .gpio_port = GPIOA,
    .gpio_pin  = MOTOR_RIGHT_PIN,
    .freq_hz   = PWM_MOTOR_HZ,
    .min_us    = ESC_MIN_US,
    .max_us    = ESC_MAX_US
};

const esc_config_t escF = {
    .tim       = TIM1,
    .ch        = TIM_FAN_MOTOR,
    .gpio_port = GPIOA,
    .gpio_pin  = MOTOR_FAN_PIN,
    .freq_hz   = PWM_MOTOR_HZ,
    .min_us    = ESC_MIN_US,
    .max_us    = ESC_MAX_US
};

static inline void pid_step_and_output(uint16_t position) {
    int error      = (int)position - SETPOINT;
    int derivative = error - last_error;

    int pid = (int)((error * KP) + (derivative * KD));
    last_error = error;

    int us_right = (int)BASE_SPEED_US - pid;
    int us_left  = (int)BASE_SPEED_US + pid;

    if (us_right > (int)MAX_SPEED_US) us_right = (int)MAX_SPEED_US;
    if (us_right < (int)MIN_SPEED_US) us_right = (int)MIN_SPEED_US;
    if (us_left  > (int)MAX_SPEED_US) us_left  = (int)MAX_SPEED_US;
    if (us_left  < (int)MIN_SPEED_US) us_left  = (int)MIN_SPEED_US;

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
    // ================== Inicializar velocidades ==================
    MIN_SPEED_US  = ESC_MIN_US;                         // mínimo físico
    BASE_SPEED_US = esc_percent_to_us(BASE_SPEED_PCT);  // base por %
    FAN_SPEED_US  = esc_percent_to_us(FAN_SPEED_PCT);   // fan por %

    // MAX_SPEED_US simétrico respecto de BASE_SPEED_US
    int max_raw = 2 * (int)BASE_SPEED_US - (int)MIN_SPEED_US;
    if (max_raw > (int)ESC_MAX_US) max_raw = (int)ESC_MAX_US;
    if (max_raw < (int)MIN_SPEED_US) max_raw = (int)MIN_SPEED_US; // por si base es muy baja
    MAX_SPEED_US = (uint16_t)max_raw;
    // =============================================================

    clocks_init();
    timing_init();
    ui_init();
    control_timer_init_400hz();
    init_js40();
    ir_sirc_init();
    // SerialBT_begin(115200);
    // SerialBT_println("SerialBT listo. Enviá PING"); // sin control de KEY
    //uart_init_115200();

    delay_ms_blocking(200);

    esc_init(&ml, &escL);
    esc_init(&mr, &escR);
    esc_init(&mf, &escF);

    delay_ms_blocking(200); 

    /*      Armado      */
    // Motores al mínimo físico (STOP_SPEED_PCT = 0%)
    esc_write_us(&mf, MIN_SPEED_US);
    esc_write_us(&mr, MIN_SPEED_US);
    esc_write_us(&ml, MIN_SPEED_US);

    delay_ms_blocking(2000);

    // Sensores
    qre_init(&qre, QRE_CH, 8);

    // Calibración (mover la regleta por línea y fondo)
    rgb_red();
    delay_ms_blocking(500);
    qre_calibrate(&qre, 2000, 500);

    // Promedio (arrancá con 1 si querés tunear KP primero)
    qre_set_averaging(&qre, 1);

    rgb_off();
    rgb_blue();

    bool modo_activo = false;

    bool js40_read = false;
    bool led_on = false;

    static bool ir_fan_on = false;
    bool ir_match = false;

    while (1) {

        if (read_js40()) {
            rgb_off();     // modo OFF
            esc_write_us(&mf, MIN_SPEED_US);
            esc_write_us(&mr, MIN_SPEED_US);
            esc_write_us(&ml, MIN_SPEED_US);
        } else if (modo_activo) {
            esc_write_us(&mf, FAN_SPEED_US);
        }

        /* ---- IR: botón remoto actúa como switch solo para la turbina ---- */
        ir_sirc_service(NULL, &ir_match, NULL);  // solo nos importa el match
        /* ----------------------------------------------------------------- */

        if (button1_was_pressed(100) || ir_match) { // 15 ms de debounce
            modo_activo = !modo_activo;
            rgb_blue();

            if (modo_activo) {
                rgb_cyan();    // modo ON
                esc_write_us(&mf, FAN_SPEED_US);
            } else {
                rgb_off();     // modo OFF
                esc_write_us(&mf, MIN_SPEED_US);
                esc_write_us(&mr, MIN_SPEED_US);
                esc_write_us(&ml, MIN_SPEED_US);
            }
        }

        // Si el modo está activo, corré el lazo a 400 Hz
        if (modo_activo && control_tick_400hz ) {
            control_tick_400hz = false;

            uint16_t pos = qre_read_position_white(&qre);

            if (pos == (uint16_t)-1) {
                // No se ve línea: detener motores
                esc_write_us(&mr, MIN_SPEED_US);
                esc_write_us(&ml, MIN_SPEED_US);
            } else {
                pid_step_and_output(pos);
            }
        }
    }
}
