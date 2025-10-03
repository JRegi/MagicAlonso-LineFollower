#include "qre_array.h"
#include "esc.h"
#include "uart.h"
#include "receptor_ir.h"
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
#define BASE_SPEED   1125                // neutral
#define MAX_SPEED    1250                 // tope alto seguro
#define MIN_SPEED    1000                 // tope bajo seguro
#define PWM_HZ       400u                 // servo @ 400 Hz

#define MOTOR_LEFT_PIN   GPIO10           // TIM1_CH3 (PA10 si corresponde a tu mapeo)
#define MOTOR_RIGHT_PIN  GPIO8            // TIM1_CH1 (PA8)

#define TIM_LEFT_MOTOR   TIM_OC3
#define TIM_RIGHT_MOTOR  TIM_OC1

// Botones
#define BUTTON1_PIN  GPIO3           // PB3 (pull-down)
#define BUTTON2_PIN  GPIO14           // PC14 (pull-down)
#define BUTTON3_PIN  GPIO15           // PC15 (pull-down)

#define BUTTON1_PORT GPIOB
#define BUTTON2_PORT GPIOC
#define BUTTON3_PORT GPIOC

// LEDs
#define RGB_RED_PIN    GPIO14           // PB12
#define RGB_GREEN_PIN  GPIO12           // PB14
#define RGB_BLUE_PIN   GPIO13           // PB13
#define RGB_PORT      GPIOB

// Ganancias “por muestra” (dt=2.5 ms). Punto de arranque conservador.
static float KP = 0.03f;
static float KD = 0.13f;
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

    float us_right = (float)BASE_SPEED - pid;
    float us_left  = (float)BASE_SPEED + pid;

    if (us_right > MAX_SPEED) us_right = MAX_SPEED;
    else if (us_right < MIN_SPEED) us_right = MIN_SPEED;
    if (us_left  > MAX_SPEED) us_left  = MAX_SPEED;
    else if (us_left  < MIN_SPEED) us_left  = MIN_SPEED;

    esc_write_us(&mr, (uint16_t)us_right);
    esc_write_us(&ml, (uint16_t)us_left);
    //uart_printf("ML: %4u MR: %4u\n", (uint16_t)us_right, (uint16_t)us_left);
}

///
///     Atenti!
///     Mover a librería aparte luego
///

static void release_jtag_keep_swd(void) {
    rcc_periph_clock_enable(RCC_AFIO);
    // Bits 26:24 = SWJ_CFG → 010 = JTAG off, SWD on
    AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24);
}


static void user_interfaces_setup(void) {
    release_jtag_keep_swd();
    
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOB);

    rcc_periph_clock_enable(RCC_AFIO);
    // Deshabilita JTAG, deja SWD (SWJ_CFG = 010)
    // Limpia los bits SWJ y setea el modo "JTAG off, SWD on"
    // gpio_primary_remap(AFIO_MAPR_SWJ_MASK, AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON);

    // GPIOs LEDs
    gpio_set_mode(RGB_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
        RGB_RED_PIN | RGB_GREEN_PIN | RGB_BLUE_PIN);

    // Botón 1 en PB3
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BUTTON1_PIN);
    // Botones 2 y 3 en PC14/PC15
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BUTTON2_PIN | BUTTON3_PIN);

                 
    // Apagar LEDs al inicio
    gpio_clear(RGB_PORT, RGB_RED_PIN | RGB_GREEN_PIN | RGB_BLUE_PIN);
}

static void rgb_red(void)   { gpio_set(RGB_PORT, RGB_RED_PIN); }
static void rgb_green(void) { gpio_set(RGB_PORT, RGB_GREEN_PIN); } // No funca
static void rgb_blue(void)  { gpio_set(RGB_PORT, RGB_BLUE_PIN); }

static void rgb_cyan(void)  { gpio_set(RGB_PORT, RGB_GREEN_PIN | RGB_BLUE_PIN); }

static void rgb_clear(void) { gpio_clear(RGB_PORT, RGB_RED_PIN | RGB_GREEN_PIN | RGB_BLUE_PIN); }

int main(void) {
    clock_and_systick_setup();
    user_interfaces_setup();
    uart_init_115200();
    delay_init_ms();

    delay_ms(200);

    esc_init(&ml, &escL);
    esc_init(&mr, &escR);

    ir_init();

    delay_ms(200); 


    // Armado
    esc_write_us(&ml, 1000);
    esc_write_us(&mr, 1000);
    // esc_calibrate(&ml);
    // delay_ms(4000);
    // esc_calibrate(&mr);
    
    //esc_arm(&ml);
    //esc_arm(&mr);

    
    // Sensores
    qre_init(&qre, QRE_CH, 8);

    //uart_printf("Calibrating QRE...\n");

    // Calibración (mover la regleta por línea y fondo)
    rgb_red();
    delay_ms(500);
    qre_calibrate(&qre, 2000, 500);

    //uart_printf("Calibration done.\n");

    // Promedio (arrancá con 1 si querés tunear KP primero)
    qre_set_averaging(&qre, 1);

    uint32_t next = ticks; // arranca ya

    //uint16_t raw_qre[8];
    rgb_clear();
    rgb_blue();

    bool boton1 = false;

    while (1) {
        if (ir_available()) {
            uint32_t code = ir_read();
            uart_printf("IR code:", code);
        }
    }
}