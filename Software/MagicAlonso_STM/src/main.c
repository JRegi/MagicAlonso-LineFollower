#include "qre_array.h"
#include "esc.h"
#include "uart.h"
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
#define BASE_SPEED   1150                 // neutral
#define MAX_SPEED    1300                 // tope alto seguro
#define MIN_SPEED    1000                 // tope bajo seguro
#define PWM_HZ       400u                 // servo @ 400 Hz

#define FAN_SPEED_US 1500                 // velocidad del ventilador en us (1500 = 50%)

#define MOTOR_LEFT_PIN   GPIO10           // TIM1_CH3 (PA10 si corresponde a tu mapeo)
#define MOTOR_RIGHT_PIN  GPIO8            // TIM1_CH1 (PA8)
#define MOTOR_FAN_PIN    GPIO9            // TIM1_CH2 (PA9)

#define TIM_LEFT_MOTOR   TIM_OC3
#define TIM_RIGHT_MOTOR  TIM_OC1
#define TIM_FAN_MOTOR    TIM_OC2

// Botones
#define BUTTON1_PIN  GPIO3            // PB3 (pull-down)
#define BUTTON2_PIN  GPIO14           // PC14 (pull-down)
#define BUTTON3_PIN  GPIO15           // PC15 (pull-down)

#define BUTTON1_PORT GPIOB
#define BUTTON2_PORT GPIOC
#define BUTTON3_PORT GPIOC

// LEDs
#define RGB_RED_PIN    GPIO14           // PB12
#define RGB_GREEN_PIN  GPIO12           // PB14
#define RGB_BLUE_PIN   GPIO13           // PB13
#define RGB_PORT       GPIOB

// Ganancias “por muestra” (dt=2.5 ms). Punto de arranque conservador.
static float KP = 0.04f;
static float KD = 0.1f;
static int   last_error = 0;

// (dejado como lo tenías)
static const uint8_t QRE_CH[8] = {7, 6, 5, 4, 3, 2, 0, 1};
qre_array_t qre;

static volatile uint32_t ticks = 0;

esc_handle_t ml, mr, mf; // left, right, fan

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
const esc_config_t escFan = {
    .tim       = TIM1,
    .ch        = TIM_FAN_MOTOR, // 2
    .gpio_port = GPIOA, 
    .gpio_pin  = MOTOR_FAN_PIN, // 9
    .freq_hz   = PWM_HZ,        // 400 Hz
    .min_us    = MIN_SPEED,     // 1100
    .max_us    = MAX_SPEED      // 1900
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
    //uart_init_115200();
    delay_init_ms();

    delay_ms(200);

    esc_init(&ml, &escL);
    esc_init(&mr, &escR);
    esc_init(&mf, &escFan);

    delay_ms(200); 


    // Armado
   
    esc_write_us(&ml, 1000);
    esc_write_us(&mr, 1000);
    esc_write_us(&mf, 1000);
    delay_ms(2000);
   
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
    bool boton2 = false;
    bool boton3 = false;
    
    bool fan_on = false;
    
    bool esc_calibrated = false;
    
    inline void reset_buttons(void) {
        boton1 = false;
        boton2 = false;
        boton3 = false;
    }
    while (1) {
        delay_ms(150);


        if (gpio_get(BUTTON1_PORT, BUTTON1_PIN)) {boton1 = true; rgb_clear(); rgb_cyan();}
        if (gpio_get(BUTTON2_PORT, BUTTON2_PIN)) {boton2 = true; rgb_clear(); rgb_green();}
        if (gpio_get(BUTTON3_PORT, BUTTON3_PIN)) {boton3 = true; rgb_clear(); rgb_green();}


        // if (boton3 && !esc_calibrated) { 
        //     boton3 = false;
        //     // Armado
        //     esc_write_us(&ml, 1000);
        //     esc_write_us(&mr, 1000);
        //     esc_write_us(&mf, 1000);
        //     delay_ms(2000);
       
        //     esc_calibrated = true;
        // }

        // if (boton3 && !esc_calibrated) { 
        //     reset_buttons();

        //     // debounce / detectar hold: si tras 500 ms sigue presionado → calibrar
        //     delay_ms(50);                // pequeño debounce inicial
        //     if (gpio_get(BUTTON3_PORT, BUTTON3_PIN)) {
        //         // esperar un poco más para confirmar hold
        //         delay_ms(450);
        //         if (gpio_get(BUTTON3_PORT, BUTTON3_PIN)) {
        //             // Calibración (botón mantenido)
        //             rgb_clear(); rgb_blue();
        //             esc_calibrate(&ml);
        //             esc_calibrate(&mr);
        //             esc_calibrate(&mf);
        //             delay_ms(200); // pequeño respiro
        //             rgb_clear();
        //             esc_calibrated = true; // opcional: marcar calibrado
        //         } else {
        //             // no fue hold prolongado → hacer armado
        //             esc_write_us(&ml, 1000);
        //             esc_write_us(&mr, 1000);
        //             esc_write_us(&mf, 1000);
        //             delay_ms(2000);
        //             esc_calibrated = true;
        //         }
        //     } else {
        //         // no quedó presionado tras debounce → hacer armado
        //         esc_write_us(&ml, 1000);
        //         esc_write_us(&mr, 1000);
        //         esc_write_us(&mf, 1000);
        //         delay_ms(2000);
        //         esc_calibrated = true;
        //     }
        // }

        // if (gpio_get(BUTTON3_PORT, BUTTON3_PIN))
        // {
        //     rgb_clear(); rgb_green();
        //     esc_calibrate(&ml);
        //     esc_calibrate(&mr);
        //     esc_calibrate(&mf);

        //     delay_ms(5000);

        //     rgb_clear(); rgb_blue();
        // }
        

        // if (boton2 && esc_calibrated) { 
        //     reset_buttons();
        //     fan_on = !fan_on;

        //     if (fan_on){
        //         esc_write_us(&mf, FAN_SPEED_US);
        //     } else if (!fan_on){
        //         esc_write_us(&mf, 1000);
        //     }
        //     rgb_clear(); rgb_red(); 
        // }

        while (boton1) {
            
            if (!fan_on) {
                esc_write_us(&mf, FAN_SPEED_US);
                fan_on = true;
            }

            // esperar el próximo tick exacto (2.5 ms)
            while ((int32_t)(ticks - next) < 0) { __asm__("nop"); }
            next += 1;

            // 1) Lectura en fase
            uint16_t pos = qre_read_position_white(&qre);

            //uart_printf("Pos: %4u\n", pos);
        
            // 2) PID + salida
            if (pos == (uint16_t)(-1)) {
                // Línea no detectada: detener motores
                esc_write_us(&ml, 1000);
                esc_write_us(&mr, 1000);
            } else {
                pid_step_and_output(pos);
            }

            //uart_printf("\n");


            // for (uint8_t i = 0; i < 8; i++) {
            //     raw_qre[i] = qre_read_raw_channel(QRE_CH[i]);
            // }

            // uart_printf("QRE: %4u %4u %4u %4u %4u %4u %4u %4u\n",
            //             raw_qre[7], raw_qre[6], raw_qre[5], raw_qre[4],
            //             raw_qre[3], raw_qre[2], raw_qre[1], raw_qre[0]);


            //delay_ms(100); // simula trabajo en el loop
            
            //if (gpio_get(BUTTON1_PORT, BUTTON1_PIN)) {boton1 = false; rgb_clear(); rgb_cyan();}

        }
    }
}