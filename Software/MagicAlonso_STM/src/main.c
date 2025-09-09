// PID velocista + QRE + ESC (1000..1200us) en PA8 (izq, TIM1_CH1) y PA10 (der, TIM1_CH3)
// Indicadores: PB4 = CALIBRANDO (ON), PB5 = ANDANDO (blink 2 Hz)

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include "esc.h"
#include "qre_array.h"

#define MOTOR_LEFT_PIN  GPIO10   // TIM1_CH3
#define MOTOR_RIGHT_PIN GPIO8    // TIM1_CH1

#define TIM_LEFT_MOTOR  TIM_OC3
#define TIM_RIGHT_MOTOR TIM_OC1

// --- Ganancias (tu estilo PD: sin integral) ---
#define KP         0.30f
#define KD         0.00f
#define SETPOINT   0     // trabajamos con posición centrada, 0 es el centro

#define US_MIN     1000u
#define US_MAX     1150u

// >>> Parámetros de movimiento que ya te funcionan <<<
#define PWM_HZ         400u
#define MIN_SPIN_US    1025u
#define BASE_FWD_US    1150u
#define KICK_US        1200u
#define KICK_MS        50u
#define LOST_BIAS_US   50u
// <<<<

#define NUM_SENS   8
#define POS_CENTER 3500  // centro para qre_read_line_position con 8 sensores

#define LED_CAL_PORT GPIOB
#define LED_CAL_PIN  GPIO15
#define LED_RUN_PORT GPIOB
#define LED_RUN_PIN  GPIO14
#define LED_ON(p, n)    gpio_set((p), (n))
#define LED_OFF(p, n)   gpio_clear((p), (n))
#define LED_TOGGLE(p,n) gpio_toggle((p), (n))

void sys_tick_handler(void);
static volatile uint32_t _ms = 0;
void sys_tick_handler(void){ _ms++; }
static inline uint32_t millis(void){ return _ms; }

static void delay_us(uint32_t us){ for (volatile uint32_t i=0;i<us*12;++i) __asm__("nop"); }
static void delay_ms(uint32_t ms){ while(ms--) delay_us(1000); }

static void clock_setup(void){
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_CAL_PIN | LED_RUN_PIN);
    LED_OFF(LED_CAL_PORT, LED_CAL_PIN);
    LED_OFF(LED_RUN_PORT, LED_RUN_PIN);
}

static void systick_setup(void){
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(72000 - 1); // 1 ms
    systick_interrupt_enable();
    systick_counter_enable();
}

static void qre_build_and_init(qre_t *qr){
    static const uint32_t ports[NUM_SENS]={GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA};
    static const uint16_t pins [NUM_SENS]={GPIO1,GPIO0,GPIO2,GPIO3,GPIO4,GPIO5,GPIO6,GPIO7};
    static const uint8_t  ch   [NUM_SENS]={1,0,2,3,4,5,6,7};
    qre_cfg_t cfg = {
        .adc=ADC1,.num_sensors=NUM_SENS,.adc_channels=ch,.gpio_ports=ports,.gpio_pins=pins,
        .has_emitters=false,.emit_port=0,.emit_pin=0,.line_is_white=true,.use_ambient_sub=false,
        .smpr_default=ADC_SMPR_SMP_55DOT5CYC,.delay_us=delay_us,
    };
    qre_init(qr,&cfg);
}

static void qre_auto_calibrate(qre_t *qr){
    LED_ON(LED_CAL_PORT, LED_CAL_PIN);
    qre_calibrate_on_samples(qr, 400, 5000);   // ~2 s
    LED_OFF(LED_CAL_PORT, LED_CAL_PIN);
}

static inline uint16_t clamp_u16(int v, int lo, int hi){
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}

// --- Kickstart / floor por motor ---
static uint16_t last_cmd_L = US_MIN, last_cmd_R = US_MIN;
static uint32_t kick_until_L = 0,     kick_until_R = 0;

static inline uint16_t ensure_spin_L(uint16_t target){
    if (target <= US_MIN) { kick_until_L=0; last_cmd_L=US_MIN; return US_MIN; }
    if (last_cmd_L <= US_MIN && target >= MIN_SPIN_US){
        kick_until_L = millis() + KICK_MS;
        last_cmd_L = KICK_US;
        return KICK_US;
    }
    if (kick_until_L && (int32_t)(millis() - kick_until_L) < 0){
        last_cmd_L = KICK_US;
        return KICK_US;
    } else kick_until_L = 0;
    if (target < MIN_SPIN_US) target = MIN_SPIN_US;
    last_cmd_L = target;
    return target;
}

static inline uint16_t ensure_spin_R(uint16_t target){
    if (target <= US_MIN) { kick_until_R=0; last_cmd_R=US_MIN; return US_MIN; }
    if (last_cmd_R <= US_MIN && target >= MIN_SPIN_US){
        kick_until_R = millis() + KICK_MS;
        last_cmd_R = KICK_US;
        return KICK_US;
    }
    if (kick_until_R && (int32_t)(millis() - kick_until_R) < 0){
        last_cmd_R = KICK_US;
        return KICK_US;
    } else kick_until_R = 0;
    if (target < MIN_SPIN_US) target = MIN_SPIN_US;
    last_cmd_R = target;
    return target;
}

int main(void){
    clock_setup();
    systick_setup();

    // ESC con 400 Hz
    esc_handle_t escL, escR;
    const esc_config_t cfgL = {.tim=TIM1,.ch=TIM_LEFT_MOTOR,.gpio_port=GPIOA,.gpio_pin=MOTOR_LEFT_PIN,
                               .freq_hz=PWM_HZ,.min_us=US_MIN,.max_us=US_MAX};
    const esc_config_t cfgR = {.tim=TIM1,.ch=TIM_RIGHT_MOTOR,.gpio_port=GPIOA,.gpio_pin=MOTOR_RIGHT_PIN,
                               .freq_hz=PWM_HZ,.min_us=US_MIN,.max_us=US_MAX};
    esc_init(&escL, &cfgL);
    esc_init(&escR, &cfgR);

    esc_write_us(&escL, US_MIN); // stop
    esc_write_us(&escR, US_MIN); // stop
    delay_ms(500);                // espera a que los ESCs inicialicen
    esc_write_us(&escL, US_MAX); // stop
    esc_write_us(&escR, US_MAX); // stop
    delay_ms(500);
    esc_write_us(&escL, US_MIN); // stop
    esc_write_us(&escR, US_MIN); // stop

    // QRE + calibración
    qre_t qre = {0};
    qre_build_and_init(&qre);
    qre_auto_calibrate(&qre);

    // --- PID estilo “tu PD simple” ---
    int16_t lastError = 0;
    uint32_t hb_t0 = millis(); LED_OFF(LED_RUN_PORT, LED_RUN_PIN);

    while (1){
        // Heartbeat 2 Hz
        if ((uint32_t)(millis() - hb_t0) >= 250){ hb_t0 += 250; LED_TOGGLE(LED_RUN_PORT, LED_RUN_PIN); }

        uint16_t cal[NUM_SENS];
        qre_read_calibrated(&qre, cal);
        int32_t pos = qre_read_line_position(&qre, cal); // 0..7000, -1 si no hay línea

        if (pos < 0){
            // Mantener avance con sesgo hacia el último error (igual que tu versión “funciona”)
            int sgn = (lastError > 0) ? 1 : (lastError < 0 ? -1 : 0);
            int left_us  = (int)BASE_FWD_US + (int)sgn * (int)LOST_BIAS_US;
            int right_us = (int)BASE_FWD_US - (int)sgn * (int)LOST_BIAS_US;

            left_us  = clamp_u16(left_us,  US_MIN, US_MAX);
            right_us = clamp_u16(right_us, US_MIN, US_MAX);

            uint16_t outL = ensure_spin_L((uint16_t)left_us);
            uint16_t outR = ensure_spin_R((uint16_t)right_us);
            esc_write_us(&escL, outL);
            esc_write_us(&escR, outR);

            // No tocamos lastError (recordamos el lado), ni acumuladores (no hay KI)
            delay_ms(10);
            continue;
        }

        // ====== PD “tu estilo” ======
        // posición centrada (−3500..+3500), setpoint=0
        int16_t position_centered = (int16_t)pos - POS_CENTER;
        int16_t proportional = position_centered - SETPOINT;
        int16_t derivative   = proportional - lastError;
        float pid            = (proportional * KP) + (derivative * KD);
        lastError            = proportional;

        // Suma/resta directa en µs (como tu main original)
        int left_us  = (int)BASE_FWD_US + (int)pid;
        int right_us = (int)BASE_FWD_US - (int)pid;

        // Clip, floor y kickstart por motor
        left_us  = clamp_u16(left_us,  US_MIN, US_MAX);
        right_us = clamp_u16(right_us, US_MIN, US_MAX);

        uint16_t outL = ensure_spin_L((uint16_t)left_us);
        uint16_t outR = ensure_spin_R((uint16_t)right_us);

        esc_write_us(&escL, outL);
        esc_write_us(&escR, outR);

        // lazo ~400/2 Hz
        delay_us(2500);
    }
}