// PID velocista + QRE + ESC (1000..1200us) en PA8 (izq, TIM1_CH1) y PA10 (der, TIM1_CH3)
// Indicadores: PB4 = CALIBRANDO (ON), PB5 = ANDANDO (blink 2 Hz)

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include "esc.h"
#include "qtr_array.h"

#define SETPOINT   3500

#define KP         0.35f
#define KI         0.00f
#define KD         2.00f

#define MAX_SPEED  1300
#define MIN_SPEED  1100
#define SPEED      1225

#define US_MIN     1000u
#define US_MAX     2000u
#define US_LIMIT   1200u

// >>> Cambios clave para que se mueva <<<
#define PWM_HZ         50u       // mejor que 50 Hz para PWM analógico en BLHeli_S
#define MIN_SPIN_US    1160u     // umbral mínimo que mantiene giro
#define BASE_FWD_US    1180u     // base hacia adelante (sigue en 1000..1200)
#define KICK_US        1200u     // pulso de arranque
#define KICK_MS        80u       // duración del pulso de arranque
// <<<

#define NUM_SENS   8
#define POS_CENTER 3500

#define LED_CAL_PORT GPIOB
#define LED_CAL_PIN  GPIO4
#define LED_RUN_PORT GPIOB
#define LED_RUN_PIN  GPIO5
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
    // si pasa de parado a movimiento, dar un pulso
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

int lastError;
float pid;
float pidRight;
float pidLeft;

int main(void){
    clock_setup();
    systick_setup();

    // ESC con 50 Hz
    esc_handle_t escL, escR;
    const esc_config_t cfgL = {.tim=TIM1,.ch=TIM_OC1,.gpio_port=GPIOA,.gpio_pin=GPIO8,
                               .freq_hz=PWM_HZ,.min_us=US_MIN,.max_us=US_MAX};
    const esc_config_t cfgR = {.tim=TIM1,.ch=TIM_OC3,.gpio_port=GPIOA,.gpio_pin=GPIO10,
                               .freq_hz=PWM_HZ,.min_us=US_MIN,.max_us=US_MAX};
    esc_init(&escL, &cfgL);
    esc_init(&escR, &cfgR);

    // Arming: 1000us ~2 s
    uint32_t t0 = millis();
    esc_begin_arming(&escL, 2000, t0);
    esc_begin_arming(&escR, 2000, t0);
    while (esc_state(&escL)!=ESC_STATE_ARMED || esc_state(&escR)!=ESC_STATE_ARMED){
        esc_write_us(&escL, US_MIN);
        esc_write_us(&escR, US_MIN);
        esc_update(&escL, millis());
        esc_update(&escR, millis());
    }

    // QRE + calibración
    qre_t qre = {0};
    qre_build_and_init(&qre);
    qre_auto_calibrate(&qre);

    // PID
    uint32_t hb_t0 = millis(); LED_OFF(LED_RUN_PORT, LED_RUN_PIN);

    while (1){
        // Heartbeat 2 Hz
        if ((uint32_t)(millis() - hb_t0) >= 250){ hb_t0 += 250; LED_TOGGLE(LED_RUN_PORT, LED_RUN_PIN); }

        uint16_t cal[NUM_SENS];
        qre_read_calibrated(&qre, cal);
        int32_t pos = qre_read_line_position(&qre, cal); // 0..7000, -1 si no hay línea

        int proportional = SETPOINT - (int)pos;
        int derivative = proportional - lastError;
        // PID discreto

        float pid = KP*(float)proportional + KD*(float)derivative;

        lastError = proportional;

        pidRight = US_LIMIT + pid;
        pidLeft = US_LIMIT - pid;

        // Floor + kickstart por motor
        uint16_t outL = ensure_spin_L((uint16_t)pidLeft);
        uint16_t outR = ensure_spin_R((uint16_t)pidRight);

        esc_write_us(&escL, outL);
        esc_write_us(&escR, outR);

        // ~400 Hz / 2 = 2.5 ms (bucle más rápido si querés)
        delay_us(2500);
    }
}