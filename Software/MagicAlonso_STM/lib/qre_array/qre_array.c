// qre_array.c - Implementación mínima + averaging (sin scan mode, bloqueante)
// Añadido: TH_AVG / TH_ONLINE (tipo Pololu), last_pos y pérdida de línea al último lado
#include "qre_array.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#ifndef QRE_ADC_DEV
#define QRE_ADC_DEV ADC1
#endif

// ---------- Utilidades ----------
static inline void delay_us_blocking(uint32_t us) {
    for (uint32_t i = 0; i < us; i++)
        for (volatile uint32_t j = 0; j < 12; j++) __asm__("nop"); // ~1us @72MHz aprox
}

static void adc1_setup_once(void) {
    static bool done = false;
    if (done) return;

    // Clocks
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_ADC1);

    // ADC clock por defecto (PCLK2/6 ~12MHz si PCLK2=72)
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

    // Power-on + calib
    adc_power_off(QRE_ADC_DEV);
    adc_disable_scan_mode(QRE_ADC_DEV);
    adc_set_single_conversion_mode(QRE_ADC_DEV);
    adc_set_right_aligned(QRE_ADC_DEV);
    adc_set_sample_time_on_all_channels(QRE_ADC_DEV, ADC_SMPR_SMP_55DOT5CYC); // estable

    adc_power_on(QRE_ADC_DEV);
    delay_us_blocking(10);

    adc_reset_calibration(QRE_ADC_DEV);
    adc_calibrate(QRE_ADC_DEV);

    done = true;
}

static void adc1_channel_pin_analog(uint8_t ch) {
    // ADC1_IN0..7 -> PA0..PA7
    // ADC1_IN8    -> PB0
    // ADC1_IN9    -> PB1
    // ADC1_IN10..15 -> PC0..PC5
    if (ch <= 7) { // PA0..PA7
        uint16_t pin = 1u << ch;
        gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, pin);
    } else if (ch == 8) { // PB0
        gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
    } else if (ch == 9) { // PB1
        gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
    } else if (ch <= 15) { // PC0..PC5
        uint16_t pin = 1u << (ch - 10);
        gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, pin);
    }
}

static uint16_t adc1_read_channel_once(uint8_t ch) {
    adc_set_regular_sequence(QRE_ADC_DEV, 1, &ch);
    adc_start_conversion_regular(QRE_ADC_DEV);
    while (!adc_eoc(QRE_ADC_DEV)) { /* wait */ }
    return adc_read_regular(QRE_ADC_DEV);
}

// ---------- API ----------
void qre_init(qre_array_t* q, uint8_t num_sensors, const uint8_t *adc_channels) {
    if (!q) return;
    if (num_sensors == 0 || num_sensors > QRE_MAX_SENSORS) num_sensors = QRE_MAX_SENSORS;

    q->num_sensors  = num_sensors;
    q->adc_channels = adc_channels;
    q->avg_samples  = 4;               // default
    q->th_avg       = 50;              // default (escala calibrada)
    q->th_online    = 200;             // default (escala calibrada)
    q->last_pos     = 0;

    for (uint8_t i = 0; i < q->num_sensors; i++) {
        q->cal_min[i] = 4095;
        q->cal_max[i] = 0;
    }

    adc1_setup_once();
    // Configurar pines como analógicos
    for (uint8_t i = 0; i < q->num_sensors; i++) {
        adc1_channel_pin_analog(q->adc_channels[i]);
    }
}

void qre_set_avg_samples(qre_array_t* q, uint8_t samples) {
    if (!q) return;
    if (samples < 1) samples = 1;
    if (samples > 32) samples = 32;
    q->avg_samples = samples;
}

void qre_set_thresholds(qre_array_t* q, uint16_t th_avg, uint16_t th_online) {
    if (!q) return;
    if (th_avg > 1000) th_avg = 1000;
    if (th_online > 1000) th_online = 1000;
    q->th_avg    = th_avg;
    q->th_online = th_online;
}

void qre_calibration_reset(qre_array_t* q) {
    if (!q) return;
    for (uint8_t i = 0; i < q->num_sensors; i++) {
        q->cal_min[i] = 4095;
        q->cal_max[i] = 0;
    }
}

void qre_calibration_step(qre_array_t* q) {
    if (!q) return;
    uint16_t v[QRE_MAX_SENSORS];
    qre_read_raw(q, v);
    for (uint8_t i = 0; i < q->num_sensors; i++) {
        if (v[i] < q->cal_min[i]) q->cal_min[i] = v[i];
        if (v[i] > q->cal_max[i]) q->cal_max[i] = v[i];
    }
}

void qre_read_raw(const qre_array_t* q, uint16_t* out) {
    if (!q || !out) return;
    const uint8_t N = q->num_sensors;
    const uint8_t K = q->avg_samples ? q->avg_samples : 1;

    for (uint8_t i = 0; i < N; i++) {
        uint32_t acc = 0;
        uint8_t ch = q->adc_channels[i];
        for (uint8_t k = 0; k < K; k++) {
            acc += adc1_read_channel_once(ch);
        }
        out[i] = (uint16_t)(acc / K);
    }
}

static inline uint16_t map_calibrated_u16(uint16_t x, uint16_t in_min, uint16_t in_max) {
    if (in_max <= in_min) return 0; // evita div/0: sensor sin calibrar, vuelve 0
    if (x <= in_min) return 0;
    if (x >= in_max) return 1000;
    uint32_t num = (uint32_t)(x - in_min) * 1000u;
    uint32_t den = (uint32_t)(in_max - in_min);
    return (uint16_t)(num / den); // 0..1000
}

void qre_read_calibrated(const qre_array_t* q, uint16_t* out) {
    if (!q || !out) return;
    uint16_t raw[QRE_MAX_SENSORS];
    qre_read_raw(q, raw);

    for (uint8_t i = 0; i < q->num_sensors; i++) {
        out[i] = map_calibrated_u16(raw[i], q->cal_min[i], q->cal_max[i]);
    }
}

// --- Núcleo de posición con umbrales tipo Pololu ---
static uint16_t qre_position_core(const qre_array_t* q, bool invert) {
    uint16_t val[QRE_MAX_SENSORS];
    qre_read_calibrated(q, val); // 0..1000

    const uint16_t right_edge = (uint16_t)((q->num_sensors - 1) * 1000u);
    const uint16_t center     = right_edge / 2;

    bool onLine = false;
    uint32_t sum = 0;
    uint32_t wsum = 0;

    for (uint8_t i = 0; i < q->num_sensors; i++) {
        uint16_t v = val[i];
        if (invert) v = 1000 - v;

        if (v > q->th_online) onLine = true;         // detecta línea presente
        if (v > q->th_avg) {                         // sólo ponderar lo significativo
            uint16_t w = (uint16_t)(i * 1000u);
            sum  += (uint32_t)v * w;
            wsum += v;
        }
    }

    // Si no se ve la línea (o no hubo contribución), ir al borde según el último lado
    if (!onLine || wsum == 0) {
        // Nota: usamos last_pos guardado en la struct (mutable)
        uint16_t last = q->last_pos;
        return (last < center) ? 0 : right_edge;
    }

    uint16_t pos = (uint16_t)(sum / wsum);

    // Actualizar last_pos (necesitamos mutar la struct; hacemos cast a non-const)
    ((qre_array_t*)q)->last_pos = pos;

    return pos;
}

// Línea NEGRA: NO invertir (negro ya es alto tras calibrar)
uint16_t qre_read_position_black(const qre_array_t* q) { return qre_position_core(q, false); }
// Línea BLANCA: SÍ invertir
uint16_t qre_read_position_white(const qre_array_t* q) { return qre_position_core(q, true); }

// Compatibilidad
uint16_t qre_read_position(const qre_array_t* q, bool line_is_white) {
    return qre_position_core(q, /*invert=*/line_is_white);
}
