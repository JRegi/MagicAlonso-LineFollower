// qre_array.c - Implementación con scan mode + promedio (sin DMA)
#include "qre_array.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

static inline void delay_us_blocking(uint32_t us) {
    for (uint32_t i = 0; i < us; i++)
        for (volatile uint32_t j = 0; j < 12; j++) __asm__("nop"); // ~1us @72MHz (aprox)
}

static void adc1_setup_once(void) {
    static bool done = false;
    if (done) return;

    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // ADC clk = PCLK2/6 = 12 MHz (<= 14 MHz)
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

    adc_power_off(ADC1);
    rcc_periph_reset_pulse(RST_ADC1);

    // === Scan mode ===
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);   // un disparo recorre toda la secuencia
    adc_set_right_aligned(ADC1);

    // Tiempo de muestreo para todos los canales (puede bajarse si tu fuente es baja Z)
    for (int ch = 0; ch <= 17; ch++) {
        adc_set_sample_time(ADC1, ch, ADC_SMPR_SMP_55DOT5CYC);
    }

    adc_power_on(ADC1);
    delay_us_blocking(10);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    // Clave en F1 para disparo por software (SWSTART):
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);

    done = true;
}

static void gpio_setup_for_channel(uint8_t ch) {
    if (ch <= 7) { // PA0..PA7
        gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, (1 << ch));
    } else if (ch == 8) { // PB0
        gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
    } else if (ch == 9) { // PB1
        gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
    } else if (ch <= 15) { // PC0..PC5
        uint16_t pin = 1 << (ch - 10);
        gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, pin);
    }
}

// Dispara una conversión en scan mode y lee todos los canales de la secuencia.
// out debe tener al menos q->num_sensors elementos.
static void adc1_read_sequence(const qre_array_t* q, uint16_t* out) {
    // Cargar la secuencia completa (1..16 canales)
    adc_set_regular_sequence(ADC1, q->num_sensors, (uint8_t*)q->adc_channels);

    // Un solo start recorre toda la secuencia; EOC se setea en cada conversión
    adc_start_conversion_regular(ADC1);
    for (uint8_t i = 0; i < q->num_sensors; i++) {
        while (!adc_eoc(ADC1)) { /* esperar cada fin de conversión */ }
        out[i] = adc_read_regular(ADC1); // lee DR (avanza al siguiente rank)
    }
}

bool qre_init(qre_array_t* q, const uint8_t* channels, uint8_t count) {
    if (!q || !channels) return false;
    if (count == 0 || count > QRE_MAX_SENSORS) return false;

    adc1_setup_once();

    q->num_sensors = count;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t ch = channels[i];
        q->adc_channels[i] = ch;
        gpio_setup_for_channel(ch);
        q->min[i] = 4095;
        q->max[i] = 0;
    }
    q->calibrated = false;
    q->avg_samples = 1; // sin promedio por defecto
    return true;
}

void qre_set_averaging(qre_array_t* q, uint8_t samples) {
    if (!q) return;
    if (samples == 0) samples = 1;
    if (samples > 32) samples = 32; // límite sano para no alargar demasiado
    q->avg_samples = samples;
}

void qre_calibrate(qre_array_t* q, uint16_t iterations, uint32_t delay_us) {
    if (!q) return;
    uint16_t v[QRE_MAX_SENSORS];

    // Durante calibración, conviene sin promediar para cubrir extremos.
    uint8_t saved_avg = q->avg_samples;
    q->avg_samples = 1;

    for (uint16_t it = 0; it < iterations; it++) {
        adc1_read_sequence(q, v);
        for (uint8_t i = 0; i < q->num_sensors; i++) {
            if (v[i] < q->min[i]) q->min[i] = v[i];
            if (v[i] > q->max[i]) q->max[i] = v[i];
        }
        if (delay_us) delay_us_blocking(delay_us);
    }
    q->avg_samples = saved_avg;
    q->calibrated = true;
}

void qre_read_raw(const qre_array_t* q, uint16_t* out) {
    if (!q || !out) return;

    if (q->avg_samples <= 1) {
        adc1_read_sequence(q, out);
        return;
    }

    uint32_t acc[QRE_MAX_SENSORS] = {0};
    uint16_t tmp[QRE_MAX_SENSORS];

    for (uint8_t s = 0; s < q->avg_samples; s++) {
        adc1_read_sequence(q, tmp);
        for (uint8_t i = 0; i < q->num_sensors; i++) {
            acc[i] += tmp[i];
        }
        // opcional: pequeño descanso para decorrelacionar ruido (p.ej. 10 us)
        // delay_us_blocking(10);
    }

    for (uint8_t i = 0; i < q->num_sensors; i++) {
        out[i] = (uint16_t)(acc[i] / q->avg_samples);
    }
}

static inline uint16_t map_u16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_max) {
    if (in_max <= in_min) return 0;
    uint32_t num = (uint32_t)(x - in_min) * out_max;
    uint32_t den = (uint32_t)(in_max - in_min);
    uint32_t y = num / den;
    if (y > out_max) y = out_max;
    return (uint16_t)y;
}

void qre_read_calibrated(const qre_array_t* q, uint16_t* out) {
    if (!q || !out) return;

    uint16_t raw[QRE_MAX_SENSORS];
    qre_read_raw(q, raw); // ya incluye el promedio si está configurado

    if (q->calibrated) {
        for (uint8_t i = 0; i < q->num_sensors; i++) {
            out[i] = map_u16(raw[i], q->min[i], q->max[i], 1000);
        }
    } else {
        for (uint8_t i = 0; i < q->num_sensors; i++) {
            out[i] = (raw[i] * 1000u) / 4095u;
        }
    }
}

// --- Núcleo común para posición (invert=true => invierte 0..1000 -> 1000..0)
static uint16_t qre_position_core(const qre_array_t* q, bool invert) {
    if (!q) return 0;
    uint16_t val[QRE_MAX_SENSORS];
    qre_read_calibrated(q, val);

    uint32_t sum = 0;
    uint32_t wsum = 0;
    for (uint8_t i = 0; i < q->num_sensors; i++) {
        uint16_t v = val[i];
        if (invert) v = 1000 - v;      // línea negra => invertir
        uint16_t w = (uint16_t)(i * 1000u);
        sum  += (uint32_t)v * w;
        wsum += v;
    }
    if (wsum == 0) {
        // sin señal: elegir un extremo fijo (simple). Se puede mejorar guardando "last_pos".
        return invert ? 0 : (uint16_t)((q->num_sensors - 1) * 1000u);
    }
    return (uint16_t)(sum / wsum);
}

uint16_t qre_read_position_black(const qre_array_t* q) {
    return qre_position_core(q, /*invert=*/true);
}
uint16_t qre_read_position_white(const qre_array_t* q) {
    return qre_position_core(q, /*invert=*/false);
}

// Deprecada: conservada por compatibilidad.
uint16_t qre_read_position(const qre_array_t* q, bool line_is_white) {
    return qre_position_core(q, /*invert_for_black=*/!line_is_white);
}
