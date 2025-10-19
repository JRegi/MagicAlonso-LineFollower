#include "qre_array.h"
#include "timing.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

// Setup de ADC, sin SCAN_MODE, SINGLE_CONVERSION, RIGHT_ALIGNED
static void adc1_setup_once(void) {
    static bool done = false;
    if (done) return;

    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // ADC clk = PCLK2/6 = 12 MHz (<=14 MHz)
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

    adc_power_off(ADC1);
    rcc_periph_reset_pulse(RST_ADC1);

    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);

    for (int ch = 0; ch <= 17; ch++)
        adc_set_sample_time(ADC1, ch, ADC_SMPR_SMP_55DOT5CYC);

    adc_power_on(ADC1);
    delay_us_blocking(10);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    // SWSTART en F1: seleccionar fuente + habilitar el trigger
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

// Lee un canal ADC1 una vez (bloqueante)
static uint16_t adc1_read_channel_once(uint8_t ch) {
    adc_set_regular_sequence(ADC1, 1, &ch);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1)) { /* wait */ }
    return adc_read_regular(ADC1);
}

// Inicializa ADC y sus canales para la regleta de sensores QRE
bool qre_init(qre_array_t* q, const uint8_t* channels, uint8_t count) {
    if (!q || !channels) return false;
    if (count == 0 || count > QRE_MAX_SENSORS) return false;

    adc1_setup_once();

    q->num_sensors = count;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t channel = channels[i];
        q->adc_channels[i] = channel;
        gpio_setup_for_channel(channel);
        q->min[i] = 4095;
        q->max[i] = 0;
    }
    q->calibrated = false;
    q->avg_samples = 1;
    return true;
}

// Configura el número de muestras para promediado (1..32)
void qre_set_averaging(qre_array_t* q, uint8_t samples) {
    if (!q) return;
    if (samples == 0) samples = 1;
    if (samples > 32) samples = 32;
    q->avg_samples = samples;
}

// Calibración automática: actualiza min y max con varias lecturas
void qre_calibrate(qre_array_t* q, uint16_t iterations, uint32_t delay) {
    if (!q) return;

    uint8_t saved = q->avg_samples;
    q->avg_samples = 1;

    for (uint16_t it = 0; it < iterations; it++) {
        for (uint8_t i = 0; i < q->num_sensors; i++) {
            uint16_t reading = adc1_read_channel_once(q->adc_channels[i]);

            if (reading < q->min[i]) q->min[i] = reading;
            if (reading > q->max[i]) q->max[i] = reading;
        }

        if (delay) delay_us_blocking(delay);
    }
    
    q->avg_samples = saved;
    q->calibrated  = true;
}

// Lectura cruda de un canal ADC1 (0..4095)
uint16_t qre_read_raw_channel(uint8_t ch) {
    return adc1_read_channel_once(ch);
}

// Lectura cruda de todos los sensores (0..4095), con opción de promediado
void qre_read_raw(const qre_array_t* q, uint16_t* out) {
    if (!q || !out) return;

    if (q->avg_samples <= 1) {
        for (uint8_t i = 0; i < q->num_sensors; i++)
            out[i] = adc1_read_channel_once(q->adc_channels[i]);
        return;
    }

    uint32_t acc[QRE_MAX_SENSORS] = {0};

    for (uint8_t s = 0; s < q->avg_samples; s++) {
        for (uint8_t i = 0; i < q->num_sensors; i++) {
            acc[i] += adc1_read_channel_once(q->adc_channels[i]);
        }
    }

    for (uint8_t i = 0; i < q->num_sensors; i++)
        out[i] = (uint16_t)(acc[i] / q->avg_samples);
}

// Mapea x de [in_min..in_max] a [0..out_max]
static inline uint16_t map_u16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_max) {
    if (in_max <= in_min) return 0;
    uint32_t num = (uint32_t)(x - in_min) * out_max;
    uint32_t den = (uint32_t)(in_max - in_min);
    uint32_t y = num / den;
    if (y > out_max) y = out_max;
    return (uint16_t)y;
}

// Lectura calibrada de todos los sensores (0..1000), con opción de promediado
void qre_read_calibrated(const qre_array_t* q, uint16_t* out) {
    if (!q || !out) return;

    uint16_t raw[QRE_MAX_SENSORS];
    qre_read_raw(q, raw); // usa promediado si está configurado

    if (q->calibrated) {
        for (uint8_t i = 0; i < q->num_sensors; i++)
            out[i] = map_u16(raw[i], q->min[i], q->max[i], 1000);
    } else {
        for (uint8_t i = 0; i < q->num_sensors; i++)
            out[i] = (raw[i] * 1000u) / 4095u;
    }
}

// Calcula la posición de la línea (0..(N-1)*1000) para línea negra o blanca
static uint16_t qre_position_core(const qre_array_t* q, bool invert) {
    if (!q) return 0;

    // "memoria" de última posición (0..(N-1)*1000)
    static uint16_t last_pos = 0;

    uint16_t readings[QRE_MAX_SENSORS];
    qre_read_calibrated(q, readings); // 0..1000

    const uint16_t THRESH_ONLINE = 200; // detecta presencia de línea
    const uint16_t THRESH_NOISE  = 50;  // ignora ruido en el promedio

    bool on_line = false;
    uint32_t weighted_sum = 0, total_sum = 0;

    for (uint8_t i = 0; i < q->num_sensors; i++) {
        uint16_t v = readings[i];
        if (invert) v = 1000 - v;

        if (v > THRESH_ONLINE) on_line = true;
        if (v > THRESH_NOISE) {
            weighted_sum += (uint32_t)v * (i * 1000u);
            total_sum += v;
        }
    }

    // Si no hay línea, devolver el último extremo conocido para una corrección rápida
    // Si la corrección es demasiado brusca, recomiendo cambiar el condicional
    // por "if (!on_line) return last_pos;" para mantener la última posición conocida.
    // PD: hacer ese cambio puede solucionar que el robot se sale si salta en la rampa.
    
    if (!on_line) {
        uint16_t mid = (uint16_t)((q->num_sensors - 1) * 1000u / 2u);
        return (last_pos < mid) ? 0u : (uint16_t)((q->num_sensors - 1) * 1000u);
    }

    if (total_sum == 0) return last_pos; // por seguridad
    uint16_t position = (uint16_t)(weighted_sum / total_sum);
    return position;
}

// Devuelve la posición de la línea (0..(N-1)*1000) para línea negra o blanca
uint16_t qre_read_position_black(const qre_array_t* q) { return qre_position_core(q, false); }
uint16_t qre_read_position_white(const qre_array_t* q) { return qre_position_core(q, true); }