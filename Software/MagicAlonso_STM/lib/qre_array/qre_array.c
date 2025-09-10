// qre_array.c - Implementación mínima para STM32F103 + libopencm3
#include "qre_array.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/systick.h>

static inline void delay_us_blocking(uint32_t us) {
    // Asume ~72 MHz. Bucle NOP muy simple (aproximado).
    for (uint32_t i = 0; i < us; i++) {
        for (volatile uint32_t j = 0; j < 12; j++) {
            __asm__("nop");
        }
    }
}

static void adc1_setup_once(void) {
    static bool done = false;
    if (done) return;

    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // ADC clock = PCLK2/6 = 72/6 = 12 MHz (max 14 MHz).
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

    // Deshabilitar ADC y resetear.
    adc_power_off(ADC1);
    rcc_periph_reset_pulse(RST_ADC1);

    // ADC en modo single, right align.
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);

    // Tiempo de muestreo por defecto para todos los canales: 55.5 ciclos.
    for (int ch = 0; ch <= 17; ch++) {
        adc_set_sample_time(ADC1, ch, ADC_SMPR_SMP_55DOT5CYC);
    }

    // Activar y calibrar.
    adc_power_on(ADC1);
    delay_us_blocking(10);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);

    done = true;
}

// Configura el GPIO correcto en modo analógico para el canal dado.
static void gpio_setup_for_channel(uint8_t ch) {
    if (ch <= 7) { // PA0..PA7
        gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, (1 << ch));
    } else if (ch <= 9) { // PB0..PB1
        uint16_t pin = (ch == 8) ? GPIO0 : GPIO1;
        gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, pin);
    } else if (ch <= 15) { // PC0..PC5
        uint16_t pin = 1 << (ch - 10);
        gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, pin);
    }
}

// Lectura bloqueante de un canal ADC1 (0..4095).
static uint16_t adc1_read_channel(uint8_t ch) {
    uint8_t seq_len = 1;
    adc_set_regular_sequence(ADC1, seq_len, &ch);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1)) { /* wait */ }
    return adc_read_regular(ADC1);
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
    return true;
}

void qre_calibrate(qre_array_t* q, uint16_t iterations, uint32_t delay_us) {
    if (!q) return;
    uint16_t v[QRE_MAX_SENSORS];

    for (uint16_t it = 0; it < iterations; it++) {
        // Leer todos
        for (uint8_t i = 0; i < q->num_sensors; i++) {
            v[i] = adc1_read_channel(q->adc_channels[i]);
        }
        // Actualizar min/max
        for (uint8_t i = 0; i < q->num_sensors; i++) {
            if (v[i] < q->min[i]) q->min[i] = v[i];
            if (v[i] > q->max[i]) q->max[i] = v[i];
        }
        if (delay_us) delay_us_blocking(delay_us);
    }
    q->calibrated = true;
}

void qre_read_raw(const qre_array_t* q, uint16_t* out) {
    if (!q || !out) return;
    for (uint8_t i = 0; i < q->num_sensors; i++) {
        out[i] = adc1_read_channel(q->adc_channels[i]);
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
    for (uint8_t i = 0; i < q->num_sensors; i++) {
        uint16_t raw = adc1_read_channel(q->adc_channels[i]);
        if (q->calibrated) {
            out[i] = map_u16(raw, q->min[i], q->max[i], 1000);
        } else {
            out[i] = (raw * 1000u) / 4095u;
        }
    }
}

uint16_t qre_read_position(const qre_array_t* q, bool white_line) {
    if (!q) return 0;
    uint16_t val[QRE_MAX_SENSORS];
    qre_read_calibrated(q, val);

    uint32_t sum = 0;
    uint32_t wsum = 0;
    for (uint8_t i = 0; i < q->num_sensors; i++) {
        uint16_t v = val[i];
        if (white_line) v = 1000 - v;
        uint16_t weight = (uint16_t)(i * 1000u);
        sum += (uint32_t)v * weight;
        wsum += v;
    }
    if (wsum == 0) {
        // Si no vemos línea, devolver último extremo según heurística
        return white_line ? 0 : (uint16_t)((q->num_sensors - 1) * 1000u);
    }
    return (uint16_t)(sum / wsum);
}