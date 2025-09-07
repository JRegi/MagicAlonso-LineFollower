#include "qre_array.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <math.h> /* powf */

/* --- Hardware fijo en este ejemplo: ADC1 + DMA1_CH1 + TIM3 TRGO --- */
#define QTR_ADC_PERIPHERAL    ADC1
#define QTR_DMA_PERIPHERAL    DMA1
#define QTR_DMA_CHANNEL       DMA_CHANNEL1
#define QTR_TIMER             TIM3

#define TIMER_INPUT_CLOCK_HZ              72000000UL
#define TIMER_PRESCALER_FOR_1MHZ          (72-1)

/* --- Estado global --- */
static qtr_config_t global_config;

static volatile uint16_t *adc_buffer;        /* len = N */
static uint16_t         *average_output;     /* len = N */
static uint32_t         *accumulator;        /* len = N */
static uint16_t         *min_values;         /* len = N */
static uint16_t         *max_values;         /* len = N */

static volatile uint16_t accumulation_count = 0;
static volatile bool     frame_ready_flag    = false;
static volatile uint32_t frame_id_counter    = 0;

/* Workspace estático simple (máx 16 sensores; ajustar si necesitás más) */
#ifndef QTR_MAX_SENSORS
#define QTR_MAX_SENSORS 16
#endif
static uint16_t storage_adc_buffer[QTR_MAX_SENSORS];
static uint16_t storage_average_output[QTR_MAX_SENSORS];
static uint32_t storage_accumulator[QTR_MAX_SENSORS];
static uint16_t storage_min_values[QTR_MAX_SENSORS];
static uint16_t storage_max_values[QTR_MAX_SENSORS];

/* --- Utilidades --- */
static void set_port_as_analog(uint32_t port, uint32_t mask_bits) {
    if (!mask_bits) return;
    for (uint8_t pin = 0; pin < 16; ++pin) {
        if (mask_bits & (1U << pin)) {
            gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, (1U << pin));
        }
    }
}

/* TIM3: TRGO en update, periodo = sample_rate_hz (sin timer_reset) */
static void timer3_init_trgo(uint32_t sample_rate_hz) {
    rcc_periph_clock_enable(RCC_TIM3);
    timer_disable_counter(QTR_TIMER);
    timer_set_prescaler(QTR_TIMER, TIMER_PRESCALER_FOR_1MHZ); /* 1 MHz */
    uint32_t auto_reload = (1000000UL / sample_rate_hz) - 1UL;
    timer_set_period(QTR_TIMER, (uint16_t)auto_reload);
    timer_set_master_mode(QTR_TIMER, TIM_CR2_MMS_UPDATE);     /* TRGO = update */
}

/* DMA para ADC1 regular conversions */
static void dma_init(uint8_t sensor_count) {
    rcc_periph_clock_enable(RCC_DMA1);

    dma_channel_reset(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL);
    dma_set_peripheral_address(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL, (uint32_t)&ADC_DR(QTR_ADC_PERIPHERAL));
    dma_set_memory_address(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL, (uint32_t)adc_buffer);
    dma_set_number_of_data(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL, sensor_count);
    dma_set_read_from_peripheral(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL);
    dma_enable_memory_increment_mode(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL);
    dma_set_peripheral_size(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL, DMA_CCR_MSIZE_16BIT);
    dma_set_priority(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL, DMA_CCR_PL_HIGH);
    dma_enable_circular_mode(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL);
    dma_enable_transfer_complete_interrupt(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    dma_enable_channel(QTR_DMA_PERIPHERAL, QTR_DMA_CHANNEL);
}

/* ADC1 en scan + trigger externo por TIM3 TRGO + DMA */
static void adc1_init_scan_dma(uint8_t sensor_count, const uint8_t *channel_list) {
    rcc_periph_clock_enable(RCC_ADC1);

    /* GPIO analógicos (A/B/C) */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    set_port_as_analog(GPIOA, global_config.gpioa_analog_mask);
    set_port_as_analog(GPIOB, global_config.gpiob_analog_mask);
    set_port_as_analog(GPIOC, global_config.gpioc_analog_mask);

    adc_power_off(QTR_ADC_PERIPHERAL);
    adc_enable_scan_mode(QTR_ADC_PERIPHERAL);
    adc_set_right_aligned(QTR_ADC_PERIPHERAL);

    /* Tiempo de muestreo: ajustar según impedancia de fuente si hace falta */
    adc_set_sample_time_on_all_channels(QTR_ADC_PERIPHERAL, ADC_SMPR_SMP_28DOT5CYC);

    /* Secuencia regular (lista de canales) — castear a uint8_t* según firma libopencm3 */
    adc_set_regular_sequence(QTR_ADC_PERIPHERAL, sensor_count, (uint8_t*)channel_list);

    /* Trigger externo: TIM3 TRGO */
    adc_enable_external_trigger_regular(QTR_ADC_PERIPHERAL, ADC_CR2_EXTSEL_TIM3_TRGO);

    /* DMA del ADC */
    adc_enable_dma(QTR_ADC_PERIPHERAL);

    /* Encendido + calibración */
    adc_power_on(QTR_ADC_PERIPHERAL);
    for (volatile int i = 0; i < 8000; ++i) __asm__("nop");

#ifdef ADC_CR2_RSTCAL
    adc_reset_calibration(QTR_ADC_PERIPHERAL);
    while (ADC_CR2(QTR_ADC_PERIPHERAL) & ADC_CR2_RSTCAL);
    /* Si existe el bit CAL, iniciamos y esperamos. Esto cubre variantes sin adc_calibrate(). */
    #ifdef ADC_CR2_CAL
        ADC_CR2(QTR_ADC_PERIPHERAL) |= ADC_CR2_CAL;
        while (ADC_CR2(QTR_ADC_PERIPHERAL) & ADC_CR2_CAL);
    #endif
#endif
}

/* --- API --- */
void qtr_init(const qtr_config_t *config) {
    /* Copia mínima (por valor) */
    global_config = *config;

    /* Sanidad */
    if (global_config.number_of_sensors == 0 || global_config.number_of_sensors > QTR_MAX_SENSORS) while(1);
    if (!global_config.adc_channel_list || !global_config.sensor_positions) while(1);
    if (global_config.sample_rate_hz == 0) while(1);
    if (global_config.averaging_samples_count == 0) global_config.averaging_samples_count = 1;

    /* Mapear storage estático a punteros */
    adc_buffer     = storage_adc_buffer;
    average_output = storage_average_output;
    accumulator    = storage_accumulator;
    min_values     = storage_min_values;
    max_values     = storage_max_values;

    /* Estado inicial */
    for (uint8_t i = 0; i < global_config.number_of_sensors; ++i) {
        accumulator[i] = 0;
        average_output[i] = 0;
        min_values[i] = 0x0FFF;
        max_values[i] = 0;
    }
    accumulation_count = 0;
    frame_ready_flag   = false;
    frame_id_counter   = 0;

    /* Periféricos */
    timer3_init_trgo(global_config.sample_rate_hz);
    dma_init(global_config.number_of_sensors);
    adc1_init_scan_dma(global_config.number_of_sensors, global_config.adc_channel_list);
}

void qtr_start(void) {
    timer_enable_counter(QTR_TIMER);
}

/* Señalización de “nuevo frame promedio disponible” */
bool qtr_poll_new_frame(uint32_t *out_frame_id) {
    if (frame_ready_flag) {
        frame_ready_flag = false;
        if (out_frame_id) *out_frame_id = frame_id_counter;
        return true;
    }
    return false;
}

/* Acceso al último promedio publicado */
const uint16_t* qtr_get_values(void) {
    return average_output;
}

/* Calibración: reiniciar min/max */
void qtr_calib_reset(void) {
    for (uint8_t i = 0; i < global_config.number_of_sensors; ++i) {
        min_values[i] = 0x0FFF;
        max_values[i] = 0;
    }
}

/* Calibración: alimentar min/max con el último promedio */
void qtr_calib_feed(void) {
    for (uint8_t i = 0; i < global_config.number_of_sensors; ++i) {
        uint16_t value = average_output[i];
        if (value < min_values[i]) min_values[i] = value;
        if (value > max_values[i]) max_values[i] = value;
    }
}

/* Centroide ponderado con gamma; sin indicador de “línea perdida” */
void qtr_get_position(int16_t *position_out_scaled,
                      uint16_t gamma_num, uint16_t gamma_den)
{
    const uint8_t N = global_config.number_of_sensors;
    const float gamma = (gamma_den == 0) ? 1.0f : ((float)gamma_num / (float)gamma_den);
    const float epsilon = 1e-6f;

    float sum_of_weights = 0.0f;
    float weighted_sum   = 0.0f;

    for (uint8_t i = 0; i < N; ++i) {
        /* Normalización [0..1] con min/max (clamp implícito) */
        const uint16_t minv = min_values[i];
        const uint16_t maxv = max_values[i];

        float normalized = 0.0f;
        if (maxv > minv) {
            normalized = (float)((int32_t)average_output[i] - (int32_t)minv)
                       / (float)((int32_t)maxv - (int32_t)minv);
            if (normalized < 0.f) normalized = 0.f;
            if (normalized > 1.f) normalized = 1.f;
        }

        /* Ponderación gamma */
        float weight = (gamma == 1.0f) ? normalized : powf(normalized, gamma);

        sum_of_weights += weight;
        weighted_sum   += (float)global_config.sensor_positions[i] * weight;
    }

    /* Siempre devuelve una posición: usa epsilon para evitar /0 */
    const float position = weighted_sum / (sum_of_weights + epsilon);

    if (position_out_scaled) {
        /* redondeo simétrico a entero */
        *position_out_scaled = (int16_t)((position >= 0.f) ? (position + 0.5f) : (position - 0.5f));
    }
}

/* --- ISR DMA: fin de transferencia (N muestras) --- */
void dma1_channel1_isr(void) {
    if (DMA1_ISR & DMA_ISR_TCIF1) {
        DMA1_IFCR = DMA_IFCR_CTCIF1;

        /* Acumular valores del frame actual */
        for (uint8_t i = 0; i < global_config.number_of_sensors; ++i) {
            accumulator[i] += adc_buffer[i];
        }
        accumulation_count++;

        /* ¿Listo para publicar promedio? */
        if (accumulation_count >= global_config.averaging_samples_count) {
            for (uint8_t i = 0; i < global_config.number_of_sensors; ++i) {
                average_output[i] = (uint16_t)(accumulator[i] / (uint32_t)global_config.averaging_samples_count);
                accumulator[i] = 0;
            }
            accumulation_count = 0;
            frame_id_counter++;
            frame_ready_flag = true;  /* <-- tu PID se sincroniza chequeando este flag */
        }
    }
}
