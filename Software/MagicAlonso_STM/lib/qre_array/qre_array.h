//// FILE: include/qre_array.h
// Librería mínima para regleta QRE1113 sobre STM32F103 + libopencm3
// API estilo Pololu QTR: calibración (min/max), lectura calibrada (0..1000) y posición por promedio ponderado.
// Requiere: clock ya configurado (72 MHz típico).
#ifndef QRE_ARRAY_H
#define QRE_ARRAY_H

#include <stdint.h>
#include <stdbool.h>

#ifndef QRE_MAX_SENSORS
#define QRE_MAX_SENSORS 16
#endif

typedef void (*qre_delay_us_fn)(uint32_t us);

typedef struct {
    uint32_t adc;                 // ADC1 (libopencm3: ADC1)
    uint8_t  num_sensors;         // cantidad de canales usados
    const uint8_t *adc_channels;  // lista de ADCx_INy (0..15) en orden físico L→R
    const uint32_t *gpio_ports;   // puerto de cada sensor (GPIOA, ...)
    const uint16_t *gpio_pins;    // pin de cada sensor (GPIO0..GPIO15)
    bool     has_emitters;        // si tenés EN de emisores IR
    uint32_t emit_port;           // puerto EN emisores
    uint16_t emit_pin;            // pin EN emisores (alto=ON)
    bool     line_is_white;       // true si la línea es blanca
    bool     use_ambient_sub;     // true para restar luz ambiente (requiere has_emitters)
    uint8_t  smpr_default;        // ADC_SMPR_SMP_xxDOT5CYC
    qre_delay_us_fn delay_us;     // callback de retardo
} qre_cfg_t;

typedef struct {
    qre_cfg_t cfg;
    uint16_t  cal_min[QRE_MAX_SENSORS];
    uint16_t  cal_max[QRE_MAX_SENSORS];
} qre_t;

void     qre_init(qre_t *q, const qre_cfg_t *cfg);
void     qre_reset_calibration(qre_t *q);
void     qre_calibrate_on_samples(qre_t *q, uint16_t samples, uint32_t gap_us);
int      qre_read_raw(qre_t *q, uint16_t *out);
int      qre_read_calibrated(qre_t *q, uint16_t *out);
int32_t  qre_read_line_position(qre_t *q, const uint16_t *pass_calibrated);
void     qre_emitters_on(qre_t *q);
void     qre_emitters_off(qre_t *q);

#endif // QRE_ARRAY_H