// qre_array.h - Minimal QRE1113 (QTR analog) array for STM32F103 + libopencm3
// + Averaging configurable (sin scan mode) + umbrales tipo Pololu + last_pos
#ifndef QRE_ARRAY_H
#define QRE_ARRAY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef QRE_MAX_SENSORS
#define QRE_MAX_SENSORS 16
#endif

typedef struct {
    // Config
    uint8_t  num_sensors;                  // cantidad de canales usados (1..QRE_MAX_SENSORS)
    const uint8_t *adc_channels;           // lista de ADC1_INx (0..15) en orden físico
    uint8_t  avg_samples;                  // 1..32 (promedio por lectura cruda)

    // Calibración por sensor (min/max crudos)
    uint16_t cal_min[QRE_MAX_SENSORS];
    uint16_t cal_max[QRE_MAX_SENSORS];

    // Estado y parámetros de posición
    uint16_t th_avg;                       // umbral para entrar al promedio (0..1000, default 50)
    uint16_t th_online;                    // umbral para decidir "estoy sobre la línea" (0..1000, default 200)
    uint16_t last_pos;                     // último resultado de posición 0..(N-1)*1000

} qre_array_t;

// --- Setup / Calibración ---
void qre_init(qre_array_t* q, uint8_t num_sensors, const uint8_t *adc_channels);
void qre_set_avg_samples(qre_array_t* q, uint8_t samples);          // clamp 1..32
void qre_set_thresholds(qre_array_t* q, uint16_t th_avg, uint16_t th_online); // escala calibrada 0..1000
void qre_calibration_reset(qre_array_t* q);
void qre_calibration_step(qre_array_t* q);                           // tomar una muestra para min/max

// --- Lecturas ---
void qre_read_raw(const qre_array_t* q, uint16_t* out);        // 0..4095
void qre_read_calibrated(const qre_array_t* q, uint16_t* out); // 0..1000 (saturado)

// --- Posición (0..(N-1)*1000) ---
// Convención tipo Pololu:
//  - Línea NEGRA: NO invertir (negro es alto en calibrado)  => _black()
//  - Línea BLANCA: SÍ invertir                              => _white()
uint16_t qre_read_position_black(const qre_array_t* q);
uint16_t qre_read_position_white(const qre_array_t* q);

// Compatibilidad (DEPRECATED): preferir _black/_white
uint16_t qre_read_position(const qre_array_t* q, bool line_is_white);

#ifdef __cplusplus
}
#endif
#endif // QRE_ARRAY_H
