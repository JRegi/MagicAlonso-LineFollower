// qre_array.h - Minimal QRE1113 (QTR analog) array for STM32F103 + libopencm3
// API inspirado en Pololu QTR: calibración min/max, lectura 0..1000,
// y posición por promedio ponderado 0..(N-1)*1000.
// Versión con funciones explícitas: _black() / _white().

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
    uint8_t  num_sensors;                 // cantidad de sensores
    uint8_t  adc_channels[QRE_MAX_SENSORS]; // ADC1_INx (0..15) en orden físico
    uint16_t min[QRE_MAX_SENSORS];        // mínimos calibrados
    uint16_t max[QRE_MAX_SENSORS];        // máximos calibrados
    bool     calibrated;
} qre_array_t;

// Inicializa ADC1 y setea GPIOs analógicos. Devuelve false si count > QRE_MAX_SENSORS.
bool qre_init(qre_array_t* q, const uint8_t* channels, uint8_t count);

// Calibra min/max leyendo 'iterations' veces. delay_us opcional entre lecturas.
void qre_calibrate(qre_array_t* q, uint16_t iterations, uint32_t delay_us);

// Lecturas
void qre_read_raw(const qre_array_t* q, uint16_t* out);        // 0..4095
void qre_read_calibrated(const qre_array_t* q, uint16_t* out); // 0..1000

// Posición (0..(N-1)*1000)
// NUEVAS (recomendadas, evitan ambigüedad):
uint16_t qre_read_position_black(const qre_array_t* q); // línea negra (invierte)
uint16_t qre_read_position_white(const qre_array_t* q); // línea blanca (no invierte)

// DEPRECATED: preferí las funciones _black/_white
uint16_t qre_read_position(const qre_array_t* q, bool line_is_white);

#ifdef __cplusplus
}
#endif

#endif // QRE_ARRAY_H