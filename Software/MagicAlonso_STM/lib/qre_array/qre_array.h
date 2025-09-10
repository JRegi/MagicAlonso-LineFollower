// qre_array.h - QRE1113 (QTR analógico) para STM32F103 + libopencm3
// Lectura en "scan mode", calibración min/max, lectura calibrada (0..1000),
// posición por promedio ponderado 0..(N-1)*1000 y promediado anti-ruido.

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
    uint8_t  num_sensors;                   // cantidad de sensores
    uint8_t  adc_channels[QRE_MAX_SENSORS]; // ADC1_INx (0..15) en orden físico

    uint16_t min[QRE_MAX_SENSORS];          // mínimos calibrados
    uint16_t max[QRE_MAX_SENSORS];          // máximos calibrados
    bool     calibrated;

    // Promedio (anti-ruido): cuántas muestras tomar por lectura (>=1)
    uint8_t  avg_samples;                   // default = 1
} qre_array_t;

// Inicializa ADC1 (scan mode) y GPIO analógicos para los canales.
// 'channels' son números de canal ADC (PA0..PA7=0..7, PB0=8, PB1=9, PC0..PC5=10..15).
bool qre_init(qre_array_t* q, const uint8_t* channels, uint8_t count);

// Opcional: configurar N muestras a promediar por lectura (1..32 aprox).
void qre_set_averaging(qre_array_t* q, uint8_t samples);

// Calibra min/max leyendo 'iterations' veces; 'delay_us' opcional entre iteraciones.
void qre_calibrate(qre_array_t* q, uint16_t iterations, uint32_t delay_us);

// Lecturas
void qre_read_raw(const qre_array_t* q, uint16_t* out);        // 0..4095 (con promedio)
void qre_read_calibrated(const qre_array_t* q, uint16_t* out); // 0..1000 (con promedio)

// Posición (0..(N-1)*1000)
// Recomendadas (sin ambigüedad):
uint16_t qre_read_position_black(const qre_array_t* q); // línea negra (invierte)
uint16_t qre_read_position_white(const qre_array_t* q); // línea blanca (no invierte)

// Deprecada: conservada por compatibilidad.
uint16_t qre_read_position(const qre_array_t* q, bool line_is_white);

#ifdef __cplusplus
}
#endif

#endif // QRE_ARRAY_H
