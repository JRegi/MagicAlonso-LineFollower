#ifndef QRE_ARRAY_H
#define QRE_ARRAY_H

#include <stdint.h>
#include <stdbool.h>

#ifndef QRE_MAX_SENSORS
#define QRE_MAX_SENSORS 16
#endif

typedef struct {
    uint8_t  num_sensors;                   // cantidad de sensores
    uint8_t  adc_channels[QRE_MAX_SENSORS]; // ADC1_INx (0..15) en orden físico
    uint16_t min[QRE_MAX_SENSORS];          // mínimos calibrados
    uint16_t max[QRE_MAX_SENSORS];          // máximos calibrados
    bool     calibrated;

    uint8_t  avg_samples;                   // Nº de muestras por lectura (>=1). Default: 1
} qre_array_t;

// Inicializa ADC y sus canales para la regleta de sensores QRE
bool qre_init(qre_array_t* q, const uint8_t* channels, uint8_t count);

// Configura el número de muestras para promediado (1..32)
void qre_set_averaging(qre_array_t* q, uint8_t samples);

// Calibración automática: actualiza min y max con varias lecturas
void qre_calibrate(qre_array_t* q, uint16_t iterations, uint32_t delay_us);

// Lectura cruda de un canal ADC1 (0..4095)
uint16_t qre_read_raw_channel(uint8_t ch);

// Lectura cruda de todos los sensores (0..4095), con opción de promediado
void qre_read_raw(const qre_array_t* q, uint16_t* out);        // 0..4095

// Lectura calibrada de todos los sensores (0..1000), con opción de promediado
void qre_read_calibrated(const qre_array_t* q, uint16_t* out); // 0..1000

// Devuelve la posición de la línea (0..(N-1)*1000) para línea negra o blanca
uint16_t qre_read_position_black(const qre_array_t* q); // línea negra (NO invierte)
uint16_t qre_read_position_white(const qre_array_t* q); // línea blanca (SÍ invierte)

#endif
