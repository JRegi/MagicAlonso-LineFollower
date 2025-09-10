// qre_array.h - Minimal QRE1113 (QTR analog) array for STM32F103 + libopencm3
// API inspirado en Pololu QTR: calibración min/max, lectura calibrada (0..1000)
// y posición por promedio ponderado (0..(N-1)*1000).
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
    uint8_t num_sensors;                 // cantidad de sensores usados
    uint8_t adc_channels[QRE_MAX_SENSORS]; // lista de canales ADCx_INy (0..15)
    uint16_t min[QRE_MAX_SENSORS];       // min calibrado por sensor
    uint16_t max[QRE_MAX_SENSORS];       // max calibrado por sensor
    bool calibrated;
} qre_array_t;

// Inicializa ADC1 y configura los GPIO como entrada analógica para los canales indicados.
// channels: lista de canales ADC (0..15) en orden físico izquierda->derecha.
// count: cantidad de sensores.
// Devuelve false si count > QRE_MAX_SENSORS.
bool qre_init(qre_array_t* q, const uint8_t* channels, uint8_t count);

// Calibra min/max. iterations: repeticiones (típico 200..1000).
// delay_us: retardo entre lecturas (ej: 1000us). Si 0, sin retardo.
// Se espera que el usuario mueva el robot sobre el fondo y la línea durante la calibración.
void qre_calibrate(qre_array_t* q, uint16_t iterations, uint32_t delay_us);

// Lee crudo ADC12 (0..4095) en 'out' (size >= num_sensors).
void qre_read_raw(const qre_array_t* q, uint16_t* out);

// Lee calibrado (0..1000) donde 0=negro (baja reflectancia) y 1000=blanco.
// Si aún no se calibró, hace un mapeo simple a 0..1000 con 0..4095.
void qre_read_calibrated(const qre_array_t* q, uint16_t* out);

// Devuelve una posición ponderada 0..(N-1)*1000 (uint16).
// Si white_line=true, invierte lecturas para líneas claras sobre fondo oscuro.
uint16_t qre_read_position(const qre_array_t* q, bool white_line);

#ifdef __cplusplus
}
#endif

#endif // QRE_ARRAY_H
