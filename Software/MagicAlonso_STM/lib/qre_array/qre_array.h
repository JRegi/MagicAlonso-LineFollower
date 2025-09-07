#ifndef QRE_ARRAY_H
#define QRE_ARRAY_H

#include <stdint.h>
#include <stdbool.h>

/* Configuración provista por el usuario en main.c */
typedef struct {
    uint8_t   number_of_sensors;            /* N */
    const uint8_t *adc_channel_list;        /* lista ADC1_INx, len = N */

    /* Máscaras de pines que deben ir a ANALOG por puerto (A/B/C) */
    uint32_t  gpioa_analog_mask;
    uint32_t  gpiob_analog_mask;
    uint32_t  gpioc_analog_mask;

    uint32_t  sample_rate_hz;               /* ticks por segundo (cada tick = 1 secuencia de N) */
    uint16_t  averaging_samples_count;      /* cuántas secuencias promediar antes de publicar */

    /* Posiciones por sensor para el centroide (escala libre, ej. -1000..1000), len = N */
    const int16_t *sensor_positions;
} qtr_config_t;

/* --- API --- */

/* Inicializa GPIO(AN), TIM3 (TRGO a sample_rate_hz), ADC1 (scan+DMA), DMA1 CH1 */
void qtr_init(const qtr_config_t *config);

/* Arranca el timer (dispara las secuencias); desde acá el DMA e ISR hacen el resto */
void qtr_start(void);

/* Devuelve true cuando hay un nuevo promedio publicado y, opcionalmente,
 * devuelve frame_id (contador monótono de frames publicados). */
bool qtr_poll_new_frame(uint32_t *out_frame_id);

/* Último vector promedio publicado (N elementos 12 bits, 0..4095) */
const uint16_t* qtr_get_values(void);

/* Calibración guiada (min/max por canal) */
void qtr_calib_reset(void);
/* Usa el último promedio publicado y actualiza min/max */
void qtr_calib_feed(void);

/* Centroide ponderado con gamma = gamma_num/gamma_den.
 * position_out_scaled se devuelve en la MISMA escala que sensor_positions[].
 * No indica “línea perdida”: siempre entrega un valor (con epsilon para evitar /0). */
void qtr_get_position(int16_t *position_out_scaled,
                      uint16_t gamma_num, uint16_t gamma_den);

#endif /* QTR_ADC_DMA_H */
