#include <stdint.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include "qre_array.h"
#include "hc05.h"

static const uint8_t  ADC_CHANNELS[8]     = {7, 6, 5, 4, 3, 2, 0, 1}; // izq→der
static const int16_t  SENSOR_POSITIONS[8] = {-1400, -1000, -600, -200, 200, 600, 1000, 1400};

static void delay_us(uint32_t us){ for (volatile uint32_t i=0;i<us*12;++i) __asm__("nop"); }
static void delay_ms(uint32_t ms){ while(ms--) delay_us(1000); }

int main(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    bt_init(9600);
    bt_write_string("BT ready\r\n");

    /* Sensores: 8 canales en PA7..PA1,PA0 (orden izq→der) */
    qtr_config_t config = {
        .number_of_sensors       = 8,
        .adc_channel_list        = ADC_CHANNELS,
        .gpioa_analog_mask       = (1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<0)|(1<<1),
        .gpiob_analog_mask       = 0,
        .gpioc_analog_mask       = 0,
        .sample_rate_hz          = 1000,  /* 1 kHz por secuencia (8 canales) */
        .averaging_samples_count = 4,     /* publica a 250 Hz */
        .sensor_positions        = SENSOR_POSITIONS
    };

    qtr_init(&config);
    qtr_start();

    /* Calibración guiada: mover por línea y fondo ~3.2 s @ 250 Hz */
    qtr_calib_reset();
    for (uint32_t frames=0; frames<800; ) {
        uint32_t fid;
        if (qtr_poll_new_frame(&fid)) {
            qtr_calib_feed();
            frames++;
        }
    }
    bt_write_string("Calibrated\r\n");

    while (1) {
        uint32_t fid;
        if (qtr_poll_new_frame(&fid)) {
            int16_t position = 0;
            qtr_get_position(&position, /*gamma=1.5*/ 3, 2);

            bt_write_string_int("position:", position);
            bt_write_string("\r\n");
        }

        /* esperita opcional para no spamear el terminal */
        delay_ms(300);
    }
}