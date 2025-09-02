//// FILE: src/qre_array.c
#include "qtr_array.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

static void qre_config_gpio_analog(const qre_t *q){
    for (uint8_t i=0; i<q->cfg.num_sensors; ++i) {
        gpio_set_mode(q->cfg.gpio_ports[i], GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, q->cfg.gpio_pins[i]);
    }
    if (q->cfg.has_emitters){
        gpio_set_mode(q->cfg.emit_port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, q->cfg.emit_pin);
        gpio_clear(q->cfg.emit_port, q->cfg.emit_pin); // emisores OFF
    }
}

static void qre_config_adc(const qre_t *q){
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(q->cfg.adc);
    // fADC <= 14 MHz (F103). Con PCLK2=72 MHz:
    rcc_set_adcpre(RCC_CFGR_ADCPRE_DIV6); // 72/6 = 12 MHz
    adc_set_right_aligned(q->cfg.adc);
    for (uint8_t i=0; i<q->cfg.num_sensors; ++i) {
        adc_set_sample_time(q->cfg.adc, q->cfg.adc_channels[i], q->cfg.smpr_default);
    }
    adc_power_on(q->cfg.adc);
    if (q->cfg.delay_us) q->cfg.delay_us(5);
    adc_reset_calibration(q->cfg.adc);
    adc_calibrate(q->cfg.adc);
}

void qre_init(qre_t *q, const qre_cfg_t *cfg){
    q->cfg = *cfg;
    if (q->cfg.num_sensors > QRE_MAX_SENSORS) q->cfg.num_sensors = QRE_MAX_SENSORS;
    qre_config_gpio_analog(q);
    qre_config_adc(q);
    qre_reset_calibration(q);
}

void qre_reset_calibration(qre_t *q){
    for (uint8_t i=0;i<q->cfg.num_sensors;++i){ q->cal_min[i]=4095; q->cal_max[i]=0; }
}

void qre_emitters_on(qre_t *q){ if (q->cfg.has_emitters) gpio_set(q->cfg.emit_port, q->cfg.emit_pin); }
void qre_emitters_off(qre_t *q){ if (q->cfg.has_emitters) gpio_clear(q->cfg.emit_port, q->cfg.emit_pin); }

static uint16_t read_one_channel(qre_t *q, uint8_t ch){
    uint8_t seq[1] = { ch };
    adc_set_regular_sequence(q->cfg.adc, 1, seq);
    adc_start_conversion_direct(q->cfg.adc);
    while (!adc_eoc(q->cfg.adc)) { /* wait */ }
    return adc_read_regular(q->cfg.adc);
}

int qre_read_raw(qre_t *q, uint16_t *out){
    if (q->cfg.use_ambient_sub && q->cfg.has_emitters){
        qre_emitters_off(q);
        if (q->cfg.delay_us) q->cfg.delay_us(100);
        uint16_t amb[QRE_MAX_SENSORS];
        for (uint8_t i=0;i<q->cfg.num_sensors;++i){ amb[i]=read_one_channel(q, q->cfg.adc_channels[i]); }
        qre_emitters_on(q);
        if (q->cfg.delay_us) q->cfg.delay_us(100);
        for (uint8_t i=0;i<q->cfg.num_sensors;++i){
            uint16_t v_on = read_one_channel(q, q->cfg.adc_channels[i]);
            int32_t v = (int32_t)v_on - (int32_t)amb[i];
            out[i] = (v<=0)?0:(v>4095?4095:(uint16_t)v);
        }
    } else {
        if (q->cfg.has_emitters){ qre_emitters_on(q); if (q->cfg.delay_us) q->cfg.delay_us(50); }
        for (uint8_t i=0;i<q->cfg.num_sensors;++i){ out[i]=read_one_channel(q, q->cfg.adc_channels[i]); }
    }
    return q->cfg.num_sensors;
}

void qre_calibrate_on_samples(qre_t *q, uint16_t samples, uint32_t gap_us){
    uint16_t v[QRE_MAX_SENSORS];
    for (uint16_t s=0; s<samples; ++s){
        qre_read_raw(q, v);
        for (uint8_t i=0;i<q->cfg.num_sensors;++i){
            if (v[i]<q->cal_min[i]) q->cal_min[i]=v[i];
            if (v[i]>q->cal_max[i]) q->cal_max[i]=v[i];
        }
        if (q->cfg.delay_us) q->cfg.delay_us(gap_us);
    }
}

int qre_read_calibrated(qre_t *q, uint16_t *out){
    uint16_t raw[QRE_MAX_SENSORS];
    qre_read_raw(q, raw);
    for (uint8_t i=0;i<q->cfg.num_sensors;++i){
        uint16_t mn=q->cal_min[i], mx=q->cal_max[i], v=raw[i];
        if (mx<=mn){ out[i]=0; continue; }
        int32_t scaled = ((int32_t)(v-mn)*1000)/(int32_t)(mx-mn);
        if (scaled<0) scaled=0; if (scaled>1000) scaled=1000;
        if (q->cfg.line_is_white) scaled = 1000 - scaled;
        out[i]=(uint16_t)scaled;
    }
    return q->cfg.num_sensors;
}

int32_t qre_read_line_position(qre_t *q, const uint16_t *pass_calibrated){
    const uint8_t N=q->cfg.num_sensors;
    uint16_t tmp[QRE_MAX_SENSORS];
    const uint16_t *v = pass_calibrated ? pass_calibrated : (qre_read_calibrated(q,tmp), tmp);
    uint32_t sum=0, weighted=0;
    for (uint8_t i=0;i<N;++i){ sum+=v[i]; weighted += (uint32_t)v[i] * (uint32_t)(i*1000); }
    if (sum < (uint32_t)(N*50)) return -1;
    return (int32_t)(weighted/sum); // 0..(N-1)*1000
}