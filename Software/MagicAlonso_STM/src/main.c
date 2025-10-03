#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <stdint.h>
#include <stdbool.h>

#define RB 256
static volatile uint8_t tx[RB], rx[RB];
static volatile uint16_t txw=0, txr=0, rxw=0, rxr=0;

static inline bool tx_empty(){ return txw==txr; }
static inline bool tx_full(){  return ((txw+1)&(RB-1))==txr; }
static inline bool rx_empty(){ return rxw==rxr; }

static void bt_init_irq(uint32_t baud){
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,        GPIO_CNF_INPUT_FLOAT,            GPIO11);

    usart_set_baudrate(USART3, baud);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_enable(USART3);

    txw=txr=rxw=rxr=0;

    nvic_set_priority(NVIC_USART3_IRQ, 12);
    nvic_enable_irq(NVIC_USART3_IRQ);
    usart_enable_rx_interrupt(USART3);     // RXNEIE
}

static void bt_write_string(const char *s){
    while (*s){
        uint16_t next = (txw+1) & (RB-1);
        if (next==txr) break;          // buffer lleno (simple)
        tx[txw] = (uint8_t)*s++;
        txw = next;
    }
    usart_enable_tx_interrupt(USART3); // arranca TXE
}

int main(void){
    bt_init_irq(9600);
    bt_write_string("READY\r\n");       // debería verse al conectar

    for(;;){
        if (!rx_empty()){
            uint8_t b = rx[rxr]; rxr = (rxr+1)&(RB-1);
            // eco
            uint16_t next = (txw+1)&(RB-1);
            if (next!=txr){ tx[txw]=b; txw=next; usart_enable_tx_interrupt(USART3); }
        }
    }
}

#ifdef __cplusplus
extern "C" {
#endif
void usart3_isr(void){
    // RX
    if (USART_SR(USART3) & USART_SR_RXNE){
        uint8_t b = usart_recv(USART3);
        uint16_t next = (rxw+1)&(RB-1);
        if (next!=rxr){ rx[rxw]=b; rxw=next; } // si se llena, descarta
    }
    // TX
    if (USART_SR(USART3) & USART_SR_TXE){
        if (tx_empty()){
            usart_disable_tx_interrupt(USART3);
        } else {
            usart_send(USART3, tx[txr]);
            txr = (txr+1)&(RB-1);
        }
    }
}
#ifdef __cplusplus
}
#endif
