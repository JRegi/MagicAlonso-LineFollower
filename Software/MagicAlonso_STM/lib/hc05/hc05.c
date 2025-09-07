#include "bt_hc05.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

/* ---- Tamaños de buffers (podés ajustarlos) ---- */
#ifndef BT_TX_BUFFER_SIZE
#define BT_TX_BUFFER_SIZE 256
#endif
#ifndef BT_RX_BUFFER_SIZE
#define BT_RX_BUFFER_SIZE 256
#endif

/* ---- Buffers y estado ---- */
static volatile uint8_t tx_buffer[BT_TX_BUFFER_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0;

static volatile uint8_t rx_buffer[BT_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;

static inline uint16_t advance_index(uint16_t index, uint16_t size) {
    index++;
    if (index >= size) index = 0;
    return index;
}

void bt_init(uint32_t baudrate)
{
    /* Clocks */
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    /* PB10 = USART3_TX (AF push-pull), PB11 = USART3_RX (input floating) */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO11);

    /* USART3: 8N1, sin flow control */
    usart_set_baudrate(USART3, baudrate);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    /* Habilitar IRQ de USART3: RX siempre, TX solo cuando haya datos */
    nvic_enable_irq(NVIC_USART3_IRQ);
    usart_enable_rx_interrupt(USART3);

    /* Limpiar buffers */
    tx_head = tx_tail = 0;
    rx_head = rx_tail = 0;

    usart_enable(USART3);
}

size_t bt_write(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t*)data;
    size_t accepted = 0;

    for (size_t i = 0; i < length; ++i) {
        uint16_t next_head = advance_index(tx_head, BT_TX_BUFFER_SIZE);
        if (next_head == tx_tail) {
            /* Buffer lleno: salimos (no bloquea) */
            break;
        }
        tx_buffer[tx_head] = bytes[i];
        tx_head = next_head;
        accepted++;
    }

    /* Disparar transmisión por IRQ (TXE) */
    usart_enable_tx_interrupt(USART3);
    return accepted;
}

size_t bt_write_string(const char *text)
{
    size_t n = 0;
    while (*text) {
        size_t pushed = bt_write(text, 1);
        if (pushed == 0) break; /* buffer lleno, no bloqueamos */
        text++; n += pushed;
    }
    return n;
}

size_t bt_available(void)
{
    if (rx_head >= rx_tail) return (size_t)(rx_head - rx_tail);
    return (size_t)(BT_RX_BUFFER_SIZE - (rx_tail - rx_head));
}

bool bt_read_byte(uint8_t *out_byte)
{
    if (rx_head == rx_tail) return false; /* vacío */
    if (out_byte) *out_byte = rx_buffer[rx_tail];
    rx_tail = advance_index(rx_tail, BT_RX_BUFFER_SIZE);
    return true;
}

size_t bt_read(uint8_t *out_buffer, size_t max_length)
{
    size_t count = 0;
    while (count < max_length && rx_head != rx_tail) {
        out_buffer[count++] = rx_buffer[rx_tail];
        rx_tail = advance_index(rx_tail, BT_RX_BUFFER_SIZE);
    }
    return count;
}

/* ---- ISR USART3: RXNE y TXE ---- */
void usart3_isr(void)
{
    /* RX: dato recibido */
    if (USART_SR(USART3) & USART_SR_RXNE) {
        uint8_t byte = usart_recv(USART3); /* leer DR limpia RXNE */
        uint16_t next_head = advance_index(rx_head, BT_RX_BUFFER_SIZE);
        if (next_head == rx_tail) {
            /* Overflow: descartamos el byte más viejo (avanzamos tail) */
            rx_tail = advance_index(rx_tail, BT_RX_BUFFER_SIZE);
        }
        rx_buffer[rx_head] = byte;
        rx_head = next_head;
    }

    /* TX: DR vacío, podemos enviar siguiente */
    if (USART_SR(USART3) & USART_SR_TXE) {
        if (tx_head != tx_tail) {
            uint8_t byte = tx_buffer[tx_tail];
            tx_tail = advance_index(tx_tail, BT_TX_BUFFER_SIZE);
            usart_send(USART3, byte);
        } else {
            /* Nada por enviar: apagamos la IRQ TX hasta que haya nuevos datos */
            usart_disable_tx_interrupt(USART3);
        }
    }
}