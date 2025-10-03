#include "hc05.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

/* ====== Validaciones en compile-time ====== */
#if (BT_TX_RB_SIZE & (BT_TX_RB_SIZE-1)) || (BT_RX_RB_SIZE & (BT_RX_RB_SIZE-1))
# error "BT_TX_RB_SIZE y BT_RX_RB_SIZE deben ser potencias de 2"
#endif

/* ====== Ring buffers ====== */
static volatile uint8_t  g_tx[BT_TX_RB_SIZE];
static volatile uint8_t  g_rx[BT_RX_RB_SIZE];
static volatile uint16_t g_tx_w = 0, g_tx_r = 0;
static volatile uint16_t g_rx_w = 0, g_rx_r = 0;
static volatile uint32_t g_rx_overruns = 0;

#define RB_MASK_TX (BT_TX_RB_SIZE-1)
#define RB_MASK_RX (BT_RX_RB_SIZE-1)

static inline bool tx_empty(void){ return g_tx_w == g_tx_r; }
static inline bool tx_full(void){  return ((g_tx_w + 1) & RB_MASK_TX) == g_tx_r; }
static inline bool rx_empty(void){ return g_rx_w == g_rx_r; }
static inline bool rx_full(void){  return ((g_rx_w + 1) & RB_MASK_RX) == g_rx_r; }

/* ====== Init ====== */
void bt_init(uint32_t baud, uint8_t nvic_prio)
{
    /* Clocks */
    rcc_periph_clock_enable(BT_RCC_GPIO);
    rcc_periph_clock_enable(BT_RCC_USART);

    /* Pines: TX AF push-pull 50MHz, RX input floating */
    gpio_set_mode(BT_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, BT_TX_PIN);
    gpio_set_mode(BT_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, BT_RX_PIN);

    /* USART: 8N1, sin HW flow */
    usart_set_baudrate(BT_USART, baud);
    usart_set_databits(BT_USART, 8);
    usart_set_stopbits(BT_USART, USART_STOPBITS_1);
    usart_set_parity(BT_USART, USART_PARITY_NONE);
    usart_set_flow_control(BT_USART, USART_FLOWCONTROL_NONE);
    usart_enable(BT_USART);

    /* Limpiar buffers e índices */
    g_tx_w = g_tx_r = 0;
    g_rx_w = g_rx_r = 0;
    g_rx_overruns = 0;

    /* NVIC + IRQ: RX siempre habilitada; TXE solo cuando haya datos */
    nvic_set_priority(BT_NVIC_USART_IRQ, nvic_prio);
    nvic_enable_irq(BT_NVIC_USART_IRQ);
    usart_enable_rx_interrupt(BT_USART);     /* RXNEIE */
    usart_disable_tx_interrupt(BT_USART);    /* TXEIE (se habilita al encolar) */
}

/* ====== TX API ====== */
bool bt_write_byte(uint8_t b)
{
    uint16_t next = (g_tx_w + 1) & RB_MASK_TX;
    if (next == g_tx_r) return false; /* lleno */
    g_tx[g_tx_w] = b;
    g_tx_w = next;
    usart_enable_tx_interrupt(BT_USART); /* arrancar TXE */
    return true;
}

size_t bt_write(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t*)data;
    size_t pushed = 0;
    while (pushed < len) {
        uint16_t next = (g_tx_w + 1) & RB_MASK_TX;
        if (next == g_tx_r) break;  /* lleno */
        g_tx[g_tx_w] = p[pushed++];
        g_tx_w = next;
    }
    if (!tx_empty()) usart_enable_tx_interrupt(BT_USART);
    return pushed;
}

void bt_write_all(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t*)data;
    size_t sent = 0;
    while (sent < len) {
        size_t n = bt_write(p + sent, len - sent);
        if (n == 0) { /* buffer lleno: esperar a que ISR drene un poco */
            /* spin corto; si tenés RTOS, dormí un tick */
            continue;
        }
        sent += n;
    }
}

size_t bt_write_string(const char *s)
{
    size_t cnt = 0;
    while (*s) {
        if (!bt_write_byte((uint8_t)*s)) break;
        ++cnt; ++s;
    }
    return cnt;
}

/* ====== RX API ====== */
bool bt_read_byte(uint8_t *out)
{
    if (rx_empty()) return false;
    *out = g_rx[g_rx_r];
    g_rx_r = (g_rx_r + 1) & RB_MASK_RX;
    return true;
}

size_t bt_read(void *dst, size_t maxlen)
{
    uint8_t *q = (uint8_t*)dst;
    size_t got = 0;
    while (got < maxlen && !rx_empty()) {
        q[got++] = g_rx[g_rx_r];
        g_rx_r = (g_rx_r + 1) & RB_MASK_RX;
    }
    return got;
}

size_t bt_rx_available(void)
{
    int32_t diff = (int32_t)g_rx_w - (int32_t)g_rx_r;
    if (diff < 0) diff += BT_RX_RB_SIZE;
    return (size_t)diff;
}

size_t bt_tx_free(void)
{
    int32_t diff = (int32_t)g_tx_w - (int32_t)g_tx_r;
    if (diff < 0) diff += BT_TX_RB_SIZE;
    /* libre = tamaño - ocupados - 1 */
    return (size_t)(BT_TX_RB_SIZE - diff - 1);
}

uint32_t bt_get_and_clear_rx_overruns(void)
{
    uint32_t n = g_rx_overruns;
    g_rx_overruns = 0;
    return n;
}

/* ====== Helpers binarios (LE) ====== */
void bt_send_u16_le(uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    bt_write_all(b, 2);
}
void bt_send_i16_le(int16_t v) { bt_send_u16_le((uint16_t)v); }

void bt_send_u32_le(uint32_t v)
{
    uint8_t b[4] = {
        (uint8_t)(v & 0xFF),
        (uint8_t)((v >> 8) & 0xFF),
        (uint8_t)((v >> 16) & 0xFF),
        (uint8_t)((v >> 24) & 0xFF)
    };
    bt_write_all(b, 4);
}
void bt_send_i32_le(int32_t v) { bt_send_u32_le((uint32_t)v); }

void bt_send_float_le(float f)
{
    union { float f; uint8_t b[4]; } u = { .f = f };
    bt_write_all(u.b, 4);
}

/* ====== Helpers de texto (livianos, sin printf) ====== */
static void _bt_print_rev(const char *buf, int len)
{
    for (int i = len - 1; i >= 0; --i) bt_write_all(&buf[i], 1);
}

void bt_print_uint(uint32_t v)
{
    if (v == 0) { uint8_t c='0'; bt_write_all(&c,1); return; }
    char tmp[10];  /* 4294967295 -> 10 dígitos */
    int n = 0;
    while (v > 0 && n < (int)sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    _bt_print_rev(tmp, n);
}

void bt_print_int(int32_t v)
{
    if (v < 0) { uint8_t c='-'; bt_write_all(&c,1); v = -v; }
    bt_print_uint((uint32_t)v);
}

void bt_println(const char *s)
{
    bt_write_all(s, __builtin_strlen(s));
    bt_write_all("\r\n", 2);
}

/* ====== ISR real, nombre configurable por macro ====== */
void BT_USART_ISR(void)
{
    /* --- RX: si hay dato, lo metemos al ring --- */
    if (usart_get_flag(BT_USART, USART_SR_RXNE)) {
        uint8_t b = usart_recv(BT_USART); /* leer DR limpia RXNE */
        uint16_t next = (g_rx_w + 1) & RB_MASK_RX;
        if (next != g_rx_r) {
            g_rx[g_rx_w] = b; g_rx_w = next;
        } else {
            /* overflow de software (ring lleno) */
            g_rx_overruns++;
        }
    }

    /* --- TX: mientras TXE=1 y haya datos, seguir enviando --- */
    if (usart_get_flag(BT_USART, USART_SR_TXE)) {
        if (tx_empty()) {
            usart_disable_tx_interrupt(BT_USART); /* nada más para enviar */
        } else {
            usart_send(BT_USART, g_tx[g_tx_r]);
            g_tx_r = (g_tx_r + 1) & RB_MASK_TX;
        }
    }
}
