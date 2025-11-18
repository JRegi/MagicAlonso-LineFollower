#include "SerialBT.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ===== SysTick millis ===== */
static volatile uint32_t _ms = 0;
void sys_tick_handler(void){ _ms++; }
static inline uint32_t millis(void){ return _ms; }
static inline void delay_ms(uint32_t ms){ uint32_t t=millis(); while((millis()-t)<ms) __asm__("nop"); }

/* ===== Estado global (única instancia, simple) ===== */
static volatile uint8_t  _rxbuf[SERIALBT_RX_BUFSZ];
static volatile uint16_t _head=0, _tail=0;
static uint32_t          _baud_data = 9600;
static uint32_t          _timeout_ms = 1000;

static inline uint16_t _nxt(uint16_t x){ return (uint16_t)((x+1) % SERIALBT_RX_BUFSZ); }
static inline int _empty(void){ return _head==_tail; }
static inline int _full (void){ return _nxt(_head)==_tail; }

/* ===== Board init por defecto ===== */
void board_clock_setup_72mhz(void){
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(rcc_ahb_frequency/1000 - 1);
    systick_counter_enable();
    systick_interrupt_enable();
}
void board_usart_pins_setup_default(void){
    /* TX */
    if(SERIALBT_TX_PORT==GPIOA) rcc_periph_clock_enable(RCC_GPIOA);
    if(SERIALBT_TX_PORT==GPIOB) rcc_periph_clock_enable(RCC_GPIOB);
    if(SERIALBT_TX_PORT==GPIOC) rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(SERIALBT_TX_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, SERIALBT_TX_PIN);
    /* RX */
    if(SERIALBT_RX_PORT==GPIOA) rcc_periph_clock_enable(RCC_GPIOA);
    if(SERIALBT_RX_PORT==GPIOB) rcc_periph_clock_enable(RCC_GPIOB);
    if(SERIALBT_RX_PORT==GPIOC) rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(SERIALBT_RX_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, SERIALBT_RX_PIN);
    /* KEY opcional */
#if SERIALBT_KEY_PORT != 0
    if(SERIALBT_KEY_PORT==GPIOA) rcc_periph_clock_enable(RCC_GPIOA);
    if(SERIALBT_KEY_PORT==GPIOB) rcc_periph_clock_enable(RCC_GPIOB);
    if(SERIALBT_KEY_PORT==GPIOC) rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(SERIALBT_KEY_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, SERIALBT_KEY_PIN);
    gpio_clear(SERIALBT_KEY_PORT, SERIALBT_KEY_PIN); /* bajo = datos */
#endif
}

/* ===== USART helpers ===== */
static inline void _usart_enable(uint32_t baud){
    if(SERIALBT_USART==USART1) rcc_periph_clock_enable(RCC_USART1);
    if(SERIALBT_USART==USART2) rcc_periph_clock_enable(RCC_USART2);
    if(SERIALBT_USART==USART3) rcc_periph_clock_enable(RCC_USART3);

    usart_set_baudrate(SERIALBT_USART, baud);
    usart_set_databits(SERIALBT_USART, 8);
    usart_set_stopbits(SERIALBT_USART, USART_STOPBITS_1);
    usart_set_parity(SERIALBT_USART, USART_PARITY_NONE);
    usart_set_flow_control(SERIALBT_USART, USART_FLOWCONTROL_NONE);
    usart_set_mode(SERIALBT_USART, USART_MODE_TX_RX);
    usart_enable(SERIALBT_USART);
}
static inline void _usart_set_baud(uint32_t baud){
    usart_disable(SERIALBT_USART);
    usart_set_baudrate(SERIALBT_USART, baud);
    usart_enable(SERIALBT_USART);
}

/* ===== API ===== */
void SerialBT_begin(uint32_t baud){
    _baud_data = baud ? baud : 9600;
    _usart_enable(_baud_data);
    usart_enable_rx_interrupt(SERIALBT_USART);

    if(SERIALBT_USART==USART1) nvic_enable_irq(NVIC_USART1_IRQ);
    if(SERIALBT_USART==USART2) nvic_enable_irq(NVIC_USART2_IRQ);
    if(SERIALBT_USART==USART3) nvic_enable_irq(NVIC_USART3_IRQ);
}
void SerialBT_end(void){
    if(SERIALBT_USART==USART1) nvic_disable_irq(NVIC_USART1_IRQ);
    if(SERIALBT_USART==USART2) nvic_disable_irq(NVIC_USART2_IRQ);
    if(SERIALBT_USART==USART3) nvic_disable_irq(NVIC_USART3_IRQ);
    usart_disable_rx_interrupt(SERIALBT_USART);
    usart_disable(SERIALBT_USART);
}

/* ISR comunes (1 instancia global, simple) */
void usart1_isr(void){
    if(usart_get_flag(USART1, USART_SR_RXNE)){
        uint8_t b = (uint8_t)usart_recv(USART1);
        if(!_full()){ _rxbuf[_head]=b; _head=_nxt(_head); }
    }
}
void usart2_isr(void){
    if(usart_get_flag(USART2, USART_SR_RXNE)){
        uint8_t b = (uint8_t)usart_recv(USART2);
        if(!_full()){ _rxbuf[_head]=b; _head=_nxt(_head); }
    }
}
void usart3_isr(void){
    if(usart_get_flag(USART3, USART_SR_RXNE)){
        uint8_t b = (uint8_t)usart_recv(USART3);
        if(!_full()){ _rxbuf[_head]=b; _head=_nxt(_head); }
    }
}

/* Disponibilidad y lectura */
int SerialBT_available(void){
    int d = (int)_head - (int)_tail;
    if(d<0) d += SERIALBT_RX_BUFSZ;
    return d;
}
int SerialBT_peek(void){ if(_empty()) return -1; return _rxbuf[_tail]; }
int SerialBT_read(void){
    if(_empty()) return -1;
    uint8_t b = _rxbuf[_tail];
    _tail = _nxt(_tail);
    return b;
}
void SerialBT_setTimeout(uint32_t ms){ _timeout_ms = ms; }
size_t SerialBT_readBytes(uint8_t *buf, size_t len){
    uint32_t t0 = millis(); size_t n=0;
    while(n<len){
        int c = SerialBT_read();
        if(c>=0){ buf[n++]=(uint8_t)c; t0 = millis(); }
        else if((millis()-t0)>=_timeout_ms) break;
    }
    return n;
}
int SerialBT_readBytesUntil(char term, char *buf, size_t len){
    if(len==0) return 0;
    size_t n=0; uint32_t t0=millis();
    while(n<(len-1)){
        int c = SerialBT_read();
        if(c>=0){
            buf[n++] = (char)c;
            if((char)c == term) break;
            t0 = millis();
        }else if((millis()-t0)>=_timeout_ms) break;
    }
    buf[n] = '\0';
    return (int)n;
}

/* Escritura */
size_t SerialBT_write(const uint8_t *data, size_t len){
    for(size_t i=0;i<len;i++) usart_send_blocking(SERIALBT_USART, data[i]);
    return len;
}
int SerialBT_print(const char *s){ if(!s) return 0; size_t n=strlen(s); SerialBT_write((const uint8_t*)s,n); return (int)n; }
int SerialBT_println(const char *s){ int n=SerialBT_print(s); SerialBT_write((const uint8_t*)"\r\n",2); return n+2; }
int SerialBT_printf(const char *fmt, ...){
    char tmp[128];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(tmp,sizeof(tmp),fmt,ap);
    va_end(ap);
    if(n<=0) return n;
    size_t to_send = (n<(int)sizeof(tmp)) ? (size_t)n : sizeof(tmp)-1;
    SerialBT_write((const uint8_t*)tmp,to_send);
    return (int)to_send;
}

/* ===== AT: programar nombre/PIN/baud con KEY por defecto ===== */
static bool _at_try_baud(uint32_t baud){
    _usart_set_baud(baud);
    const char *cmd="AT\r\n"; for(const char *p=cmd; *p; ++p) usart_send_blocking(SERIALBT_USART, (uint8_t)*p);
    char r[16]; int n=0; uint32_t t0=millis();
    while((millis()-t0)<200 && n<(int)(sizeof(r)-1)){
        if(usart_get_flag(SERIALBT_USART, USART_SR_RXNE)){
            r[n++] = (char)usart_recv(SERIALBT_USART);
            if(n>=2 && r[n-2]=='\r' && r[n-1]=='\n') break;
        }
    }
    r[n]=0;
    return (strstr(r,"OK")!=NULL);
}
static bool _at_cmd(const char *cmd, char *resp, size_t maxlen, uint32_t to_ms){
    for(const char *p=cmd; *p; ++p) usart_send_blocking(SERIALBT_USART,(uint8_t)*p);
    size_t n=0; uint32_t t0=millis();
    while((millis()-t0)<to_ms && n<(maxlen-1)){
        if(usart_get_flag(SERIALBT_USART, USART_SR_RXNE)){
            resp[n++] = (char)usart_recv(SERIALBT_USART);
            if(n>=2 && resp[n-2]=='\r' && resp[n-1]=='\n') break;
        }
    }
    resp[n]=0; return (n>0);
}
bool SerialBT_program(const char *name, const char *pin4, uint32_t new_baud){
#if SERIALBT_KEY_PORT = 0
    (void)name; (void)pin4; (void)new_baud;
    return false; /* Sin KEY no podemos entrar a AT de forma confiable */
#else
    /* KEY alto para AT “completo” (firmware típico a 38400) */
    gpio_set(SERIALBT_KEY_PORT, SERIALBT_KEY_PIN);
    delay_ms(50);

    bool ok = _at_try_baud(38400) || _at_try_baud(9600);
    if(!ok){
        /* salir de AT y restaurar */
        gpio_clear(SERIALBT_KEY_PORT, SERIALBT_KEY_PIN);
        delay_ms(20);
        _usart_set_baud(_baud_data);
        return false;
    }

    char r[48], cmd[48];
    if(name){
        snprintf(cmd,sizeof(cmd),"AT+NAME=%s\r\n",name);
        if(!_at_cmd(cmd,r,sizeof(r),300) || !strstr(r,"OK")) ok=false;
    }
    if(pin4){
        snprintf(cmd,sizeof(cmd),"AT+PSWD=%s\r\n",pin4); // algunos usan AT+PIN=
        if(!_at_cmd(cmd,r,sizeof(r),300) || !strstr(r,"OK")) ok=false;
    }
    if(new_baud){
        snprintf(cmd,sizeof(cmd),"AT+UART=%lu,0,0\r\n",(unsigned long)new_baud);
        if(!_at_cmd(cmd,r,sizeof(r),400) || !strstr(r,"OK")) ok=false;
        else _baud_data = new_baud;
    }

    /* Salir a datos y fijar nuestro USART al baud final */
    gpio_clear(SERIALBT_KEY_PORT, SERIALBT_KEY_PIN);
    delay_ms(20);
    _usart_set_baud(_baud_data);
    return ok;
#endif
}
