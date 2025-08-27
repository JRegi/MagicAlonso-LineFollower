#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>

#define LED1_PORT GPIOB
#define LED2_PORT GPIOB
#define LED3_PORT GPIOB
#define BUZ_PORT  GPIOB

#define LED1_PIN  GPIO4   // PB4 (liberado quitando JTAG, mantenemos SWD)
#define LED2_PIN  GPIO8   // PB8
#define LED3_PIN  GPIO5   // PB5
#define BUZ_PIN   GPIO9   // PB9 (buzzer activo por MOSFET low-side: ALTO = ON)

#define T_LED_MS  500U
#define T_BUZ_MS  250U

static volatile uint32_t ms_ticks = 0;

/* SysTick 1 ms */
void sys_tick_handler(void) { ms_ticks++; }
static void delay_ms(uint32_t ms) {
    uint32_t t0 = ms_ticks;
    while ((ms_ticks - t0) < ms) { __asm__("nop"); }
}

static void clock_setup(void) {
    /* 72 MHz desde HSE 8 MHz (Bluepill) */
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

static void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);    // 72 MHz
    systick_set_reload(72000 - 1);                     // 1 ms
    systick_clear();
    systick_counter_enable();
    systick_interrupt_enable();
}

static void gpio_setup(void) {
    /* Habilitar AFIO y liberar PB3/PB4 del JTAG (dejando SWD en PA13/PA14) */
    rcc_periph_clock_enable(RCC_AFIO);
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);

    /* Salidas en PB4, PB5, PB8, PB9 */
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  LED1_PIN | LED2_PIN | LED3_PIN | BUZ_PIN);

    /* Estado inicial:
       - LEDs “activo en bajo” (LED -> R -> 3V3, el pin hunde a GND para encender) → APAGADOS = HIGH
       - Buzzer con MOSFET low-side → APAGADO = LOW
    */
    gpio_set(LED1_PORT, LED1_PIN);   // LED1 OFF
    gpio_set(LED2_PORT, LED2_PIN);   // LED2 OFF
    gpio_set(LED3_PORT, LED3_PIN);   // LED3 OFF
    gpio_clear(BUZ_PORT, BUZ_PIN);   // Buzzer OFF
}

int main(void) {
    clock_setup();
    systick_setup();
    gpio_setup();

    for (;;) {
        /* Encender en orden: PB4 -> PB8 -> PB5 (500 ms entre cada uno) */
        gpio_clear(LED1_PORT, LED1_PIN);  // LED1 ON (activo-bajo)
        delay_ms(T_LED_MS);

        gpio_clear(LED2_PORT, LED2_PIN);  // LED2 ON
        delay_ms(T_LED_MS);

        gpio_clear(LED3_PORT, LED3_PIN);  // LED3 ON
        delay_ms(T_LED_MS);

        /* Apagar todos juntos 500 ms y, dentro de esa ventana, buzzer ON 250 ms */
        gpio_set(LED1_PORT, LED1_PIN);    // LEDs OFF
        gpio_set(LED2_PORT, LED2_PIN);
        gpio_set(LED3_PORT, LED3_PIN);

        gpio_set(BUZ_PORT, BUZ_PIN);      // Buzzer ON (activo-alto por MOSFET)
        delay_ms(T_BUZ_MS);
        gpio_clear(BUZ_PORT, BUZ_PIN);    // Buzzer OFF

        delay_ms(T_LED_MS - T_BUZ_MS);    // completar los 500 ms de “pausa”
    }
}
