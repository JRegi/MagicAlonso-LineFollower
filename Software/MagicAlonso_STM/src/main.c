#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>

#define LED_PORT GPIOC
#define LED_PIN  GPIO13        // LED del Bluepill en PC13 (activo en bajo)
#define T_MS     500           // periodo de parpadeo en milisegundos

static volatile uint32_t ms_ticks = 0;

/* ISR de SysTick: se llama cada 1 ms */
void sys_tick_handler(void) {
    ms_ticks++;
}

/* Espera en milisegundos (wrap-safe) */
static void delay_ms(uint32_t ms) {
    uint32_t t0 = ms_ticks;
    while ((ms_ticks - t0) < ms) {
        __asm__("nop");
    }
}

static void clock_setup(void) {
    /* 72 MHz desde HSE de 8 MHz (cristal del Bluepill) */
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

static void systick_setup(void) {
    /* SysTick a 1 kHz (1 ms): fuente AHB = 72 MHz → reload = 72000-1 */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(72000 - 1);
    systick_clear();
    systick_counter_enable();
    systick_interrupt_enable();
}

static void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOC);
    /* PC13 salida push-pull 2 MHz (suficiente y con menos EMI) */
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
    gpio_set(LED_PORT, LED_PIN);   // apagado (activo en bajo)
}

int main(void) {
    clock_setup();
    systick_setup();
    gpio_setup();

    while (1) {
        /* Encender (activo en bajo), esperar 500 ms */
        gpio_clear(LED_PORT, LED_PIN);
        delay_ms(T_MS);

        /* Apagar, esperar 500 ms */
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(T_MS);
    }
}
