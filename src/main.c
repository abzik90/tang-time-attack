#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

volatile uint32_t start_time = 0;
volatile uint32_t elapsed_time = 0;
volatile bool timer_running = false;
volatile uint32_t counter = 0;
// TODO:
/*
    Write a function that goes through the pin codes 0-9, 00-99, etc.
*/

void gpio_setup(void) {
    // Enable GPIOA clock
    rcc_periph_clock_enable(RCC_GPIOA);
    
    // Configure GPIOA0 and GPIOA1 as input pins for EXTI
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    
    // Configure PA9 and PA10 for USART1 (alternate function)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
    
    // Configure PA2 and PA3 for USART2 (alternate function)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
}

void usart_setup(void) {
    // Enable clocks for USART1 and USART2
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);
    
    // Configure USART1: 115200 baud, 8 data bits, no parity, 1 stop bit
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
    nvic_enable_irq(NVIC_USART1_IRQ);
    usart_enable_rx_interrupt(USART1);

    // Configure USART2: 115200 baud, 8 data bits, no parity, 1 stop bit
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);
    usart_enable_rx_interrupt(USART2);
}

void timer_setup(void) {
    // Enable Timer 2 clock
    rcc_periph_clock_enable(RCC_TIM2);
    
    // Reset Timer 2 settings
    rcc_periph_reset_pulse(RST_TIM2);
    
    // Set the prescaler value to get a 10 MHz timer clock (0.1 µs resolution)
    timer_set_prescaler(TIM2, rcc_apb1_frequency / 10000000 - 1);
    
    // Set the timer mode to up-counting
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    
    // Enable auto-reload preload
    timer_enable_preload(TIM2);
    
    // Set auto-reload value to the maximum (32-bit timer)
    timer_set_period(TIM2, 0xFFFFFFFF);
    
    // Enable Timer 2 counter
    timer_enable_counter(TIM2);
}

void exti_setup(void) {
    // Enable clock for system configuration controller (needed for EXTI)
    rcc_periph_clock_enable(RCC_SYSCFG);
    
    // Configure EXTI0 for GPIOA0 (trigger A)
    exti_select_source(EXTI0, GPIOA);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI0);
    
    // Configure EXTI1 for GPIOA1 (trigger B)
    exti_select_source(EXTI1, GPIOA);
    exti_set_trigger(EXTI1, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI1);
    
    // Enable EXTI interrupts in the NVIC
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    nvic_enable_irq(NVIC_EXTI1_IRQ);
}

void exti0_isr(void) {
    // Clear the interrupt flag
    exti_reset_request(EXTI0);
    
    // Start or restart the timer
    timer_set_counter(TIM2, 0);
    start_time = 0;
    timer_running = true;
    ++counter;
    usart_send_string("Interrupt triggered\n", USART2);
}

void exti1_isr(void) {
    // Clear the interrupt flag
    exti_reset_request(EXTI1);
    
    if (timer_running) {
        // Stop the timer and calculate elapsed time in microseconds
        elapsed_time = timer_get_counter(TIM2);
        timer_running = false;
        
        // Output the duration via USART1
        // output_duration_usart1(elapsed_time);
    }
}

void output_duration_usart1(uint32_t duration) {
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "Duration: %lu us\r\n", duration);
    
    // Send the duration string via USART1
    for (int i = 0; i < len; i++) {
        usart_send_blocking(USART1, buffer[i]);
    }
}

/* USART1 interrupt service routine. */
void usart1_isr(void){
    if (usart_get_flag(USART1, USART_SR_RXNE)) {
        /* Read received character. */
        char received = usart_recv(USART1);
    
        /* Echo received character back. */
        usart_send_blocking(USART1, received);
        // usart_send_string("Hello World!\n", USART1);
    }
}

/* USART2 interrupt service routine. */
void usart2_isr(void){
    if (usart_get_flag(USART2, USART_SR_RXNE)) {
        /* Read received character. */
        char received = usart_recv(USART2);

        /* Echo received character back. */
        // usart_send_blocking(USART2, received);
    }
}

/* Simple function to send a string via USART2. */
void usart_send_string(const char *str, uint32_t uart_addr)
{
    while (*str) {
        usart_send_blocking(uart_addr, *str++);
    }
}

int main(void) {
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    // Setup peripherals
    gpio_setup();
    usart_setup();
    timer_setup();
    exti_setup();
    usart_send_string("Hello world!\n", USART1);
    usart_send_string("From the 2nd USART too\n!", USART2);
    // Main loop does nothing; interrupts handle the timing and UART output
    while (1) {
        __asm__("nop");
    }
}
