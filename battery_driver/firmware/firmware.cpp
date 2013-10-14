#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/gpio.h>

static void delay() {
  volatile int i = 0;
  while (i++ < static_cast<int>(1e6)) {
    asm ("nop");
  }
}

int main() {
  // Turn on some peripheral clocks
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);

  // put CLK_EN high to start the 8Mhz oscillator attached to HSE
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_set(GPIOC, GPIO15);

  // Switch clock to use HSE
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  
  // Blink an LED
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  while (true) {
    delay();
    gpio_toggle(GPIOB, GPIO2);
    delay();
    gpio_toggle(GPIOC, GPIO13);
  }
}

// early thoughts: Port quadcopter script which runs st-util and gdb together and reloads firmware

extern "C"
void _exit(int) {
  while(true);
}

extern "C"
void _kill(int) { }

extern "C"
int _getpid() {
  return 0;
}


