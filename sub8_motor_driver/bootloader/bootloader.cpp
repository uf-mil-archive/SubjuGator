#include <cstring>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include <uf_subbus_protocol/sha256.h>
#include <uf_subbus_protocol/protocol.h>
#include <arm_bootloader/uniqueid.h>

#include <arm_bootloader/handler.cpp>


void usart_setup(void) {
  /* Enable clocks for GPIO port B (for GPIO_USART1_TX) and USART1. */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);

  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
  gpio_clear(GPIOB, GPIO5);

  AFIO_MAPR |= AFIO_MAPR_USART1_REMAP;
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_RE_TX);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RE_RX);


  /* Setup UART parameters. */
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART1);
}

class UARTSink : public uf_subbus_protocol::ISink {
public:
  void handleStart() {
    gpio_set(GPIOB, GPIO5);
  }
  void handleByte(uint8_t byte) {
    usart_send_blocking(USART1, byte);
  }
  void handleEnd() {
    // make sure write finishes
    usart_send_blocking(USART1, 0);
    usart_wait_send_ready(USART1);
    
    gpio_clear(GPIOB, GPIO5);
  }
};

int main() {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  usart_setup();
  
  UARTSink sink;
  arm_bootloader::Handler handler(sink, arm_bootloader::get_unique_dest(), 1024);
  
  while(true) {
    handler.handleByte(usart_recv_blocking(USART1));
  }
}
