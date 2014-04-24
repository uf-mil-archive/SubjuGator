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

#include <sub8_motor_driver/protocol.h>
#include "hbridge.h"
#include "temperature.h"
#include "time.h"

using namespace sub8_motor_driver;


class Handler {
  class GotMessageFunctor {
    Handler &handler;
  public:
    GotMessageFunctor(Handler &handler) :
      handler(handler) {
    }
    void operator()(const Command &command) {
      handler.handle_message(command);
    }
  };
  
  GotMessageFunctor gmf;
  uf_subbus_protocol::SimpleReceiver<Command, GotMessageFunctor> receiver;
  uf_subbus_protocol::SimpleSender<Response, uf_subbus_protocol::ISink> sender;
  arm_bootloader::Dest dest;

public:
  Handler(uf_subbus_protocol::ISink &sink) :
    gmf(*this), receiver(gmf), sender(sink), dest(arm_bootloader::get_unique_dest()) {
  }
  
  void handleByte(uint8_t byte) {
    receiver.handleRawByte(byte);
  }
  
  void handle_message(const Command &msg) {
    if(msg.dest != dest) return;
    
    Response resp; memset(&resp, 0, sizeof(resp));
    resp.id = msg.id;
    
    switch(msg.command) {
    
      case CommandID::Reset: {
        // action happens later, after response is sent
      } break;
    
      case CommandID::GetStatus: {
        resp.resp.GetStatus.magic = GetStatusResponse::MAGIC_VALUE;
        resp.resp.GetStatus.uptime_ns = time_get_ns();
      } break;
      
      case CommandID::SetMotorDutyCycle: {
        hbridge_set_duty_cycle(msg.args.SetMotorDutyCycle.duty_cycle);
      } break;
      
      case CommandID::GetTemperatures: {
        temperature_read(resp.resp.GetTemperatures);
      } break;
      
      case CommandID::GetCurrentLog: {
        current_read(resp.resp.GetCurrentLog);
      } break;
      
      default: {
        return; // send nothing back if command is invalid
      } break;

    }
    
    if(resp.id) {
      sender.write_object(resp);
    }

    switch(msg.command) {
      case CommandID::Reset: {
        scb_reset_system();
      } break;
      
      default: {
      } break;
    }
  }
};


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
  hbridge_init();
  temperature_setup();
  time_init();
  
  UARTSink sink;
  Handler handler(sink);
  
  while(true) {
    handler.handleByte(usart_recv_blocking(USART1));
  }
}
