#include <cstring>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "sha256.h"
#include <arm_bootloader/subbus_protocol.h>
#include "uniqueid.h"

#include "protocol.h"
#include "hbridge.h"
#include "temperature.h"
#include "time.h"

using namespace sub8_motor_driver;

Dest dest = arm_bootloader::get_unique_dest();
arm_bootloader::ChecksumAdder<arm_bootloader::Packetizer<void (*)(uint8_t byte)> > *p_checksumadder;

void messageReceived(const Command &msg) {
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
    gpio_set(GPIOB, GPIO5);
    write_object(resp, *p_checksumadder);
    // make sure write finishes
    usart_send_blocking(USART1, 0);
    usart_wait_send_ready(USART1);
    //while ((USART_SR(USART1) & USART_SR_TC) == 0);
    gpio_clear(GPIOB, GPIO5);
  }
  
  switch(msg.command) {
    case CommandID::Reset: {
      // make sure write finishes
      usart_send_blocking(USART1, 0);
      usart_wait_send_ready(USART1);
      
      scb_reset_system();
    } break;
    
    default: {
    } break;
  }
}



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

void write_byte(uint8_t byte) {
  usart_send_blocking(USART1, byte);
}

uint8_t read_byte() {
  return usart_recv_blocking(USART1);
}

int main() {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  usart_setup();
  hbridge_init();
  temperature_setup();
  time_init();
  
  arm_bootloader::Packetizer<void (*)(uint8_t byte)>
    packetizer(write_byte);
  arm_bootloader::ChecksumAdder<arm_bootloader::Packetizer<void (*)(uint8_t byte)> >
    checksumadder(packetizer);
  p_checksumadder = &checksumadder;
  
  arm_bootloader::ObjectReceiver<Command, void(const Command &)>
    objectreceiver(messageReceived);
  arm_bootloader::ChecksumChecker<arm_bootloader::ObjectReceiver<Command, void(const Command &)> >
    cc(objectreceiver);
  arm_bootloader::Depacketizer<arm_bootloader::ChecksumChecker<arm_bootloader::ObjectReceiver<Command, void(const Command &)> > >
    depacketizer(cc);
  
  while(true) {
    depacketizer.handleRawByte(read_byte());
  }
}



extern "C" {

void _exit(int) {
  while(true);
}

void _kill(int) {
  ;
}

int _getpid() {
  return 0;
}

}
