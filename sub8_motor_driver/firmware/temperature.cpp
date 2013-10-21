#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "temperature.h"

#include "hbridge.h"
#include "time.h"

using namespace sub8_motor_driver;

/*
ADC0 - M+ low temperature
ADC1 - motor current into M+ of motor
ADC2 - M+ voltage (divided)
ADC3 - M- voltage (divided)
ADC4 - Vin voltage (divided)
ADC5 - M- high temperature
ADC6 - M+ high temperature
ADC9 - M- low temperature
*/

void temperature_setup() {
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);

  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO2);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO3);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO4);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO5);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO6);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
   
  /* Make sure the ADC doesn't run during config. */
  adc_off(ADC1);

  /* We configure everything for one single conversion. */
  adc_disable_scan_mode(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_right_aligned(ADC1);
  adc_enable_dma(ADC1);
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
  adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);

  adc_power_on(ADC1);

  /* Wait for ADC starting up. */
  for (int i = 0; i < 800000; i++) /* Wait a bit. */
          __asm__("nop");

  adc_reset_calibration(ADC1);
  adc_calibration(ADC1);
   
  /* Make sure the ADC doesn't run during config. */
  adc_off(ADC2);

  /* We configure everything for one single conversion. */
  adc_disable_scan_mode(ADC2);
  adc_set_single_conversion_mode(ADC2);
  adc_disable_external_trigger_regular(ADC2);
  adc_set_right_aligned(ADC2);
  adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_28DOT5CYC);
  adc_enable_external_trigger_regular(ADC2, ADC_CR2_EXTSEL_SWSTART);

  adc_power_on(ADC2);

  /* Wait for ADC starting up. */
  for (int i = 0; i < 800000; i++) /* Wait a bit. */
          __asm__("nop");

  adc_reset_calibration(ADC2);
  adc_calibration(ADC2);
}

static void start_read_adc_naiive(uint8_t channel) {
  adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
  uint8_t channel_array[16];
  channel_array[0] = channel;
  adc_set_regular_sequence(ADC1, 1, channel_array);
  adc_start_conversion_regular(ADC1);
}

static uint16_t finish_read_adc_naiive() {
  while (!adc_eoc(ADC1));
  uint16_t reg16 = adc_read_regular(ADC1);
  return reg16;
}

static void start_read_adc_dual(uint8_t channel1, uint8_t channel2) {
  adc_set_dual_mode(ADC_CR1_DUALMOD_RSM);
  
  uint8_t channel_array1[16];
  channel_array1[0] = channel1;
  adc_set_regular_sequence(ADC1, 1, channel_array1);
  
  uint8_t channel_array2[16];
  channel_array2[0] = channel2;
  adc_set_regular_sequence(ADC2, 1, channel_array2);
  
  adc_start_conversion_regular(ADC1);
}

static void finish_read_adc_dual(uint16_t &result1, uint16_t &result2) {
  while (!adc_eoc(ADC1));
  uint32_t result = adc_read_regular(ADC1);
  result1 = result;
  result2 = result >> 16;
}

void temperature_read(GetTemperaturesResponse &resp) {
  for(int ch = 0; ch <= 9; ch++) {
    start_read_adc_naiive(ch);
    resp.voltage[ch] = finish_read_adc_naiive();
  }
}

void current_read(GetCurrentLogResponse &resp) {
  static_assert(sizeof(resp.current)/sizeof(resp.current[0]) ==
    sizeof(resp.voltage)/sizeof(resp.voltage[0]), "result lengths don't match");
  static_assert(sizeof(resp.current)/sizeof(resp.current[0]) ==
    sizeof(resp.time)/sizeof(resp.time[0]), "result lengths don't match");
  
  for(unsigned int i = 0; i < sizeof(resp.current)/sizeof(resp.current[0]); i++) {
    start_read_adc_dual(1, 2);
    resp.time[i] = time_get_ns();
    //hbridge_set_floating(i > sizeof(resp.current)/sizeof(resp.current[0])/10);
    finish_read_adc_dual(resp.current[i], resp.voltage[i]);
  }
  hbridge_set_floating(false);
}
