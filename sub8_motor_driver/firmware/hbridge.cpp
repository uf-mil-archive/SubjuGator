#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "hbridge.h"


static const double cpu_frequency = 7.2e6; // MHz

static const double pwm_frequency = 100;
static const double period = 1 / pwm_frequency;
static const double half_period = period / 2;
static const uint32_t half_period_ticks = cpu_frequency * half_period + 0.5; 
static_assert(half_period_ticks <= 65535, "half_period_ticks too large for counter!");

static const double dead_time = 500e-9 + 216e-9; // 500 ns + 216 ns
static const double half_dead_time = dead_time / 2;
static const uint32_t half_dead_time_ticks = cpu_frequency * half_dead_time + 0.5;


void hbridge_init() {
  // M- bridge
  // A9 - pin 21 - PWM2A - HIN
  // B0 - pin 15 - PWM2B - \LIN
  
  // M+ bridge
  // A8 - pin 20 - PWM1A - HIN
  // A7 - pin 14 - PWM1B - \LIN
  
  rcc_peripheral_enable_clock(&RCC_APB2ENR,
    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
    RCC_APB2ENR_AFIOEN | RCC_APB2ENR_TIM1EN);
  
  AFIO_MAPR |= AFIO_MAPR_TIM1_REMAP_PARTIAL_REMAP;
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO8 | GPIO9);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0);
  
  timer_reset(TIM1);
  timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                 TIM_CR1_DIR_UP);
  timer_set_period(TIM1, half_period_ticks);
  timer_set_prescaler(TIM1, 9); // / 10
  
  timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
  timer_set_oc_polarity_high(TIM1, TIM_OC1);
  timer_set_oc_polarity_low(TIM1, TIM_OC1N);
  timer_enable_oc_output(TIM1, TIM_OC1);
  timer_enable_oc_output(TIM1, TIM_OC1N);
  timer_set_oc_value(TIM1, TIM_OC1, half_period_ticks);
  
  timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);
  timer_set_oc_polarity_high(TIM1, TIM_OC2);
  timer_set_oc_polarity_low(TIM1, TIM_OC2N);
  timer_enable_oc_output(TIM1, TIM_OC2);
  timer_enable_oc_output(TIM1, TIM_OC2N);
  timer_set_oc_value(TIM1, TIM_OC2, half_period_ticks);
  
  timer_enable_break_main_output(TIM1);
  timer_enable_counter(TIM1);
}

static double lerp(double x, double a, double b) {
  return (1-x)*a + (x)*b;
}

void hbridge_set_duty_cycle(float duty_cycle) {
  if(duty_cycle > 0) {
    if(duty_cycle > 1) duty_cycle = 1;
    timer_set_oc_value(TIM1, TIM_OC2, half_period_ticks);
    // 0% -> half_period_ticks - dead_time_ticks
    // 100% -> dead_time_ticks
    double x = lerp(duty_cycle, half_period_ticks - half_dead_time_ticks, half_dead_time_ticks);
    //50 + (1 - duty_cycle)*660
    timer_set_oc_value(TIM1, TIM_OC1, (uint32_t)(x + 0.5));
  } else if(duty_cycle < 0) {
    if(duty_cycle < -1) duty_cycle = -1;
    timer_set_oc_value(TIM1, TIM_OC1, half_period_ticks);
    double x = lerp(-duty_cycle, half_period_ticks - half_dead_time_ticks, half_dead_time_ticks);
    timer_set_oc_value(TIM1, TIM_OC2, (uint32_t)(x + 0.5));
  } else {
    timer_set_oc_value(TIM1, TIM_OC1, half_period_ticks);
    timer_set_oc_value(TIM1, TIM_OC2, half_period_ticks);
  }
}

void hbridge_set_floating(bool floating) {
  if(floating) {
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO7 | GPIO8);
  } else {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO8);
  }
}
