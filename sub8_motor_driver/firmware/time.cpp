#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>

static uint64_t rollovers;

void time_init() {
  systick_set_reload(0xffffff);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  rollovers = 0;
  systick_interrupt_enable();
  systick_counter_enable();
}

bool check_rollover() {
  if(systick_get_countflag()) {
    rollovers++;
    return true;
  }
  return false;
}

extern "C" {

void sys_tick_handler(void) {
  cm_disable_interrupts();
  check_rollover();
  cm_enable_interrupts();
}

}

uint64_t time_get_ns() {
  uint64_t result;
  // keep calculating result until it is calculated without a rollover having
  // happened
  cm_disable_interrupts();
  do {
    result = (rollovers << 24) | (0xffffff - systick_get_value());
  } while(check_rollover());
  cm_enable_interrupts();
  return result*125/9;
}
