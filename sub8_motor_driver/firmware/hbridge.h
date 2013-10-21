#ifndef SUBJUGATOR_SUB8_MOTOR_DRIVER_FIRMWARE_HBRIDGE_H
#define SUBJUGATOR_SUB8_MOTOR_DRIVER_FIRMWARE_HBRIDGE_H


void hbridge_init();
void hbridge_set_duty_cycle(float duty_cycle);
void hbridge_set_floating(bool floating);


#endif
