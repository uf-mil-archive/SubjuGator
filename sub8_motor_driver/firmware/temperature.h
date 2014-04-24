#ifndef SUBJUGATOR_SUB8_MOTOR_DRIVER_FIRMWARE_TEMPERATURE_H
#define SUBJUGATOR_SUB8_MOTOR_DRIVER_FIRMWARE_TEMPERATURE_H

#include <sub8_motor_driver/protocol.h>

void temperature_setup();
void temperature_read(sub8_motor_driver::GetTemperaturesResponse &resp);
void current_read(sub8_motor_driver::GetCurrentLogResponse &resp);

#endif
