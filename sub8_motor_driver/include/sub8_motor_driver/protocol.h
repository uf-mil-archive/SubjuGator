#ifndef ARM_TEST_STM32F3DISCOVERY_IMU_DRIVER_FIRMWARE_PROTOCOL_H
#define ARM_TEST_STM32F3DISCOVERY_IMU_DRIVER_FIRMWARE_PROTOCOL_H

#include <stdint.h>

namespace sub8_motor_driver {


enum class CommandID : uint16_t {
  // commands that everything should implement
  Reset = 0,
  GetStatus,
  
  // commands specific to this firmware
  SetMotorDutyCycle = 0x72cf,
  GetTemperatures,
  GetCurrentLog,
};


struct __attribute__((packed)) ResetCommand {
};
struct __attribute__((packed)) ResetResponse {
};

struct __attribute__((packed)) GetStatusCommand {
};
struct __attribute__((packed)) GetStatusResponse {
  uint64_t magic; static uint64_t const MAGIC_VALUE = 0x9a206b613491430e;
  uint64_t uptime_ns;
};


struct __attribute__((packed)) SetMotorDutyCycleCommand {
  float duty_cycle;
};
struct __attribute__((packed)) SetMotorDutyCycleResponse {
};


struct __attribute__((packed)) GetTemperaturesCommand {
};
struct __attribute__((packed)) GetTemperaturesResponse {
  float voltage[10];
};


struct __attribute__((packed)) GetCurrentLogCommand {
};
struct __attribute__((packed)) GetCurrentLogResponse {
  uint64_t time[800];
  uint16_t current[800];
  uint16_t voltage[800];
};


typedef uint32_t Dest;
typedef uint16_t ID;

struct __attribute__((packed)) Command {
  Dest dest;
  ID id; // 0 means don't send a response
  CommandID command;
  union {
    ResetCommand Reset;
    GetStatusCommand GetStatus;
    SetMotorDutyCycleCommand SetMotorDutyCycle;
    GetTemperaturesCommand GetTemperatures;
    GetCurrentLogCommand GetCurrentLog;
  } args;
};

struct __attribute__((packed)) Response {
  ID id;
  union {
    ResetResponse Reset;
    GetStatusResponse GetStatus;
    SetMotorDutyCycleResponse SetMotorDutyCycle;
    GetTemperaturesResponse GetTemperatures;
    GetCurrentLogResponse GetCurrentLog;
  } resp;
};


}

#endif
