#pragma once

#include <stdint.h>

namespace commands {

#pragma pack(push, 1)
struct _base {
  uint8_t id;

  explicit _base(uint8_t id) : id(id) {}

  const uint8_t* begin() const {
    return reinterpret_cast<const uint8_t*>(this);
  }
};

struct set_pwm : public _base {
  static const uint8_t ID = 1;

  set_pwm() : _base(ID), value(0){};

  int8_t value;
};

struct reply_status : public _base {
  static const uint8_t ID = 2;
  reply_status() : _base(ID){};

  uint16_t tempFET;
  uint32_t ampsMotor;
  uint32_t ampsInput;
  uint16_t dutyCycle;
  uint32_t rpm;
  uint16_t voltInput;
  uint32_t tachometer;
  uint32_t tachometerAbs;
};
#pragma pack(pop)

}  // namespace commands
