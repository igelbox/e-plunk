#include <RH_NRF24.h>
#include "../lib/commands.hpp"
#include "../lib/io.hpp"
#include "../lib/panic.hpp"

using namespace panic;

#define PIN_PANIC 4

#define PIN_NRF_CE 9
#define PIN_NRF_SC 10
// #define PIN_NRF_.. 11, 12, 13

RH_NRF24 nrf24(PIN_NRF_CE, PIN_NRF_SC);

void setup() {
  panic_init(PIN_PANIC);
  analogReference(INTERNAL);

  for (auto r = io::init(nrf24); r != Error::OK;) {
    return halt(r);
  }

  report(Error::OK);
}

typedef decltype(millis()) millis_t;

static void send_pwm(millis_t ms, millis_t period) {
  static millis_t lastMs = 0;

  auto dt = ms - lastMs;
  if (dt < period) {
    return;
  }
  lastMs = ms;

  auto a0 = analogRead(A0);
  auto a1 = analogRead(A1);
  auto pos = map(a0, 0, a1, INT8_MIN, INT8_MAX);
  // auto vin = map(a1, 1776, 2945, 3300, 5000);

  commands::set_pwm cmd;
  cmd.value = pos;
  auto rs = io::send(nrf24, cmd.begin(), sizeof(cmd));
  if (rs != Error::OK) {
    report(rs);
  }
}

static void handle_command(const commands::_base *cmd) {
  switch (cmd->id) {
    case commands::reply_status::ID: {
      auto &status = *static_cast<const commands::reply_status*>(cmd);
      Serial.print(status.tempFET);
      Serial.print(" ");
      Serial.print(status.ampsMotor);
      Serial.print(" ");
      Serial.print(status.ampsInput);
      Serial.print(" ");
      Serial.print(status.dutyCycle);
      Serial.print(" ");
      Serial.print(status.rpm);
      Serial.print(" ");
      Serial.print(status.voltInput);
      Serial.print(" ");
      Serial.print(status.tachometer);
      Serial.print(" ");
      Serial.print(status.tachometerAbs);
      Serial.print(" ");
      Serial.println();
    } break;
    default:
      panic::report(Error::UNKNOWN_CMD);
      return;
  }
}

void loop() {
  auto ms = millis();

  send_pwm(ms, 10);

  uint8_t message[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(message);
  auto rr = io::recv(nrf24, message, &len);
  switch (rr) {
    case Error::OK:
      if (len) {
        auto cmd = reinterpret_cast<const commands::_base*>(message);
        handle_command(cmd);
      }
      break;
    default:
      report(rr);
      break;
  }
}
