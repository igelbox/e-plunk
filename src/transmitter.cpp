#include <RH_NRF24.h>
#include "../lib/commands.hpp"
#include "../lib/io.hpp"
#include "../lib/panic.hpp"

using namespace panic;

RH_NRF24 nrf24(9, 10);

void setup() {
  panic_init(13);

  for (auto r = io::init(nrf24); r != Error::OK;) {
    return halt(r);
  }

  Serial.println("Ready");
}

typedef decltype(millis()) millis_t;

static void send_pwm(millis_t ms, millis_t period) {
  static millis_t lastMs = 0;

  auto dt = ms - lastMs;
  if (dt < period) {
    return;
  }
  lastMs = ms;

  commands::set_pwm cmd;
  cmd.value = 0;
  auto rs = io::send(nrf24, cmd.begin(), sizeof(cmd));
  if (rs != Error::OK) {
    Serial.print("Send PWM failed: ");
    Serial.println(static_cast<uint8_t>(rs));
  }
}

static void handle_command(commands::_base& cmd) {
  switch (cmd.id) {
    case commands::reply_status::ID: {
      auto status = *static_cast<commands::reply_status*>(&cmd);
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
      Serial.print("Unknown command: ");
      Serial.println(cmd.id);
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
        auto cmd = *reinterpret_cast<commands::_base*>(message);
        handle_command(cmd);
      }
      break;
    default:
      Serial.print("Recv failed: ");
      Serial.println(static_cast<uint8_t>(rr));
      break;
  }
}
