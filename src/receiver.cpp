#include <RH_NRF24.h>
#include <avr/wdt.h>

#include "../lib/commands.hpp"
#include "../lib/io.hpp"
#include "../lib/panic.hpp"

using namespace panic;

RH_NRF24 nrf24(9, 10);

void setup() {
  wdt_enable(WDTO_500MS);

  panic_init(13);
  for (auto r = io::init(nrf24); r != Error::OK;) {
    return halt(r);
  }

  Serial.println("Ready");
}

typedef decltype(millis()) millis_t;

static void send_status(millis_t ms, millis_t period) {
  static millis_t lastMs = 0;

  auto dt = ms - lastMs;
  if (dt < period) {
    return;
  }
  lastMs = ms;

  commands::reply_status cmd;
  auto rs = io::send(nrf24, cmd.begin(), sizeof(cmd));
  if (rs != Error::OK) {
    Serial.print("Send RS failed: ");
    Serial.println(static_cast<uint8_t>(rs));
  }
}

static void handle_command(commands::_base& cmd) {
  switch (cmd.id) {
    case commands::set_pwm::ID: {
      auto set_pwm = *static_cast<commands::set_pwm*>(&cmd);
      Serial.println(set_pwm.value);
    } break;
    default:
      Serial.print("Unknown command: ");
      Serial.println(cmd.id);
      return;
  }
}

void loop() {
  wdt_reset();

  auto ms = millis();

  send_status(ms, 100);

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
