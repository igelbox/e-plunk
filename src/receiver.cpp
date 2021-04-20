#include <RH_NRF24.h>
#include <avr/wdt.h>
#include <Servo.h>

#include "../lib/commands.hpp"
#include "../lib/io.hpp"
#include "../lib/panic.hpp"

using namespace panic;

typedef decltype(millis()) millis_t;

#define PIN_PANIC 4
#define PIN_PWM 5

#define PIN_NRF_CE 9
#define PIN_NRF_SC 10
// #define PIN_NRF_.. 11, 12, 13

RH_NRF24 nrf24(PIN_NRF_CE, PIN_NRF_SC);
static const long PWM_MIN = 1000;
static const long PWM_NO_SIGNAL = 1400;  // A slight brake
static const long PWM_MAX = 2000;
static const millis_t PWM_TIMEOUT_MS = 500;  // Resets PWM to Neutral
Servo vescPwm;

void setup() {
  wdt_enable(WDTO_500MS);
  panic_init(PIN_PANIC);

  if (!vescPwm.attach(PIN_PWM)) {
    // return halt(Error::SERVO_INIT);
  }
  vescPwm.writeMicroseconds(PWM_NO_SIGNAL);

  for (auto r = io::init(nrf24); r != Error::OK;) {
    return halt(r);
  }

  report(Error::OK);
}

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
    report(rs);
  }
}

millis_t lastPwm = 0;

static void handle_command(const commands::_base *cmd, const millis_t ms) {
  switch (cmd->id) {
    case commands::set_pwm::ID: {
      lastPwm = ms;
      auto set_pwm = static_cast<const commands::set_pwm*>(cmd);
      auto pwm = map(set_pwm->value, INT8_MIN, INT8_MAX, PWM_MIN, PWM_MAX);
      // Serial.println(set_pwm->value);
      vescPwm.writeMicroseconds(pwm);
    } break;
    default:
      report(Error::UNKNOWN_CMD);
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
        auto cmd = reinterpret_cast<const commands::_base*>(message);
        handle_command(cmd, ms);
      }
      break;
    default:
      report(rr);
      break;
  }

  if ((ms - lastPwm) > PWM_TIMEOUT_MS) {
    vescPwm.writeMicroseconds(PWM_NO_SIGNAL);
    report(Error::PWM_TIMEOUT);
  }
}
