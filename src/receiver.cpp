#include <RH_NRF24.h>
#include <Servo.h>
#include <VescUart.h>
#include <avr/wdt.h>

#include "../lib/commands.hpp"
#include "../lib/io.hpp"
#include "../lib/panic.hpp"

using namespace panic;

typedef decltype(millis()) millis_t;

#define PIN_PANIC 4
#define PIN_PWM 6

#define PIN_NRF_CE 9
#define PIN_NRF_SC 10
// #define PIN_NRF_.. 11, 12, 13

#define WHEEL_CIRCUMFERENCE_M .075f * PI
#define GEAR_RATIO 16.f / 48.f
#define MOTOR_POLES 7

RH_NRF24 nrf24(PIN_NRF_CE, PIN_NRF_SC);
VescUart vescUart;
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

  Serial.begin(19200);
  vescUart.setSerialPort(&Serial);

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
  if (!vescUart.getVescValues()) {
    return report(Error::VESC_UART);
  }
  cmd.tempFET = (int8_t)vescUart.data.tempMosfet;
  cmd.ampsMotor = (int16_t)(vescUart.data.avgMotorCurrent * 10.f);
  cmd.ampsInput = (int16_t)(vescUart.data.avgInputCurrent * 10.f);
  cmd.dutyCycle = (uint8_t)(vescUart.data.dutyCycleNow * UINT8_MAX);
  auto wpm = vescUart.data.rpm / MOTOR_POLES * GEAR_RATIO;
  auto speed = WHEEL_CIRCUMFERENCE_M * wpm * .06f;
  cmd.speed = (uint8_t)(speed * 10.0f);
  cmd.voltInput = (int16_t)(vescUart.data.inpVoltage * 10.f);
  auto rs = io::send(nrf24, cmd.begin(), sizeof(cmd));
  if (rs != Error::OK) {
    report(rs);
  }
}

millis_t lastPwm = 0;

static void handle_command(const commands::_base* cmd, const millis_t ms) {
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
