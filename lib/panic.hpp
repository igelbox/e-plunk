#pragma once

#include <Arduino.h>

namespace panic {

enum class Error {
  OK = 0x0,
  NRF_INIT = 0x1,
  NRF_CHANNEL = 0x2,
  NRF_POWER = 0x3,
  NRF_RECV = 0x4,
  NRF_RECV_EMPTY = 0x5,
  RECV_CRC = 0x6,
  SEND = 0x7,
  SEND_WAIT = 0x8,
  SERVO_INIT = 0x9,
  PWM_TIMEOUT = 0xA,
  UNKNOWN_CMD = 0xB,
  VESC_UART = 0xC,
  DISPLAY_INIT = 0xD,
};

static uint8_t pin = 0;
typedef void (*println_t)(const char *kind, uint8_t code);
static println_t println = nullptr;

void panic_init(uint8_t pin) {
  panic::pin = pin;
  pinMode(pin, OUTPUT);

  noInterrupts();
  TCCR2A = 0;
  TCCR2B = 0b101;          // 1024 prescaler
  TIMSK2 |= (1 << TOIE2);  // enable timer overflow interrupt ISR
  interrupts();
}

void init_printer(println_t println) {
  panic::println = println;
}

typedef decltype(millis()) millis_t;

static volatile uint8_t _echo_code;
static volatile millis_t _echo_started = 0;

static bool _echo(uint8_t code) {
  if (_echo_started) {
    return false;
  }

  _echo_code = code;
  _echo_started = millis();
  return true;
}

bool idle(const millis_t ms) {
  if (!_echo_started) {
    return false;
  }

  auto idx = (ms - _echo_started) / 64;
  if (idx < 8) {
    digitalWrite(pin, (0b00101000 >> idx) & 0x1);
    return true;
  }
  idx -= 8;
  auto bit = idx / 8;
  auto val = _echo_code >> bit;
  if (!val) {
    _echo_started = 0;
    return false;
  }

  auto mask = val & 0x1         //
                  ? 0b01111110  //
                  : 0b00010000;
  digitalWrite(pin, (mask >> (idx & 0x7)) & 0x1);
  return true;
}

// volatile uint32_t counter = 0;

ISR(TIMER2_OVF_vect) {
  // ++counter;
  idle(millis());
}

void report(Error error) {
  if (!pin) {
    return;
  }

  const auto code = static_cast<uint8_t>(error);

  if (println) {
    println("REPORT", code);
  }

  _echo(code);
}

void halt(Error error) {
  while (!pin) {
  }

  const auto code = static_cast<uint8_t>(error);

  if (println) {
    println("!PANIC", code);
  }

  for (;;) {
    _echo(code);
  }
}

}  // namespace panic
