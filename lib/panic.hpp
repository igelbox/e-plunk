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
};

static uint8_t pin = 0;

void panic_init(uint8_t pin) {
  Serial.begin(9600);
  panic::pin = pin;
  pinMode(pin, OUTPUT);
}

void halt(Error error) {
  while (!pin) {
  }

  const auto code = static_cast<uint8_t>(error);
  Serial.print("Panic: ");
  Serial.println(code);

  for (;;) {
    for (auto i = 0; i < 2; ++i) {
      digitalWrite(pin, HIGH);
      delay(50);
      digitalWrite(pin, LOW);
      delay(50);
    }
    delay(400);

    for (auto i = 0; i < 4; ++i) {
      auto time = (code >> i) & 1 ? 400 : 100;
      digitalWrite(pin, HIGH);
      delay(time);
      digitalWrite(pin, LOW);
      delay(500 - time);
    }

    delay(1000);
  }
}

}  // namespace panic
