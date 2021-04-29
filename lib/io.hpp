#pragma once

#include <RF24.h>

#define RF24_MAX_MESSAGE_LEN 32

#include <OneWire.h>
#include <assert.h>
#include <stdint.h>

#include "panic.hpp"

#include "../secret.hpp"

namespace io {
using panic::Error;

static void _crypt(void* data, uint8_t size) {
  static_assert(sizeof(CIPHER) >= 4);
  assert(data);

  const uint8_t to = max(sizeof(CIPHER), size);
  for (auto i = 0; i < to; ++i) {
    reinterpret_cast<uint8_t*>(data)[i % size] ^= CIPHER[i % sizeof(CIPHER)];
  }
}

Error init(RF24& nrf) {
  if (!nrf.begin()) {
    return Error::NRF_INIT;
  }

  nrf.setChannel(CHANNEL);
  nrf.setPALevel(RF24_PA_HIGH);
  nrf.enableDynamicPayloads();
  if (!nrf.setDataRate(RF24_250KBPS)) {
    return Error::NRF_POWER;
  }

  return Error::OK;
}

typedef uint8_t message_t[RF24_MAX_MESSAGE_LEN];
uint8_t prepare_send(message_t& message, const uint8_t* data, uint8_t size) {
  assert(data);
  assert(size < RF24_MAX_MESSAGE_LEN);

  auto crc = OneWire::crc8(data, size);
  memcpy(message, data, size);
  _crypt(message, size);
  message[size] = crc;
  return size + 1;
}

Error recv(RF24& nrf, uint8_t* buffer, uint8_t* size) {
  assert(buffer);
  if (!nrf.available()) {
    *size = 0;
    return Error::OK;
  }

  nrf.read(buffer, *size);
  *size = nrf.getDynamicPayloadSize();
  if (*size == 0) {
    return Error::NRF_RECV_EMPTY;
  }

  --*size;
  _crypt(buffer, *size);
  auto crc = OneWire::crc8(buffer, *size);
  if (crc != buffer[*size]) {
    return Error::RECV_CRC;
  }

  return Error::OK;
}

}  // namespace io
