#pragma once

#include <RH_NRF24.h>

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

Error init(RH_NRF24& nrf) {
  if (!nrf.init()) {
    return Error::NRF_INIT;
  }

  if (!nrf.setChannel(CHANNEL)) {
    return Error::NRF_CHANNEL;
  }

  if (!nrf.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm)) {
    return Error::NRF_POWER;
  }

  return Error::OK;
}

Error send(RH_NRF24& nrf, const uint8_t* data, uint8_t size) {
  assert(data);
  assert(size < RH_NRF24_MAX_MESSAGE_LEN);

  uint8_t message[RH_NRF24_MAX_MESSAGE_LEN];
  auto crc = OneWire::crc8(data, size);
  memcpy(message, data, size);
  _crypt(message, size);
  message[size] = crc;

  if (!nrf.send(message, size + 1)) {
    return Error::SEND;
  }

  if (!nrf.waitPacketSent()) {
    return Error::SEND_WAIT;
  }

  return Error::OK;
}

Error recv(RH_NRF24& nrf, uint8_t* buffer, uint8_t* size) {
  assert(buffer);
  if (!nrf.available()) {
    *size = 0;
    return Error::OK;
  }

  if (!nrf.recv(buffer, size)) {
    return Error::NRF_RECV;
  }
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
