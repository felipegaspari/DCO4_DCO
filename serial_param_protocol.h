#ifndef SERIAL_PARAM_PROTOCOL_H
#define SERIAL_PARAM_PROTOCOL_H

#include <stdint.h>

// Shared helpers for decoding parameter frames carried over Serial
// using the 'p'/'w'/'x' commands.
//
// Layouts:
//   'p' : [paramNumber, hi, lo, finish]
//   'w' : [paramNumber, int8 value, finish]
//   'x' : [paramNumber, b0, b1, b2, b3, finish]  (LE 32-bit)

struct ParamFrame {
  uint8_t id;
  int32_t value;
};

static inline int16_t decode_i16_be(const uint8_t* b) {
  return (int16_t)((uint16_t(b[0]) << 8) | uint16_t(b[1]));
}

static inline int32_t decode_i32_le(const uint8_t* b) {
  return int32_t(uint32_t(b[0]) |
                 (uint32_t(b[1]) << 8) |
                 (uint32_t(b[2]) << 16) |
                 (uint32_t(b[3]) << 24));
}

static inline void decode_param_p(const uint8_t* payload, ParamFrame& out) {
  out.id = payload[0];
  out.value = decode_i16_be(payload + 1);
}

static inline void decode_param_w(const uint8_t* payload, ParamFrame& out) {
  out.id = payload[0];
  out.value = (int8_t)payload[1];
}

static inline void decode_param_x(const uint8_t* payload, ParamFrame& out) {
  out.id = payload[0];
  out.value = decode_i32_le(payload + 1);
}

#endif  // SERIAL_PARAM_PROTOCOL_H


