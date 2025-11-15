#include "serial_param_protocol.h"

void init_serial() {
  // init serial midi
  Serial1.setFIFOSize(256);
  Serial1.setPollingMode(true);

  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(31250);

  Serial2.setFIFOSize(512);
  Serial2.setPollingMode(false);
  Serial2.setRX(21);
  Serial2.setTX(20);
  Serial2.begin(2500000);

  Serial.begin(2000000);
}

// -------------------------------
// Serial2 parser (non-blocking)
// -------------------------------
//
// This code parses the high-speed UART link on Serial2 (2.5 Mbps).
// The protocol is intentionally simple:
//
//   [1 byte] command character
//   [N bytes] payload (length depends on command)
//
// There is no explicit start-of-frame marker or checksum. We rely on:
//   - fixed, known payload sizes per command
//   - a timeout to drop incomplete frames
//
// Current commands and payload layouts (as seen by this parser):
//
//   'f' : 2-byte little-endian value
//         payload[0..1] = uint16_t pwRaw (LE)
//
//   's' : 8-byte big-endian ADSR values
//         payload[0..1] = ADSR1_attack  (BE)
//         payload[2..3] = ADSR1_decay   (BE)
//         payload[4..5] = ADSR1_sustain (BE)
//         payload[6..7] = ADSR1_release (BE)
//
//   'p' : 3 bytes + 1 finish byte
//         payload[0]   = paramNumber
//         payload[1..2]= paramValue (BE uint16)
//         payload[3]   = finishByte (not used here)
//
//   'w' : 2 bytes + 1 finish byte
//         payload[0]   = paramNumber
//         payload[1]   = int8 value (will be sign-extended)
//         payload[2]   = finishByte (not used here)
//
//   'x' : 4 bytes + 1 finish byte
//         payload[0]   = paramNumber
//         payload[1..4]= int32 value (LE)
//         payload[5]   = finishByte (not used here)
//
// If you add a new command in the future:
//   1) Choose a command character.
//   2) Define the payload layout.
//   3) Add its expected length to serial2_process_byte().
//   4) Add a case in serial2_handle_complete_frame().
//
// The parser is fully non-blocking: it processes one byte at a time
// and never spins waiting for data.

// Maximum payload length among all current commands:
// 'f' = 2, 's' = 8, 'p' = 4, 'w' = 3, 'x' = 5
static const uint8_t SERIAL2_MAX_PAYLOAD = 8;
// Timeout to abandon a partially received frame (in microseconds).
// If the other side stops sending part-way through a frame, we reset
// the parser after this time so new commands can be received cleanly.
static const uint32_t SERIAL2_FRAME_TIMEOUT_US = 5000;  // 5 ms is huge at 2.5 Mbps

enum Serial2ParserState : uint8_t {
  SERIAL2_WAIT_FOR_CMD = 0,  // waiting for the initial command byte
  SERIAL2_READ_PAYLOAD       // accumulating payload bytes for the current command
};

struct Serial2ParserContext {
  Serial2ParserState state;         // current parser state
  char command;                     // current command character
  uint8_t payload[SERIAL2_MAX_PAYLOAD];  // payload buffer for this frame
  uint8_t expected_len;             // how many payload bytes we expect in total
  uint8_t received_len;             // how many payload bytes we have so far
  uint32_t last_byte_time_us;       // timestamp of the last received byte (for timeout)
};

static Serial2ParserContext serial2Parser = {
  SERIAL2_WAIT_FOR_CMD,
  0,
  {0},
  0,
  0,
  0
};

// Endianness helpers
static inline uint16_t read_u16_be(const uint8_t *b) {
  return (uint16_t(b[0]) << 8) | uint16_t(b[1]);
}

static inline uint16_t read_u16_le(const uint8_t *b) {
  return uint16_t(b[0]) | (uint16_t(b[1]) << 8);
}

static inline int32_t read_i32_le(const uint8_t *b) {
  return int32_t(b[0]) |
         (int32_t(b[1]) << 8) |
         (int32_t(b[2]) << 16) |
         (int32_t(b[3]) << 24);
}

// Reset the parser to the initial "waiting for command" state.
static void serial2_reset_parser() {
  serial2Parser.state = SERIAL2_WAIT_FOR_CMD;
  serial2Parser.command = 0;
  serial2Parser.expected_len = 0;
  serial2Parser.received_len = 0;
  serial2Parser.last_byte_time_us = 0;
}

// Handle a completely received frame for the specified command.
// At this point, "payload" contains "length" bytes corresponding
// to the layouts documented at the top of this file.
static void serial2_handle_complete_frame(char command, const uint8_t *payload, uint8_t length) {

  switch (command) {
    case 'f': {
      // 2-byte value, little-endian, applied to PW[0].
      // This keeps the original semantics of the previous implementation.
      if (length == 2) {
        uint16_t pwRaw = read_u16_le(payload);
        PW[0] = DIV_COUNTER_PW - (pwRaw / 4);
      }
      break;
    }

    case 's': {
      // 8 bytes: 4 big-endian uint16 values for ADSR1.
      if (length == 8) {
        ADSR1_attack  = read_u16_be(payload + 0);
        ADSR1_decay   = read_u16_be(payload + 2);
        ADSR1_sustain = read_u16_be(payload + 4);
        ADSR1_release = read_u16_be(payload + 6);
      }
      break;
    }

    case 'p': {
      // [paramNumber, value_hi, value_lo, finishByte]
      // 16-bit big-endian value, decoded via shared helper.
      if (length == 4) {
        ParamFrame frame;
        decode_param_p(payload, frame);
        update_parameters(frame.id, (int16_t)frame.value);
      }
      break;
    }

    case 'w': {
      // [paramNumber, int8 value, finishByte]
      // int8 value sign-extended to 16-bit via shared helper.
      if (length == 3) {
        ParamFrame frame;
        decode_param_w(payload, frame);
        update_parameters(frame.id, (int16_t)frame.value);
      }
      break;
    }

    case 'x': {
      // [paramNumber, int32_t value (little-endian), finishByte]
      // 32-bit little-endian value decoded via shared helper and then
      // truncated to int16 to preserve existing DCO behavior.
      if (length == 5) {
        ParamFrame frame;
        decode_param_x(payload, frame);
        update_parameters(frame.id, (int16_t)frame.value);
      }
      break;
    }

    default:
      // Unknown command: ignore frame
      break;
  }
}

static void serial2_process_byte(uint8_t b, uint32_t now_us) {

  // Timeout handling: if we're partway through a frame and it takes too long, reset.
  if (serial2Parser.state == SERIAL2_READ_PAYLOAD &&
      serial2Parser.last_byte_time_us != 0 &&
      (uint32_t)(now_us - serial2Parser.last_byte_time_us) > SERIAL2_FRAME_TIMEOUT_US) {
    serial2_reset_parser();
  }

  serial2Parser.last_byte_time_us = now_us;

  if (serial2Parser.state == SERIAL2_WAIT_FOR_CMD) {
    char commandCharacter = (char)b;

    // Determine expected payload length for this command.
    uint8_t expected_len = 0;
    switch (commandCharacter) {
      case 'f': expected_len = 2; break;
      case 's': expected_len = 8; break;
      case 'p': expected_len = 4; break;
      case 'w': expected_len = 3; break;
      case 'x': expected_len = 5; break;
      default:
        // Unknown command byte, ignore.
        return;
    }

    serial2Parser.command = commandCharacter;
    serial2Parser.expected_len = expected_len;
    serial2Parser.received_len = 0;
    serial2Parser.state = SERIAL2_READ_PAYLOAD;
    return;
  }

  // SERIAL2_READ_PAYLOAD
  if (serial2Parser.received_len < SERIAL2_MAX_PAYLOAD) {
    serial2Parser.payload[serial2Parser.received_len++] = b;
  }

  if (serial2Parser.received_len >= serial2Parser.expected_len) {
    serial2_handle_complete_frame(serial2Parser.command, serial2Parser.payload, serial2Parser.received_len);
    serial2_reset_parser();
  }
}

static void serial2_check_timeout() {
  if (serial2Parser.state == SERIAL2_READ_PAYLOAD && serial2Parser.last_byte_time_us != 0) {
    uint32_t now_us = micros();
    if ((uint32_t)(now_us - serial2Parser.last_byte_time_us) > SERIAL2_FRAME_TIMEOUT_US) {
      serial2_reset_parser();
    }
  }
}

void serial_STM32_task() {

  // First, expire any stale partial frame
  serial2_check_timeout();

  // Then, consume all available bytes without blocking
  while (Serial2.available() > 0) {
    uint8_t b = Serial2.read();
    uint32_t now_us = micros();
    serial2_process_byte(b, now_us);
  }
}

inline void serial_send_note_on(uint8_t voice_n, uint8_t note_velo, uint8_t note) {


  byte sendArray[4];
  sendArray[0] = (uint8_t)'n';
  sendArray[1] = voice_n;
  sendArray[2] = note_velo;
  sendArray[3] = note;

  while (Serial2.availableForWrite() < 1) {}
  //Serial2.write((char *)"n");
  //while (Serial2.availableForWrite() < 1) {}
  Serial2.write(sendArray, 4);
}

inline void serial_send_note_off(uint8_t voice_n) {
  byte sendArray[2] = { (uint8_t)'o', voice_n };
  //while (Serial2.availableForWrite() < 1) {}
  //Serial2.write((char *)"o");
  while (Serial2.availableForWrite() < 1) {}
  Serial2.write(sendArray, 2);
}

void serial_send_generaldata(uint16_t data) {

  if (uart_is_writable(uart1) > 0) {

    uint8_t *b = (uint8_t *)&data;
    uart_putc(uart1, 'w');
    uart_putc_raw(uart1, b[0]);
    uart_putc_raw(uart1, b[1]);
    //uart_putc(uart1, 'z');
    return;
  }
}

inline void serialSendParam32(byte paramNumber, uint32_t paramValue) {

  uint8_t *b = (uint8_t *)&paramValue;
  byte finishByte = 1;

  byte bytesArray[7] = { (uint8_t)'x', paramNumber, b[0], b[1], b[2], b[3], finishByte };
  while (Serial2.availableForWrite() < 1) {}
  Serial2.write(bytesArray, 7);
}

void serial_send_freq(float f) {
  //     uint8_t *b = (uint8_t *)&f;
  //     uint8_t* ret = malloc(4 * sizeof(uint8_t));
  // unsigned int asInt = *((int*)&f);

  // int i;
  // for (i = 0; i < 4; i++) {
  //     ret[i] = (asInt >> 8 * i) & 0xFF;
  // }

  // uint8_t *b = (uint8_t *)&f;
  // while (Serial2.availableForWrite() == 0) {}

  // Serial2.write((char *)'f');
  // Serial2.write(b, 2);
  return;
}

// void serial_send_voice_freq() {
//   for (uint8_t i = 0; i < NUM_VOICES; i++) {
//     float f = voiceFreq[i];
//     if (uart_is_writable(uart1) > 0) {
//       uint8_t *b = (uint8_t *)&f;
//       uart_putc(uart1, 'v');
//       uart_putc(uart1, i);
//       uart_putc(uart1, (b[0]));
//       uart_putc(uart1, (b[1]));
//       uart_putc(uart1, (b[2]));
//       uart_putc(uart1, (b[3]));
//       uart_putc(uart1, '6');
//       return;
//     }
//   }
// }
