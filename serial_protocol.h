#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>

// -----------------------------------------------------------------------------
// Serial protocol: command byte definitions and payload sizes.
// This is the SINGLE source of truth for the on-wire protocol between the
// mainboard and the DCO board. All MCUs use the same command set, even if
// some commands are ignored on a particular board.
//
// Frames on the wire have the form:
//   [1 byte] command character
//   [N bytes] payload (length depends on command, defined below)
// -----------------------------------------------------------------------------

// Command bytes (char) used on the high-speed links (mainboard <-> DCO).
// Semantics:
//
//   'n' : NOTE ON   (DCO -> mainboard)
//   'o' : NOTE OFF  (DCO -> mainboard)
//
//   'f' : PW UPDATE (mainboard -> DCO)
//   's' : ADSR BLOCK (mainboard -> DCO)
//
//   'p' : PARAM 16-bit (id + int16 BE + finish)
//   'w' : PARAM 8-bit  (id + int8  + finish)
//   'x' : PARAM 32-bit (id + uint32 LE + finish)
//
// Other links (e.g. Serial8 to the input board) have additional commands,
// but this header focuses on the core mainboard<->DCO protocol.
enum SerialCmd : char {
  SERIAL_CMD_NOTE_ON    = 'n',
  SERIAL_CMD_NOTE_OFF   = 'o',

  SERIAL_CMD_PW_UPDATE  = 'f',
  SERIAL_CMD_ADSR_BLOCK = 's',

  SERIAL_CMD_PARAM_16   = 'p',
  SERIAL_CMD_PARAM_8    = 'w',
  SERIAL_CMD_PARAM_32   = 'x',
};

// Canonical payload sizes (NOT counting the command byte itself).
// These must match the on-wire layouts used by all MCUs.
//
// NOTE ON ('n'):
//   payload[0] = voice index
//   payload[1] = velocity
//   payload[2] = MIDI note
static constexpr uint8_t SERIAL_PAYLOAD_LEN_NOTE_ON    = 3;

// NOTE OFF ('o'):
//   payload[0] = voice index
static constexpr uint8_t SERIAL_PAYLOAD_LEN_NOTE_OFF   = 1;

// PW UPDATE ('f'):
//   payload[0..1] = uint16_t pwRaw (little-endian)
static constexpr uint8_t SERIAL_PAYLOAD_LEN_PW_UPDATE  = 2;

// ADSR BLOCK ('s'):
//   payload[0..1] = ADSR_attack  (big-endian)
//   payload[2..3] = ADSR_decay   (big-endian)
//   payload[4..5] = ADSR_sustain (big-endian)
//   payload[6..7] = ADSR_release (big-endian)
static constexpr uint8_t SERIAL_PAYLOAD_LEN_ADSR_BLOCK = 8;

// PARAM 16-bit ('p'):
//   payload[0]   = paramNumber (ParamId)
//   payload[1..2]= int16 value, big-endian
//   payload[3]   = finish byte (usually 1)
static constexpr uint8_t SERIAL_PAYLOAD_LEN_PARAM_16   = 4;

// PARAM 8-bit ('w'):
//   payload[0]   = paramNumber (ParamId)
//   payload[1]   = int8 value (sign-extended)
//   payload[2]   = finish byte (usually 1)
static constexpr uint8_t SERIAL_PAYLOAD_LEN_PARAM_8    = 3;

// PARAM 32-bit ('x'):
//   payload[0]   = paramNumber (ParamId)
//   payload[1..4]= uint32 value, little-endian
//   payload[5]   = finish byte (usually 1)
static constexpr uint8_t SERIAL_PAYLOAD_LEN_PARAM_32   = 6;

// Helper: return canonical payload length for a command.
// Every MCU should use this so that the parser treats commands consistently.
static inline uint8_t serial_protocol_payload_len(char cmd) {
  switch (cmd) {
    case SERIAL_CMD_NOTE_ON:    return SERIAL_PAYLOAD_LEN_NOTE_ON;
    case SERIAL_CMD_NOTE_OFF:   return SERIAL_PAYLOAD_LEN_NOTE_OFF;
    case SERIAL_CMD_PW_UPDATE:  return SERIAL_PAYLOAD_LEN_PW_UPDATE;
    case SERIAL_CMD_ADSR_BLOCK: return SERIAL_PAYLOAD_LEN_ADSR_BLOCK;
    case SERIAL_CMD_PARAM_16:   return SERIAL_PAYLOAD_LEN_PARAM_16;
    case SERIAL_CMD_PARAM_8:    return SERIAL_PAYLOAD_LEN_PARAM_8;
    case SERIAL_CMD_PARAM_32:   return SERIAL_PAYLOAD_LEN_PARAM_32;
    default:                    return 0; // unknown or unsupported command
  }
}

#endif // SERIAL_PROTOCOL_H



