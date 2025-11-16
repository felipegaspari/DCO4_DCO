#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <stdint.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
// Generic serial frame parser core.
//
// Responsibilities:
//   - Tracks "wait for command" vs "read payload" state.
//   - Uses a per-link SerialCommandDef[] table to know valid commands and
//     payload sizes (often taken from serial_protocol.h).
//   - Accumulates payload bytes for each frame.
//   - On full frame, calls the corresponding on_frame() callback.
//   - Enforces a timeout to drop broken/partial frames.
//
// The parser itself does NOT know anything about the meaning of commands
// ('n', 'o', 'p', 'w', 'x', etc.). It only knows:
//   - "This command expects N payload bytes"
//   - "When we have N bytes, call this callback".
//
// This keeps the parser fully shared between MCUs; each board only needs
// to define its own command table and on_frame handlers.
// -----------------------------------------------------------------------------

// Maximum payload size among all commands on this link.
// For the current protocols:
//   - mainboard<->DCO: 's' uses 8 bytes,
//   - other links may use up to 9 bytes (e.g. preset-name frames).
// So 9 bytes is enough for all current commands.
static const uint8_t SERIAL_MAX_PAYLOAD = 9;

// Timeout (in microseconds) to abandon a partially received frame.
// If we get stuck mid-frame (e.g. bytes lost), we reset after this delay
// so new commands can be received cleanly.
static const uint32_t SERIAL_FRAME_TIMEOUT_US = 5000;  // 5 ms

// Parser states:
//   SERIAL_WAIT_FOR_CMD : expecting the command byte.
//   SERIAL_READ_PAYLOAD : accumulating payload bytes for the current command.
enum SerialParserState : uint8_t {
  SERIAL_WAIT_FOR_CMD = 0,
  SERIAL_READ_PAYLOAD = 1
};

// Forward declaration of SerialCommandDef (see below).
struct SerialCommandDef;

// Per-link parser context: one instance per UART/link.
// This struct holds all the state needed for the parser between calls.
struct SerialParserContext {
  SerialParserState        state;                        // current parser state
  char                     command;                      // current command byte
  const SerialCommandDef*  cmd_def;                      // cached command definition
  uint8_t                  payload[SERIAL_MAX_PAYLOAD];  // payload buffer
  uint8_t                  expected_len;                 // total payload bytes expected
  uint8_t                  received_len;                 // payload bytes received so far
  uint32_t                 last_byte_time_us;            // timestamp of last byte (for timeout)
};

// Per-command definition:
//   - cmd        : command character (e.g. 'n', 'p', 'w', ...)
//   - payload_len: number of payload bytes expected after cmd
//   - on_frame   : callback to invoke when a complete frame is received.
//
// The callback is responsible for interpreting the payload and updating
// local state (note arrays, parameters, etc.).
struct SerialCommandDef {
  char     cmd;
  uint8_t  payload_len;
  void   (*on_frame)(char cmd, const uint8_t* payload, uint8_t len);
};

// Reset parser to initial state (wait for a new command).
static inline void serial_parser_reset(SerialParserContext& ctx) {
  ctx.state             = SERIAL_WAIT_FOR_CMD;
  ctx.command           = 0;
  ctx.cmd_def           = nullptr;
  ctx.expected_len      = 0;
  ctx.received_len      = 0;
  ctx.last_byte_time_us = 0;
}

// Find a command definition in the table for the given cmd.
// Returns nullptr if not found (unknown command).
static inline const SerialCommandDef* serial_parser_find_cmd(
    const SerialCommandDef* commands,
    size_t numCommands,
    char cmd)
{
  for (size_t i = 0; i < numCommands; ++i) {
    if (commands[i].cmd == cmd) {
      return &commands[i];
    }
  }
  return nullptr;
}

// Optionally called from your main loop to drop stale partial frames.
// (You can also rely solely on the timeout logic in serial_parser_process_byte.)
static inline void serial_parser_check_timeout(SerialParserContext& ctx,
                                               uint32_t now_us)
{
  if (ctx.state == SERIAL_READ_PAYLOAD && ctx.last_byte_time_us != 0) {
    if ((uint32_t)(now_us - ctx.last_byte_time_us) > SERIAL_FRAME_TIMEOUT_US) {
      serial_parser_reset(ctx);
    }
  }
}

// Main entry point: feed one byte into the parser.
//
//   - ctx         : per-link parser context (keeps state between calls).
//   - commands    : array of SerialCommandDef entries for this link.
//   - numCommands : number of entries in the commands[] array.
//   - b           : newly received byte from the UART.
//   - now_us      : current time in microseconds (for timeout).
//
// Behavior:
//   - In SERIAL_WAIT_FOR_CMD:
//       * interpret 'b' as command byte
//       * look up payload length from commands[]
//       * if unknown command, ignore byte
//       * if known, switch to SERIAL_READ_PAYLOAD and start filling payload[]
//   - In SERIAL_READ_PAYLOAD:
//       * store 'b' into payload[]
//       * when we reach expected_len, call on_frame()
//       * then reset back to SERIAL_WAIT_FOR_CMD
static inline void serial_parser_process_byte(
    SerialParserContext& ctx,
    const SerialCommandDef* commands,
    size_t numCommands,
    uint8_t b,
    uint32_t now_us)
{
  // Timeout handling: if we're partway through a frame and it took too long
  // since the last byte, reset to avoid getting stuck.
  if (ctx.state == SERIAL_READ_PAYLOAD &&
      ctx.last_byte_time_us != 0 &&
      (uint32_t)(now_us - ctx.last_byte_time_us) > SERIAL_FRAME_TIMEOUT_US) {
    serial_parser_reset(ctx);
  }

  ctx.last_byte_time_us = now_us;

  if (ctx.state == SERIAL_WAIT_FOR_CMD) {
    char cmd = (char)b;
    const SerialCommandDef* def =
        serial_parser_find_cmd(commands, numCommands, cmd);
    if (!def) {
      // Unknown command: ignore and stay in SERIAL_WAIT_FOR_CMD.
      return;
    }

    ctx.command      = cmd;
    ctx.cmd_def      = def;
    ctx.expected_len = def->payload_len;
    ctx.received_len = 0;
    ctx.state        = SERIAL_READ_PAYLOAD;
    return;
  }

  // SERIAL_READ_PAYLOAD
  if (ctx.received_len < SERIAL_MAX_PAYLOAD) {
    ctx.payload[ctx.received_len++] = b;
  }

  if (ctx.received_len >= ctx.expected_len) {
    // We have a full frame: use cached command definition, if any, and call it.
    if (ctx.cmd_def && ctx.cmd_def->on_frame) {
      ctx.cmd_def->on_frame(ctx.command, ctx.payload, ctx.received_len);
    }
    serial_parser_reset(ctx);
  }
}

#endif // SERIAL_PARSER_H



