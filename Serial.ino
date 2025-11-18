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

  // Serial.begin(2000000);
}

/// -------------------------------
// Serial2 parser (non-blocking, shared core)
// -------------------------------
//
// This section wires the generic serial_parser.h core to the DCO's
// Serial2 link. The DCO receives:
//
//   'f' : PW UPDATE       (from mainboard)
//   's' : ADSR BLOCK      (from mainboard)
//   'p' : PARAM 16-bit    (shared param frame)
//   'w' : PARAM 8-bit     (shared param frame)
//   'x' : PARAM 32-bit    (shared param frame, truncated to int16 here)
//
// The parser core is fully shared between MCUs; only the per-command
// handlers differ by board.

// DCO-specific handler: PW UPDATE ('f')
static void dco_handle_pw_update(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_PW_UPDATE) {
    return;
  }
  // 2-byte value, little-endian, applied to PW[0].
  uint16_t pwRaw = (uint16_t)payload[0] | (uint16_t(payload[1]) << 8);
  PW[0] = pwRaw / 4;
}

// DCO-specific handler: ADSR BLOCK ('s')
static void dco_handle_adsr_block(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_ADSR_BLOCK) {
    return;
  }
  // 8 bytes: 4 big-endian uint16 values for ADSR1.
  ADSR1_attack  = (uint16_t(payload[0]) << 8) | uint16_t(payload[1]);
  ADSR1_decay   = (uint16_t(payload[2]) << 8) | uint16_t(payload[3]);
  ADSR1_sustain = (uint16_t(payload[4]) << 8) | uint16_t(payload[5]);
  ADSR1_release = (uint16_t(payload[6]) << 8) | uint16_t(payload[7]);
}

// Shared handler pattern for parameter frames: decode via serial_param_protocol.h
// and then call update_parameters() with int16_t (DCO uses 16-bit transport).
static void dco_handle_param16(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_PARAM_16) {
    return;
  }
  ParamFrame frame;
  decode_param_p(payload, frame);
  update_parameters(frame.id, (int16_t)frame.value);
}

static void dco_handle_param8(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_PARAM_8) {
    return;
  }
  ParamFrame frame;
  decode_param_w(payload, frame);
  update_parameters(frame.id, (int16_t)frame.value);
}

static void dco_handle_param32(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_PARAM_32) {
    return;
  }
  ParamFrame frame;
  decode_param_x(payload, frame);
  // Truncate 32-bit value to 16-bit to preserve existing DCO behavior.
  update_parameters(frame.id, (int16_t)frame.value);
}

// Command table for the DCO's Serial2 link.
// Only commands that the DCO actually expects are listed here; any other
// command bytes will be ignored by the parser core.
static const SerialCommandDef dcoSerial2Commands[] = {
  { SERIAL_CMD_PW_UPDATE,  SERIAL_PAYLOAD_LEN_PW_UPDATE,  dco_handle_pw_update  },
  { SERIAL_CMD_ADSR_BLOCK, SERIAL_PAYLOAD_LEN_ADSR_BLOCK, dco_handle_adsr_block },
  { SERIAL_CMD_PARAM_16,   SERIAL_PAYLOAD_LEN_PARAM_16,   dco_handle_param16    },
  { SERIAL_CMD_PARAM_8,    SERIAL_PAYLOAD_LEN_PARAM_8,    dco_handle_param8     },
  { SERIAL_CMD_PARAM_32,   SERIAL_PAYLOAD_LEN_PARAM_32,   dco_handle_param32    },
};

// Parser context for this link: stores state between bytes.
static SerialParserContext dcoSerial2Parser = {
  SERIAL_WAIT_FOR_CMD,
  0,
  nullptr,
  {0},
  0,
  0,
  0
};

// Main entry point for DCO <-> mainboard Serial2 receive.
void serial_STM32_task() {
  // First, expire any stale partial frame (only if we're in a frame).
  if (dcoSerial2Parser.state == SERIAL_READ_PAYLOAD) {
    uint32_t now = micros();
    serial_parser_check_timeout(dcoSerial2Parser, now);
  }

  // Then, consume all available bytes without blocking.
  if (Serial2.available() > 0) {
    uint32_t now = micros();  // one timestamp per batch is enough
    while (Serial2.available() > 0) {
      uint8_t b = Serial2.read();
      serial_parser_process_byte(
        dcoSerial2Parser,
        dcoSerial2Commands,
        sizeof(dcoSerial2Commands) / sizeof(dcoSerial2Commands[0]),
        b,
        now
      );
    }
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
