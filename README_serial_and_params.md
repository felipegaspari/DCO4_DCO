## Serial & Parameter Protocol – Usage Guide

This document explains how to use the shared **serial** and **parameter** infrastructure in any MCU project in the DCO4 system (DCO board, mainboard, screen, input board, etc.).

The goal: you can copy the library headers into a new project, define a few hooks, and immediately speak the same protocol.

---

## 1. Shared “library” headers

These files are intended to be MCU‑agnostic and copy‑pasteable between projects:

- `params_def.h` – canonical `enum ParamId : uint16_t` for the whole system.
- `param_router.h` – generic table‑driven parameter router (`ParamDescriptorT` + `param_router_apply`).
- `serial_param_protocol.h` – decode helpers for `'p'/'w'/'x'` parameter frames into `ParamFrame { id, value }`.
- `serial_protocol.h` – command bytes and payload sizes for the **mainboard ↔ DCO** link (`'n','o','f','s','p','w','x'`).
- `serial_input_protocol.h` – command bytes and payload sizes for the **input board → mainboard** link (`'a'..'f','p','w','q'`).
- `serial_parser.h` – generic non‑blocking state‑machine parser (`SerialParserContext`, `SerialCommandDef`, `serial_parser_process_byte`).

You can treat these as the “library core.”

---

## 2. Using the parameter protocol in a new MCU

### 2.1. Set up `params_def.h` and `param_router.h`

1. **Include the shared headers** in your new project:

   ```cpp
   #include "params_def.h"
   #include "param_router.h"
   ```

2. **Pick the value type** this MCU will use for incoming parameters:

   - DCO‑style: `int16_t`.
   - Mainboard‑style: `int32_t`.

3. **Create a router module** (`params.ino` / `.cpp`) on the new MCU:

   ```cpp
   using ParamValueT     = int16_t;          // or int32_t
   using ParamDescriptor = ParamDescriptorT<ParamValueT>;

   // Board-specific apply functions:
   static void apply_param_lfo1_waveform(ParamValueT v) {
     LFO1Waveform = (int8_t)v;
     LFO1_class.setWaveForm(LFO1Waveform);
   }

   static void apply_param_voice_mode(ParamValueT v) {
     voiceMode = (uint8_t)v;
     setVoiceMode();
   }

   // Parameter table:
   static const ParamDescriptor paramTable[] = {
     { PARAM_LFO1_WAVEFORM, apply_param_lfo1_waveform },
     { PARAM_VOICE_MODE,    apply_param_voice_mode    },
     // ...more params...
   };

   static const size_t paramTableSize =
     sizeof(paramTable) / sizeof(paramTable[0]);

   // Public entry point:
   inline void update_parameters(uint16_t rawId, ParamValueT value) {
     param_router_apply<ParamValueT>(
       paramTable,
       paramTableSize,
       rawId,
       value
     );
   }
   ```

4. Anywhere in this MCU you can now call `update_parameters(paramId, value)` and let the router dispatch.

---

## 3. Using `'p'/'w'/'x'` serial parameter frames

### 3.1. Include the protocol helpers

```cpp
#include "serial_param_protocol.h"
#include "serial_protocol.h"   // for mainboard<->DCO-style links
#include "serial_parser.h"
```

### 3.2. Write parameter frame handlers

These are per‑link but structurally the same on every MCU:

```cpp
static void link_handle_param16(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_PARAM_16) return;
  ParamFrame frame;
  decode_param_p(payload, frame);
  update_parameters(frame.id, (ParamValueT)frame.value);
}

static void link_handle_param8(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_PARAM_8) return;
  ParamFrame frame;
  decode_param_w(payload, frame);
  update_parameters(frame.id, (ParamValueT)frame.value);
}

static void link_handle_param32(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_PARAM_32) return;
  ParamFrame frame;
  decode_param_x(payload, frame);
  // cast or truncate as appropriate:
  update_parameters(frame.id, (ParamValueT)frame.value);
}
```

### 3.3. Hook into the generic serial parser

1. Define a command table for this UART:

   ```cpp
   static const SerialCommandDef linkCommands[] = {
     { SERIAL_CMD_PARAM_16, SERIAL_PAYLOAD_LEN_PARAM_16, link_handle_param16 },
     { SERIAL_CMD_PARAM_8,  SERIAL_PAYLOAD_LEN_PARAM_8,  link_handle_param8  },
     { SERIAL_CMD_PARAM_32, SERIAL_PAYLOAD_LEN_PARAM_32, link_handle_param32 },
     // add non-param commands here (see next section)
   };
   ```

2. Allocate a parser context:

   ```cpp
   static SerialParserContext linkParser = {
     SERIAL_WAIT_FOR_CMD,
     0,
     {0},
     0,
     0,
     0
   };
   ```

3. In your serial task for that UART:

   ```cpp
   inline void link_serial_task(HardwareSerial& port) {
     uint32_t now = micros();
     serial_parser_check_timeout(linkParser, now);

     while (port.available() > 0) {
       uint8_t b = port.read();
       now = micros();
       serial_parser_process_byte(
         linkParser,
         linkCommands,
         sizeof(linkCommands) / sizeof(linkCommands[0]),
         b,
         now
       );
     }
   }
   ```

Now this link speaks the shared `'p'/'w'/'x'` protocol and uses the generic parser.

---

## 4. Adding a new serial command (non‑parameter)

Example: add `'g'` = “global reset” on the mainboard↔DCO link.

### 4.1. Extend `serial_protocol.h`

```cpp
enum SerialCmd : char {
  // existing commands...
  SERIAL_CMD_GLOBAL_RESET = 'g',
};

static constexpr uint8_t SERIAL_PAYLOAD_LEN_GLOBAL_RESET = 1;

static inline uint8_t serial_protocol_payload_len(char cmd) {
  switch (cmd) {
    // existing cases...
    case SERIAL_CMD_GLOBAL_RESET: return SERIAL_PAYLOAD_LEN_GLOBAL_RESET;
    default: return 0;
  }
}
```

### 4.2. Implement handlers on receiver MCUs

```cpp
static void handle_global_reset(char, const uint8_t* payload, uint8_t len) {
  if (len != SERIAL_PAYLOAD_LEN_GLOBAL_RESET) return;
  uint8_t flag = payload[0];
  if (flag) {
    reset_all_voices();
    reset_modulation_state();
  }
}
```

Add to that link’s `SerialCommandDef[]`:

```cpp
{ SERIAL_CMD_GLOBAL_RESET, SERIAL_PAYLOAD_LEN_GLOBAL_RESET, handle_global_reset },
```

### 4.3. Send the command

On the sender MCU:

```cpp
inline void serial_send_global_reset(uint8_t flag) {
  while (Serial2.availableForWrite() < 2) {}
  uint8_t bytes[2] = { (uint8_t)SERIAL_CMD_GLOBAL_RESET, flag };
  Serial2.write(bytes, 2);
}
```

---

## 5. Adding a new parameter (`ParamId`) and actions

Example: `PARAM_LFO3_TO_VCF` controlling LFO3 depth to filter.

### 5.1. Add `ParamId` in `params_def.h`

In **both** projects’ `params_def.h`:

```cpp
// choose an unused number
PARAM_LFO3_TO_VCF = 52,  // LFO3 depth -> filter cutoff
```

Rules:

- Never renumber existing IDs.
- Use the **same numeric value** on all MCUs and tools.

### 5.2. Implement `apply_param_*` per MCU

On DCO‑style MCU:

```cpp
static void apply_param_lfo3_to_vcf(int16_t v) {
  LFO3toVCFVal = v;
  float depth  = someMappingFunction(v);
  LFO3toVCF_q24 = (int32_t)(depth * (1 << 24) + 0.5f);
}

static const ParamDescriptorT<int16_t> paramTable[] = {
  // ...
  { PARAM_LFO3_TO_VCF, apply_param_lfo3_to_vcf },
};
```

On mainboard‑style MCU:

```cpp
static void apply_param_lfo3_to_vcf(int32_t v) {
  LFO3toVCFVal = (int16_t)v;
  // optional: update local formulas
  serialSendParamToDCOFunction(PARAM_LFO3_TO_VCF, (int16_t)v);
}

static const ParamDescriptorT<int32_t> paramTable[] = {
  // ...
  { PARAM_LFO3_TO_VCF, apply_param_lfo3_to_vcf },
};
```

The router doesn’t change; the new entry is picked up automatically.

### 5.3. Send the new parameter over serial

From any MCU/tool that sends 16‑bit params:

```cpp
inline void send_lfo3_to_vcf(uint16_t value) {
  uint8_t bytes[5] = {
    (uint8_t)'p',
    (uint8_t)PARAM_LFO3_TO_VCF,
    highByte(value),
    lowByte(value),
    1  // finish
  };
  while (Serial2.availableForWrite() < 5) {}
  Serial2.write(bytes, 5);
}
```

Or use `'x'` + `serialSendParam32ToDCO(...)` if you need 32‑bit values.

---

## 6. Quick checklist for a new MCU

1. **Copy in headers:** `params_def.h`, `param_router.h`, `serial_param_protocol.h`, `serial_protocol.h`, `serial_input_protocol.h` (if needed), `serial_parser.h`.
2. **Create a `params` module:**
   - Implement `apply_param_*` functions.
   - Build a `ParamDescriptorT<ValueT> paramTable[]`.
   - Implement `update_parameters(rawId, value)` using `param_router_apply`.
3. **For each UART link:**
   - Decide which commands it uses (`serial_protocol.h`, `serial_input_protocol.h`, or your own header).
   - Implement `on_frame` handlers.
   - Build a `SerialCommandDef[]` table.
   - Call `serial_parser_process_byte(...)` in that link’s task function.
4. **When adding new parameters or commands:**
   - Update the shared header (`params_def.h` or protocol header).
   - Implement handlers on every MCU that cares.
   - Keep IDs and frame layouts identical across MCUs.

Follow this pattern and all MCUs will share the same serial + parameter “language,” with only local DSP/UI logic differing between boards.


