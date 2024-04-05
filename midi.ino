void init_midi() {
  MIDI_USB.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB.setHandleNoteOn(handleNoteOn);
  MIDI_USB.setHandleNoteOff(handleNoteOff);

  MIDI_SERIAL.begin(MIDI_CHANNEL_OMNI);
  MIDI_SERIAL.setHandleNoteOn(handleNoteOn);
  MIDI_SERIAL.setHandleNoteOff(handleNoteOff);
}

void handleNoteOn(byte channel, byte pitch, byte velocity) {
  note_on(pitch, velocity);
}
void handleNoteOff(byte channel, byte pitch, byte velocity) {
  note_off(pitch);
}

void note_on(uint8_t note, uint8_t velocity) {

  if (note > highestNote) {
    note = note - ((uint8_t(note - 108) / 12) * 12);
  }

  switch (voiceMode) {
    case 0:
      VOICE_NOTES[0] = note;
      VOICES[0] = millis();
      note_on_flag[0] = 1;
      noteStart[0] = 1;
      serial_send_note_on(0, velocity, note - 36 + OSC1_interval);
      return;

      break;

    case 1:

      if (polyMode == 0) {
        if (STACK_VOICES < 2) {
          for (int i = 0; i < NUM_VOICES; i++)  // REVISAR!!
          {
            if (VOICE_NOTES[i] == note) {
              VOICES[i] = millis();
              note_on_flag[i] = 1;
              noteStart[i] = 1;
              serial_send_note_on(i, velocity, note - 36 + OSC1_interval);
              return;  // note already playing
            }
          }
        }

        for (int i = 0; i < STACK_VOICES; i++) {  // REVISAR!! Quizas debiera ser NUM_VOICES y no STACK
          uint8_t voice_num = get_free_voice();
          VOICES[voice_num] = millis();
          VOICE_NOTES[voice_num] = note;
          note_on_flag[voice_num] = 1;
          noteStart[voice_num] = 1;
          serial_send_note_on(voice_num, velocity, note - 36 + OSC1_interval);
        }
      }

      if (polyMode == 1) {
        if (STACK_VOICES < 2) {
          for (int i = 0; i < NUM_VOICES; i++)  // REVISAR!!
          {
            if (VOICE_NOTES[i] == note) {
              VOICES[i] = 1;
              note_on_flag[i] = 1;
              noteStart[i] = 1;
              serial_send_note_on(i, velocity, note - 36 + OSC1_interval);
              return;  // note already playing
            }
          }
        }

        uint8_t voice_num = get_free_voice_sequential();
        VOICES[voice_num] = 1;
        VOICE_NOTES[voice_num] = note;
        note_on_flag[voice_num] = 1;
        noteStart[voice_num] = 1;
        serial_send_note_on(voice_num, velocity, note - 36 + OSC1_interval);
      }
      break;

    case 2:
      for (int i = 0; i < NUM_VOICES; i++)  // REVISAR!!
      {
        VOICES[i] = 1;
        VOICE_NOTES[i] = note;
        note_on_flag[i] = 1;
        noteStart[i] = 1;
        serial_send_note_on(i, velocity, note - 36 + OSC1_interval);
      }
      break;
  }


  last_midi_pitch_bend = 0;
  //voice_task(); // Only if ran on the same core
}

void note_off(uint8_t note) {
  // gate off
  for (int i = 0; i < NUM_VOICES; i++)  // REVISAR!!
  {
    if (VOICE_NOTES[i] == note) {
      // gpio_put(GATE_PINS[i], 0);
      // VOICE_NOTES[i] = 0;
      VOICES[i] = 0;
      VOICES_LAST[i] = note;
      noteEnd[i] = 1;
      serial_send_note_off(i);
    }
  }
  // if (portamento_stop == note) {
  //   portamento_start = portamento_stop;
  //   portamento_stop = 0;
  //   portamento_cur_freq = 0.0f;
  // }

  // if (portamento_start == note) {
  //   portamento_stop = 0;
  //   portamento_cur_freq = 0.0f;
  // }
}

void serial_midi_task() {
  // if (!uart_is_readable(uart0))
  //   return;

  // uint8_t lsb = 0, msb = 0;
  // uint8_t data = uart_getc(uart0);

  // LED_BLINK_START = millis();
  // digitalWrite(LED_BUILTIN, HIGH);

  // // status
  // if (data >= 0xF0 && data <= 0xF7) {
  //   midi_serial_status = 0;
  //   return;
  // }

  // // realtime message
  // if (data >= 0xF8 && data <= 0xFF) {
  //   return;
  // }

  // if (data >= 0x80 && data <= 0xEF) {
  //   midi_serial_status = data;
  // }

  // if (midi_serial_status >= 0x80 && midi_serial_status <= 0x90 || midi_serial_status >= 0xB0 && midi_serial_status <= 0xBF ||  // cc messages
  //     midi_serial_status >= 0xE0 && midi_serial_status <= 0xEF) {
  //   lsb = uart_getc(uart0);
  //   msb = uart_getc(uart0);
  // }

  // if (midi_serial_status == (0x90 | (MIDI_CHANNEL - 1))) {
  //   if (msb > 0) {
  //     note_on(lsb, msb);
  //   } else {
  //     note_off(lsb);
  //   }
  // }

  // if (midi_serial_status == (0x80 | (MIDI_CHANNEL - 1))) {
  //   note_off(lsb);
  // }

  // if (midi_serial_status == (0xE0 | (MIDI_CHANNEL - 1))) {
  //   midi_pitch_bend = lsb | (msb << 7);
  // }

  // if (midi_serial_status == (0xB0 | (MIDI_CHANNEL - 1))) {
  //   if (lsb == 5) {  // portamento time
  //     portamento_time = msb;
  //   }
  //   if (lsb == 65) {  // portamento on/off
  //     portamento = msb > 63;
  //   }
  // }
}

void usb_midi_task() {
  // if (tud_midi_available() < 4)
  //   return;

  // uint8_t buff[4];

  // LED_BLINK_START = millis();
  // digitalWrite(LED_BUILTIN, HIGH);

  // if (tud_midi_packet_read(buff)) {
  //   if (buff[1] == (0x90 | (MIDI_CHANNEL - 1))) {
  //     if (buff[3] > 0) {
  //       note_on(buff[2], buff[3]);
  //     } else {
  //       note_off(buff[2]);
  //     }
  //   }

  //   if (buff[1] == (0x80 | (MIDI_CHANNEL - 1))) {
  //     note_off(buff[2]);
  //   }

  //   if (buff[1] == (0xE0 | (MIDI_CHANNEL - 1))) {
  //     midi_pitch_bend = buff[2] | (buff[3] << 7);
  //   }

  //   if (midi_serial_status == (0xB0 | (MIDI_CHANNEL - 1))) {
  //     if (buff[2] == 5) {  // portamento time
  //       portamento_time = buff[3];
  //     }
  //     if (buff[2] == 65) {  // portamento on/off
  //       portamento = buff[3] > 63;
  //     }
  //   }
  // }
}