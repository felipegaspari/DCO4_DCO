
void init_midi() {
  MIDI_USB.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB.setHandleNoteOn(handleNoteOn);
  MIDI_USB.setHandleNoteOff(handleNoteOff);
  MIDI_USB.setHandleControlChange(handleControlChange);
  MIDI_USB.setHandleProgramChange(handleProgramChange);
  MIDI_USB.setHandlePitchBend(handlePitchBend);


  MIDI_SERIAL.begin(MIDI_CHANNEL_OMNI);
  MIDI_SERIAL.setHandleNoteOn(handleNoteOn);
  MIDI_SERIAL.setHandleNoteOff(handleNoteOff);
  MIDI_SERIAL.setHandleControlChange(handleControlChange);
  MIDI_SERIAL.setHandleProgramChange(handleProgramChange);
  MIDI_SERIAL.setHandlePitchBend(handlePitchBend);
}


void handleNoteOn(byte channel, byte pitch, byte velocity) {
  note_on(pitch, velocity);
}
void handleNoteOff(byte channel, byte pitch, byte velocity) {
  note_off(pitch);
}

void handleControlChange(byte channel, byte number, byte value) {
}

void handleProgramChange(byte channel, byte program) {
}

void handlePitchBend(byte channel, int pitchBend) {
  midi_pitch_bend = pitchBend + 8192;
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
              noteEnd[i] = 0;
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
        noteEnd[voice_num] = 0;
        serial_send_note_on(voice_num, velocity, note - 36 + OSC1_interval);
      }
      break;

    case 2:
      for (int i = 0; i < NUM_VOICES_TOTAL; i++)  // REVISAR!! // Previously NUM_VOICES
      {
        VOICES[i] = 1;
        VOICE_NOTES[i] = note;
        note_on_flag[i] = 1;
        noteStart[i] = 1;
        serial_send_note_on(i, velocity, note - 36 + OSC1_interval);
      }
      break;
    default:
      return;
      break;
  }
  last_midi_pitch_bend = 0;
}

void note_off(uint8_t note) {
  // gate off
  for (int i = 0; i < NUM_VOICES_TOTAL; i++)  // REVISAR!! // Previously NUM_VOICES
  {
    if (VOICE_NOTES[i] == note) {
      // gpio_put(GATE_PINS[i], 0);
      // VOICE_NOTES[i] = 0;
      VOICES[i] = 0;
      VOICES_LAST[i] = note;
      noteEnd[i] = 1;
      noteStart[i] = 0;
      serial_send_note_off(i);
    }
  }
}