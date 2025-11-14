void init_ADSR() {
  
  adsrCreateTables(ADSR_1_CC, ARRAY_SIZE);

    for (int i = 0; i < LIN_TO_EXP_TABLE_SIZE; i++) {
    linToLogLookup[i] = linearToLogarithmic(i, 10, maxADSRControlValue);
  }

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);    // initialize attack
    ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);      // initialize decay
    ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);  // initialize sustain
    ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
    ADSRVoices[i].adsr1_voice.setResetAttack(ADSRRestart);
  }
}

inline void ADSR_update() {
  tADSR = millis();
  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    if (noteEnd[i] == 1) {
      // With internal timing, ADSR handles timestamps itself
      ADSRVoices[i].adsr1_voice.noteOff();
      noteEnd[i] = 0;
    } else if (noteStart[i] == 1) {
      ADSRVoices[i].adsr1_voice.noteOff();
      ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);
      ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);
      ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
      ADSRVoices[i].adsr1_voice.noteOn();
      noteStart[i] = 0;
    }
    ADSR1Level[i] = ADSRVoices[i].adsr1_voice.getWave();
  }
  ADSR_set_parameters();
}

inline void ADSR_set_parameters() {
  if ((tADSR - tADSR_params) > 5) {
    // Only push new parameters to the voices when they actually change,
    // so we don't recompute internal scales/divides unnecessarily.
    static uint16_t last_attack  = 0xFFFF;
    static uint16_t last_decay   = 0xFFFF;
    static uint16_t last_sustain = 0xFFFF;
    static uint16_t last_release = 0xFFFF;

    bool attack_changed  = (ADSR1_attack  != last_attack);
    bool decay_changed   = (ADSR1_decay   != last_decay);
    bool sustain_changed = (ADSR1_sustain != last_sustain);
    bool release_changed = (ADSR1_release != last_release);

    if (attack_changed || decay_changed || sustain_changed || release_changed) {
      for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
        if (attack_changed) {
          ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);
        }
        if (decay_changed) {
          ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);
        }
        if (sustain_changed) {
          ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);
        }
        if (release_changed) {
          ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
        }
      }

      last_attack  = ADSR1_attack;
      last_decay   = ADSR1_decay;
      last_sustain = ADSR1_sustain;
      last_release = ADSR1_release;
    }

    tADSR_params = tADSR;
  }
}

void ADSR1_set_restart() {
  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    ADSRVoices[i].adsr1_voice.setResetAttack(ADSRRestart);
  }
}

void ADSR1_change_curves() {
  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    //ADSRVoices[i].adsr1_voice.changeCurves(ADSR_1_DACSIZE, ADSR1_curve1, ADSR1_curve2);
    ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);    // initialize attack
    ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);      // initialize decay
    ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);  // initialize sustain
    ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
    ADSRVoices[i].adsr1_voice.setResetAttack(ADSRRestart);
  }
}
