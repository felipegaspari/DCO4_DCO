void init_ADSR() {
  for (int i = 0; i < NUM_VOICES; i++) {
    ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);    // initialize attack
    ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);      // initialize decay
    ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);  // initialize sustain
    ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
    ADSRVoices[i].adsr1_voice.setResetAttack(ADSRRestart);
  }
}

void ADSR_update() {
  tADSR = micros();
  for (int i = 0; i < NUM_VOICES; i++) {
    if (noteEnd[i] == 1) {
      ADSRVoices[i].adsr1_voice.noteOff(tADSR - 1);
      noteEnd[i] = 0;
    } else if (noteStart[i] == 1) {
      ADSRVoices[i].adsr1_voice.noteOff(tADSR - 1);
      ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack * 1000);
      ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay * 1000);
      ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release * 1000);
      ADSRVoices[i].adsr1_voice.noteOn(tADSR);
      noteStart[i] = 0;
    }
    tADSR = micros();
    ADSR1Level[i] = ADSRVoices[i].adsr1_voice.getWave(tADSR);
  }
  ADSR_set_parameters();
}

void ADSR_set_parameters() {
  if ((tADSR - tADSR_params) > 5000) {
    for (int i = 0; i < NUM_VOICES; i++) {
      ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);
    }
    tADSR_params = tADSR;
  }
}

void ADSR1_set_restart() {
  for (int i = 0; i < NUM_VOICES; i++) {
    ADSRVoices[i].adsr1_voice.setResetAttack(ADSRRestart);
  }
}

void ADSR1_change_curves() {
  for (int i = 0; i < NUM_VOICES; i++) {
    ADSRVoices[i].adsr1_voice.changeCurves(ADSR_1_DACSIZE, ADSR1_curve1, ADSR1_curve2);
    ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);    // initialize attack
    ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);      // initialize decay
    ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);  // initialize sustain
    ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
    ADSRVoices[i].adsr1_voice.setResetAttack(ADSRRestart);
  }
}
