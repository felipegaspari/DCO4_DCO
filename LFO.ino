void init_LFOs() {
  init_LFO1();
  init_LFO2();
}

void init_DRIFT_LFOs() {
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    init_DRIFT_LFO(LFO_DRIFT_CLASS[i], LFO_DRIFT_CC, i);
  }
}

void init_DRIFT_LFO(lfo &LFO, int CC, byte LFONumber) {
  LFO_DRIFT_SPEED_OFFSET[LFONumber] = (float)(1.00f - (float)((float)analogDriftSpread * 0.005) + (float)((float)analogDriftSpread * 0.00125f * (float)LFONumber)) * (float)expConverterFloat((float)analogDriftSpeed, 5000);
  LFO.setWaveForm(LFO_DRIFT_WAVEFORM);                  // inicializar forma de onda
  LFO.setAmpl(CC);                                      // establecer amplitud máxima
  LFO.setAmplOffset(0);                                 // sin offset a la amplitud
  LFO.setMode(0);                                       // establecer modo de sincronización a modo0 -> sin sincronización a BPM
  //LFO.setMode0Freq(LFO_DRIFT_SPEED_OFFSET[LFONumber]);  // establecer LFO a 0.5 Hz
  LFO.setMode0Freq(LFO_DRIFT_SPEED_OFFSET[LFONumber], micros());
}

void init_LFO1() {
  LFO1_class.setWaveForm(LFO1Waveform);  // initialize waveform
  LFO1_class.setAmpl(LFO1_CC);           // set amplitude to maximum
  LFO1_class.setAmplOffset(0);           // no offset to the amplitude
  LFO1_class.setMode(0);                 // set sync mode to mode0 -> no sync to BPM
  LFO1_class.setMode0Freq(0.5);          // set LFO to 30 Hz
}


void init_LFO2() {
  LFO2_class.setWaveForm(2);    // initialize waveform
  LFO2_class.setAmpl(LFO2_CC);  // set amplitude to maximum
  LFO2_class.setAmplOffset(0);  // no offset to the amplitude
  LFO2_class.setMode(0);        // set sync mode to mode0 -> no sync to BPM
  LFO2_class.setMode0Freq(5);   // set LFO to 30 Hz
}

inline void LFO1() {
  //tLFO1 = micros();                                     // take timestamp
  //LFO1Level = LFO1_CC_HALF - LFO1_class.getWave(micros());
  LFO1Level = LFO1_class.getWave(micros()) - LFO1_CC_HALF;
  // Produce detune modulation directly in Q24 fixed-point:
  // detune_q24 = (LFO1Level * LFO1toDCO) * 2^24
  DETUNE_INTERNAL_q24 = (int32_t)LFO1Level * LFO1toDCO_q24;
}

inline void LFO2() {
  //tLFO1 = micros();                                     // take timestamp
  //LFO1Level = LFO1_CC_HALF - LFO1_class.getWave(micros());
  LFO2Level = LFO2_class.getWave(micros()) - LFO2_CC_HALF;
  //PW_MOD = (float)((float)LFO2Level * LFO2toPW);
  // Produce OSC2-only detune modulation in Q24 fixed-point:
  // detune2_q24 = (LFO2Level * LFO2toDETUNE2) * 2^24
  DETUNE_INTERNAL2_q24 = (int32_t)LFO2Level * LFO2toDETUNE2_q24;
}

inline void DRIFT_LFOs() {
  unsigned long currentMicros = micros();
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    LFO_DRIFT_LEVEL[i] = LFO_DRIFT_CC_HALF - LFO_DRIFT_CLASS[i].getWave(currentMicros);
  }
}