void init_LFOs() {
  init_LFO1();
  init_LFO2();
}

void init_LFO1() {
  LFO1_class.setWaveForm(LFO1Waveform);     // initialize waveform
  LFO1_class.setAmpl(LFO1_CC);   // set amplitude to maximum
  LFO1_class.setAmplOffset(0);   // no offset to the amplitude
  LFO1_class.setMode(0);         // set sync mode to mode0 -> no sync to BPM
  LFO1_class.setMode0Freq(0.5);  // set LFO to 30 Hz
}


void init_LFO2() {
  LFO2_class.setWaveForm(2);              // initialize waveform
  LFO2_class.setAmpl(LFO2_CC);                     // set amplitude to maximum
  LFO2_class.setAmplOffset(0);                       // no offset to the amplitude
  LFO2_class.setMode(0);                             // set sync mode to mode0 -> no sync to BPM
  LFO2_class.setMode0Freq(5);                       // set LFO to 30 Hz
}

void LFO1() {
  //tLFO1 = micros();                                     // take timestamp
  //LFO1Level = LFO1_CC_HALF - LFO1_class.getWave(micros());
  LFO1Level = LFO1_class.getWave(micros()) - 500;
  DETUNE_INTERNAL = (float)((float)LFO1Level * LFO1toDCO);
}

void LFO2() {
  //tLFO1 = micros();                                     // take timestamp
  //LFO1Level = LFO1_CC_HALF - LFO1_class.getWave(micros());
  LFO2Level = LFO2_class.getWave(micros()) - 500;
  //PW_MOD = (float)((float)LFO2Level * LFO2toPW);
}