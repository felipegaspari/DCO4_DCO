void update_parameters(byte paramNumber, int16_t paramValue) {
  switch (paramNumber) {

    case 10:
      ADSR3ToOscSelect = paramValue;
      break;

    case 11:
      LFO1Waveform = paramValue;
      LFO1_class.setWaveForm(LFO1Waveform);
      break;

    case 12:
      LFO2Waveform = paramValue;
      LFO2_class.setWaveForm(LFO2Waveform);
      break;

    case 13:
      OSC1_interval = paramValue;
      break;

    case 14:
      OSC2_interval = paramValue;
      break;

    case 15:
      OSC2DetuneVal = 512 - paramValue;
      break;

    case 16:
      LFO2toDETUNE2 = (float)expConverterFloat(paramValue, 500) / 275000;
      break;

    case 17:
      oscSync = paramValue;
      break;

    case 18:
      {
        uint8_t portaSerial = paramValue;
        if (portaSerial == 0) {
          portamento_time = 0;
        } else if (portaSerial < 200) {
          portamento_time = (expConverter(portaSerial + 15, 100) * 2000);
        } else {
          portamento_time = map(portaSerial, 200, 255, 1000000, 10000000);
        }
        break;
      }
    case 25:
      // = paramValue " CALIBRATION VAL";
      break;

    case 26:
      voiceMode = paramValue;
      setVoiceMode();
      break;

    case 27:
      unisonDetune = paramValue;
      break;

    case 28:
      analogDrift = paramValue;
      break;

    case 29:
      analogDriftSpeed = paramValue;
      for (int i = 0; i < NUM_OSCILLATORS; i++) {
        LFO_DRIFT_SPEED_OFFSET[i] = (float)(1.00f - (float)((float)analogDriftSpread * 0.005f) + (float)((float)analogDriftSpread * 0.00125 * (float)i)) * (float)expConverterFloat((float)analogDriftSpeed, 5000);
        LFO_DRIFT_CLASS[i].setMode0Freq(LFO_DRIFT_SPEED_OFFSET[i], micros());
      }
      break;

    case 30:
      analogDriftSpread = paramValue;
      for (int i = 0; i < NUM_OSCILLATORS; i++) {
        LFO_DRIFT_SPEED_OFFSET[i] = (float)(1.00f - (float)((float)analogDriftSpread * 0.005f) + (float)((float)analogDriftSpread * 0.00125 * (float)i)) * (float)expConverterFloat((float)analogDriftSpeed, 5000);
        LFO_DRIFT_CLASS[i].setMode0Freq(LFO_DRIFT_SPEED_OFFSET[i], micros());
      }
      break;

    case 31:
      syncMode = paramValue;
      setSyncMode();

    case 40:
      LFO1toDCOVal = paramValue;
      LFO1toDCO = (float)expConverterFloat(LFO1toDCOVal, 500) / 275000;
      break;

    case 41:
      LFO1SpeedVal = paramValue;
      LFO1Speed = expConverterFloat(LFO1SpeedVal, 5000);
      LFO1_class.setMode0Freq((float)LFO1Speed, micros());
      break;

    case 42:
      LFO2SpeedVal = paramValue;
      LFO2Speed = expConverterFloat(LFO2SpeedVal, 5000);
      LFO2_class.setMode0Freq((float)LFO2Speed, micros());
      break;

    case 45:
      LFO2toPW = paramValue;
      LFO2toPW = LFO2toPW / 4;
      LFO2toPWM_formula = (float)1 / 64 * LFO2toPW;
      break;

    case 46:
      ADSR1toPWM = paramValue - 512;
      break;

    case 47:
      ADSR1toDETUNE1 = paramValue;
      ADSR1toDETUNE1_formula = (float)1 / 1080000 * (int16_t)ADSR1toDETUNE1;
      break;

    case 48:
      // = paramValue " ADSR1 Curve";
      break;
    case 49:
      // = paramValue " ADSR2 Curve";      // case 'j':
      //   {
      //     while (Serial2.available() < 1) {}
      //     PWMPotsControlManual = Serial2.read();
      //     break;
      //   }
      break;

    case 124:
      PWMPotsControlManual = paramValue;
      break;

      // case 126:
      //   ADSR3Enabled = paramValue;
      //   break;

    case 127:
      //= paramValue " FUNCTION KEY";
      break;


      // case 101:
      //   = paramValue " CALIB MODE";
      //   break;

      // case 885:
      //   = paramValue " VOICE MODE";
      //   break;
      // case 886:
      //   = paramValue " UNISON DETUNE";
      //   break;

      // case 990:
      //   = paramValue " PW";
      //   break;
      // case 991:
      //   = paramValue " LFO3 Speed";
      //   break;
      // case 992:
      //   = paramValue " LFO3 Shape";
      //   break;
      // case 994:
      //   = paramValue " ADSR3 Restart";
      //   break;
      // case 995:
      //   = paramValue " VCA -> LEVEL";
      //   break;

    default:
      break;
  }
}