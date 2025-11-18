#include "include_all.h"
void init_FS() {
  LittleFS.begin();

  if (!LittleFS.exists("voiceTables")) {
    fileVoiceTablesFS = LittleFS.open("voiceTables", "w+");
  } else {
    fileVoiceTablesFS = LittleFS.open("voiceTables", "r");
  }

#ifdef ENABLE_FS_CALIBRATION

  fileVoiceTablesFS.read(voiceTablesBankBuffer, FSBankSize);
  fileVoiceTablesFS.close();


    for (int i = 0; i < (chanLevelVoiceDataSize * NUM_OSCILLATORS); i++) {
     freq_to_amp_comp_array[i] = (int32_t(voiceTablesBankBuffer[i * 4 + 3]) << 24) |
                    (int32_t(voiceTablesBankBuffer[i * 4 + 2]) << 16) |
                    (int32_t(voiceTablesBankBuffer[i * 4 + 1]) << 8) |
                    int32_t(voiceTablesBankBuffer[i * 4 ]);
  }

    for (int datasetIndex = 0; datasetIndex < NUM_OSCILLATORS; ++datasetIndex) {
        for (int pairIndex = 0; pairIndex < chanLevelVoiceDataSize / 2; ++pairIndex) {
            int rawIndex = datasetIndex * chanLevelVoiceDataSize + pairIndex * 2;

        // Stored frequencies are in Hz*100.
            int32_t freq_x100 = freq_to_amp_comp_array[rawIndex];

#ifdef USE_FLOAT_AMP_COMP
        // Float engine: convert directly to Hz for the pure-float amp-comp path.
        float freqHz = (float)freq_x100 / 100.0f;
        ampCompFrequencyHz[datasetIndex][pairIndex] = freqHz;
        // Level is shared between fixed and float paths.
        ampCompArray[datasetIndex][pairIndex] = freq_to_amp_comp_array[rawIndex + 1];
#else
        // Fixed-point engine: convert to fixed-point Hz (Hz * 2^FREQ_FRAC_BITS).
        int64_t scaled = (int64_t)freq_x100 * (1LL << FREQ_FRAC_BITS);  // use 64-bit to avoid overflow
            int32_t freq_fx = (scaled >= 0)
                            ? (int32_t)((scaled + 50LL) / 100LL)        // round to nearest
                                : (int32_t)(-((( -scaled) + 50LL) / 100LL));
            ampCompFrequencyArray[datasetIndex][pairIndex] = freq_fx;
        ampCompArray[datasetIndex][pairIndex]          = freq_to_amp_comp_array[rawIndex + 1];
#endif
        }
    }


  uint8_t highestNoteFound = 255;
  // for (int i = 0; i < NUM_OSCILLATORS; i++) {
  //   highestOSCNote[i] =
  //     if (highestOSCNote[i] < highestNoteFound) {
  //     highestNoteFound = highestOSCNote[i];
  //   }
  // }

  // PW CALIBRATION VALUES VALUES FROM FS
  // PW_CENTER
  if (!LittleFS.exists("PWCenter")) {
    filePWCenterFS = LittleFS.open("PWCenter", "w+");
  } else {
    filePWCenterFS = LittleFS.open("PWCenter", "r");
  }

  filePWCenterFS.read(PWCenterBankBuffer, FSPWBankSize);
  filePWCenterFS.close();

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    uint16_t uint16Data;
    for (int j = 0; j < FSPWDataSize; j++) {
      ((uint8_t *)&uint16Data)[j] = PWCenterBankBuffer[i * 2 + j];
    }

    PW_CENTER[i] = (uint16_t)uint16Data;

    // delay(1000);
    // Serial.println((String) "PW_CENTER " + i + (String) ": " + uint16Data);
  }
  // PW_HIGH_LIMIT
  if (!LittleFS.exists("PWHighLimit")) {
    filePWHighLimitFS = LittleFS.open("PWHighLimit", "w+");
  } else {
    filePWHighLimitFS = LittleFS.open("PWHighLimit", "r");
  }

  filePWHighLimitFS.read(PWHighLimitBankBuffer, FSPWBankSize);
  filePWHighLimitFS.close();

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    uint16_t uint16Data;
    for (int j = 0; j < FSPWDataSize; j++) {
      ((uint8_t *)&uint16Data)[j] = PWHighLimitBankBuffer[i * 2 + j];
    }
    PW_HIGH_LIMIT[i] = (uint16_t)uint16Data;

    // // Debug:
    // // Serial.println((String) "PW_HIGH_LIMIT " + i + (String) ": " + uint16Data);
  }
  // PW_LOW_LIMIT
  if (!LittleFS.exists("PWLowLimit")) {
    filePWLowLimitFS = LittleFS.open("PWLowLimit", "w+");
  } else {
    filePWLowLimitFS = LittleFS.open("PWLowLimit", "r");
  }

  filePWLowLimitFS.read(PWLowLimitBankBuffer, FSPWBankSize);
  filePWLowLimitFS.close();

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    uint16_t uint16Data;
    for (int j = 0; j < FSPWDataSize; j++) {
      ((uint8_t *)&uint16Data)[j] = PWLowLimitBankBuffer[i * 2 + j];
    }
    PW_LOW_LIMIT[i] = (uint16_t)uint16Data;

    // delay(1000);
    // Serial.println((String) "PW_LOW_LIMIT " + i + (String) ": " + uint16Data);
  }

#endif

  //singleFileDrive.begin("voiceTables", "voicetables.txt");
}

void update_FS_voice(byte voiceN) {
  byte calibrationDataBytes[FSVoiceDataSize];

  // Serialize calibrationData (uint32_t pairs: [freq_x100, pwm]) for this voice
  // into a contiguous byte buffer. Each entry is written little-endian.

  for (int i = 0; i < chanLevelVoiceDataSize; i++) {
    // freq_to_amp_comp_array[i + (voiceN * chanLevelVoiceDataSize)] = calibrationData[i]; // can be used for in-RAM updates if desired
    byte *b = (byte *)&calibrationData[i];
    for (int j = 0; j < 4; j++) {
      calibrationDataBytes[i * 4 + j] = b[j];
    }
  }
  uint16_t startByteN = voiceN * FSVoiceDataSize;

  fileVoiceTablesFS = LittleFS.open("voiceTables", "r+");
  fileVoiceTablesFS.seek(startByteN);
  fileVoiceTablesFS.write(calibrationDataBytes, FSVoiceDataSize);
  fileVoiceTablesFS.close();
}


void update_FS_PWCenter(byte voiceN, uint16_t value) {
  byte calibrationDataBytes[FSPWDataSize];
  byte *b = (byte *)&value;

  uint16_t startByteN = voiceN * FSPWDataSize;

  filePWCenterFS = LittleFS.open("PWCenter", "r+");
  filePWCenterFS.seek(startByteN);
  filePWCenterFS.write(b, FSPWDataSize);
  filePWCenterFS.close();
}

void update_FS_PW_High_Limit(byte voiceN, uint16_t value) {
  byte calibrationDataBytes[FSPWDataSize];
  byte *b = (byte *)&value;

  uint16_t startByteN = voiceN * FSPWDataSize;

  filePWCenterFS = LittleFS.open("PWHighLimit", "r+");
  filePWCenterFS.seek(startByteN);
  filePWCenterFS.write(b, FSPWDataSize);
  filePWCenterFS.close();
}

void update_FS_PW_Low_Limit(byte voiceN, uint16_t value) {
  byte calibrationDataBytes[FSPWDataSize];
  byte *b = (byte *)&value;

  uint16_t startByteN = voiceN * FSPWDataSize;

  filePWCenterFS = LittleFS.open("PWLowLimit", "r+");
  filePWCenterFS.seek(startByteN);
  filePWCenterFS.write(b, FSPWDataSize);
  filePWCenterFS.close();
}
