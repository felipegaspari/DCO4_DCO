void init_FS() {
  LittleFS.begin();

  if (!LittleFS.exists("voiceTables")) {
    fileVoiceTablesFS = LittleFS.open("voiceTables", "w+");
  } else {
    fileVoiceTablesFS = LittleFS.open("voiceTables", "r");
  }

#ifdef ENABLE_FS_CALIBRATION
  for (int i = 0; i < 20; i++) {
    delay(200);
    Serial.println("A");
  }
  fileVoiceTablesFS.read(voiceTablesBankBuffer, FSBankSize);
  fileVoiceTablesFS.close();

  //for (int i = 0; i < NUM_OSC / 4; i++) {

// size_t const u32Size = sizeof(uint32_t);
// for (size_t i = 0; i < sizeof(voiceTablesBankBuffer) / u32Size; i++) {
//   uint8_t tmp[u32Size];
//   for (size_t j = 0; j < u32Size; j++) {
//     tmp[j] = voiceTablesBankBuffer[u32Size * i + u32Size - j - 1];
//   }
//   memcpy(&freq_to_amp_comp_array[i], &tmp, u32Size);

//      delay(60);
//       Serial.println(freq_to_amp_comp_array[i]);
//     delay(70);
// }

  for (size_t  i = 0; i < 44 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
    uint32_t int32Data;
    for (int j = 0; j < 4; j++) {
      ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
    }
    

    freq_to_amp_comp_array[i] = (int_fast32_t)int32Data;
    //Serial.println(freq_to_amp_comp_array[i]);
    //delay(20);
  }

  Serial.println("B1");

    for (size_t  i = 44; i < 88 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
    uint32_t int32Data;
    for (int j = 0; j < 4; j++) {
      ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
    }
    freq_to_amp_comp_array[i] = (int_fast32_t)int32Data;
    Serial.println(freq_to_amp_comp_array[i]);
    delay(20);
  }

  Serial.println("B2");

      for (size_t  i = 88; i < 132 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
    uint32_t int32Data;
    for (int j = 0; j < 4; j++) {
      ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
    }

    freq_to_amp_comp_array[i] = (int_fast32_t)int32Data;
    Serial.println(freq_to_amp_comp_array[i]);
    delay(20);
  }

  Serial.println("B3");

        for (size_t  i = 132; i < 176 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
    uint32_t int32Data;
    for (int j = 0; j < 4; j++) {
      ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
    }
  //  Serial.println(int32Data);
    //delay(20);
    freq_to_amp_comp_array[i] = (int_fast32_t)int32Data;
     Serial.println(freq_to_amp_comp_array[i]);
    //delay(20);
  }

    Serial.println("B4");

        for (size_t  i = 176; i < 220 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
    uint32_t int32Data;
    for (int j = 0; j < 4; j++) {
      ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
    }
    //Serial.println(int32Data);
    //delay(20);
    freq_to_amp_comp_array[i] = (int_fast32_t)int32Data;
     Serial.println(freq_to_amp_comp_array[i]);
    //delay(20);
  }

      Serial.println("B5");

        for (size_t  i = 220; i < 264 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
    uint32_t int32Data;
    for (int j = 0; j < 4; j++) {
      ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
    }
    //Serial.println(int32Data);
    //delay(20);
    freq_to_amp_comp_array[i] = (int_fast32_t)int32Data;
     Serial.println(freq_to_amp_comp_array[i]);
    //delay(20);
  }

        Serial.println("B6");

        for (size_t  i = 264; i < 308 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
    uint32_t int32Data;
    for (int j = 0; j < 4; j++) {
      ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
    }
    //Serial.println(int32Data);
    //delay(20);
    freq_to_amp_comp_array[i] = (int_fast32_t)int32Data;
     Serial.println(freq_to_amp_comp_array[i]);
    //delay(20);
  }

          Serial.println("B7");

        for (size_t  i = 308; i < 352 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
    uint32_t int32Data;
    for (int j = 0; j < 4; j++) {
      ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
    }
    //Serial.println(int32Data);
    //delay(20);
    freq_to_amp_comp_array[i] = (int_fast32_t)int32Data;
     Serial.println(freq_to_amp_comp_array[i]);
    //delay(20);
  }

  // for (int i = 0; i < 88 /*chanLevelVoiceDataSize * NUM_OSCILLATORS*/; i++) {
  //   uint32_t int32Data;
  //   for (int j = 0; j < 4; j++) {
  //     ((uint8_t *)&int32Data)[j] = voiceTablesBankBuffer[i * 4 + j];
  //   }
  // //  Serial.println(int32Data);
  //   delay(70);
  //   freq_to_amp_comp_array[i] = int32Data;
  //   delay(60);
  // }

  // delay(1000);
  // Serial.println("D");

  uint8_t highestNoteFound = 255;
  // for (int i = 0; i < NUM_OSCILLATORS; i++) {
  //   highestOSCNote[i] =
  //     if (highestOSCNote[i] < highestNoteFound) {
  //     highestNoteFound = highestOSCNote[i];
  //   }
  // }


#endif
}

void update_FS_voice(byte voiceN) {
  byte calibrationDataBytes[FSVoiceDataSize];

  for (int i; i < chanLevelVoiceDataSize; i++) {
    //freq_to_amp_comp_array[i + (voiceN * chanLevelVoiceDataSize) ] = calibrationData[i];
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