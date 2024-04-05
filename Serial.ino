void init_serial() {
  // init serial midi
  Serial1.setFIFOSize(256);
  Serial1.setPollingMode(true);

  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(31250);

  Serial2.setFIFOSize(256);
  Serial2.setPollingMode(true);
  Serial2.setRX(21);
  Serial2.setTX(20);
  Serial2.begin(2500000);

  Serial.begin(2000000);
}

void serial_STM32_task() {

  while (Serial2.available() > 0) {

    char commandCharacter = Serial2.read();
    switch (commandCharacter) {
      case 'v':
        {
          while (Serial2.available() < 1) {}
          Serial2.readBytes(dataArray, 4);
          ((uint8_t *)&LFOMultiplier)[0] = dataArray[0];
          ((uint8_t *)&LFOMultiplier)[1] = dataArray[1];
          ((uint8_t *)&LFOMultiplier)[2] = dataArray[2];
          ((uint8_t *)&LFOMultiplier)[3] = dataArray[3];
          break;
        }
        // case 'b':
        //   {
        //     while (Serial2.available() < 1) {}
        //     LFO1Waveform = Serial2.read();
        //     LFO1_class.setWaveForm(LFO1Waveform);
        //     break;
        //   }
        // case 'p':
        //   {
        //     //while (Serial2.available() < 1) {}
        //     // Serial2.readBytes(dataArray, 5);

        //     // ((uint8_t *)&DETUNE2)[0] = dataArray[0];
        //     // ((uint8_t *)&DETUNE2)[1] = dataArray[1];
        //     // ((uint8_t *)&DETUNE2)[2] = dataArray[2];
        //     // ((uint8_t *)&DETUNE2)[3] = dataArray[3];
        //     //OSC2_serial_detune = dataArray[4];

        //     break;
        //   }

      case 'q':
        {
          while (Serial2.available() < 1) {}
          OSC2_serial_detune = Serial2.read();
          break;
        }

      // case 'u':
      //   {
      //     while (Serial2.available() < 1) {}
      //     Serial2.readBytes(dataArray, 2);

      //     ((uint8_t *)&dato_serial)[0] = dataArray[0];
      //     ((uint8_t *)&dato_serial)[1] = dataArray[1];

      //     break;
      //   }

      // case 'a':
      //   {
      //     //Serial.println(" Received ""a"" ");
      //     uint8_t autotuneByte = Serial2.read();
      //     if (autotuneByte == 255) {
      //       autotuneOnFlag = true;
      //       init_DCO_calibration();
      //     } else {
      //       autotuneOnFlag = false;
      //     }
      //     break;
      //   }

      // case 'r':
      //   {
      //     while (Serial2.available() < 1) {}
      //     uint8_t portaSerial = Serial2.read();
      //     if (portaSerial == 0) {
      //       portamento_time = 0;
      //     } else if (portaSerial < 200) {
      //       portamento_time = (expConverter(portaSerial + 15, 100) * 2000);
      //     } else {
      //       portamento_time = map(portaSerial, 200, 255, 1000000, 10000000);
      //     }
      //     break;
      //   }
      case 's':
        {
          while (Serial2.available() < 1) {}
          Serial2.readBytes(dataArray, 4);
          ADSR1_attack = dataArray[0] * 16;
          ADSR1_decay = dataArray[1] * 16;
          ADSR1_sustain = dataArray[2] * 16;
          ADSR1_release = dataArray[3] * 16;
          break;
        }
      // case 'w':
      //   {
      //     while (Serial2.available() < 1) {}
      //     Serial2.readBytes(dataArray, 2);
      //     ((uint8_t *)&ADSR1toDETUNE1)[0] = dataArray[0];
      //     ((uint8_t *)&ADSR1toDETUNE1)[1] = dataArray[1];
      //     ADSR1toDETUNE1_formula = (float)1 / 1080000 * (int16_t)ADSR1toDETUNE1;
      //     break;
      //   }
      // case 't':
      //   {
      //     while (Serial2.available() < 1) {}
      //     oscSync = Serial2.read();
      //     break;
      //   }
      // case 'l':
      //   {
      //     while (Serial2.available() < 1) {}
      //     Serial2.readBytes(dataArray, 2);

      //     ((uint8_t *)&LFO1SpeedVal)[0] = dataArray[0];
      //     ((uint8_t *)&LFO1SpeedVal)[1] = dataArray[1];

      //     LFO1Speed = expConverterFloat(LFO1SpeedVal, 5000);
      //     LFO1_class.setMode0Freq((float)LFO1Speed, micros());

      //     break;
      //   }
      // case 'm':
      //   {
      //     while (Serial2.available() < 1) {}
      //     Serial2.readBytes(dataArray, 2);

      //     ((uint8_t *)&LFO1toDCOVal)[0] = dataArray[0];
      //     ((uint8_t *)&LFO1toDCOVal)[1] = dataArray[1];

      //     //LFO1toDCO = expConverterFloat(LFO1toDCOVal, 500);

      //     LFO1toDCO = (float)expConverterFloat(LFO1toDCOVal, 500) / 275000;

      //     break;
      //   }
      // case 'y':
      //   {
      //     while (Serial2.available() < 1) {}
      //     OSC1_interval = Serial2.read();
      //     break;
      //   }
      // case 'z':
      //   {
      //     while (Serial2.available() < 1) {}
      //     OSC2_interval = Serial2.read();
      //     break;
      //   }
      // case 'c':
      //   {
      //     while (Serial2.available() < 1) {}
      //     ADSR3ToOscSelect = Serial2.read();
      //     break;
      //   }
      // case 'd':
      //   {
      //     while (Serial2.available() < 1) {}
      //     voiceMode = Serial2.read();
      //     setVoiceMode();
      //   }
      // case 'e':
      //   {
      //     while (Serial2.available() < 1) {}
      //     unisonDetune = Serial2.read();
      //     break;
      //   }
      case 'f':
        {
          while (Serial2.available() < 1) {}
          Serial2.readBytes(dataArray, 2);
          ((uint8_t *)&PW[0])[0] = dataArray[0];
          ((uint8_t *)&PW[0])[1] = dataArray[1];
          PW[0] = DIV_COUNTER_PW - (PW[0] / 4);
          break;
        }
        //case 'h':
        // {
        //   while (Serial2.available() < 1) {}
        //   Serial2.readBytes(dataArray, 2);
        //   ((uint8_t *)&LFO2toPW)[0] = dataArray[0];
        //   ((uint8_t *)&LFO2toPW)[1] = dataArray[1];
        //   LFO2toPW = LFO2toPW / 4;
        //   LFO2toPWM_formula = (float)1 / 64 * LFO2toPW;
        //   break;
        // }
        // case 'j':
        //   {
        //     while (Serial2.available() < 1) {}
        //     PWMPotsControlManual = Serial2.read();
        //     break;
        //   }
      case 'p':
        {
          byte paramBytes[3];
          byte finishByte = 1;
          byte readByte = 0;

          while (Serial2.available() < 1) {}

          Serial2.readBytes(paramBytes, 3);

          while (readByte != finishByte) {
            readByte = Serial2.read();
          }

          uint8_t paramNumber = paramBytes[0];
          int16_t paramValue = (int16_t)word(paramBytes[1], paramBytes[2]);

          update_parameters(paramNumber, paramValue);

          break;
        }
      case 'w':
        {
          byte paramBytes[2];
          byte finishByte = 1;
          byte readByte = 0;

          while (Serial2.available() < 1) {}

          Serial2.readBytes(paramBytes, 2);

          while (readByte != finishByte) {
            readByte = Serial2.read();
          }

          uint8_t paramNumber = paramBytes[0];
          int16_t paramValue = paramBytes[1];

          update_parameters(paramNumber, (uint16_t)paramValue);
          break;
        }
    }
  }
}

void serial_send_note_on(uint8_t voice_n, uint8_t note_velo, uint8_t note) {


  byte sendArray[4];
  sendArray[0] = (uint8_t)'n';
  sendArray[1] = voice_n;
  sendArray[2] = note_velo;
  sendArray[3] = note;

  while (Serial2.availableForWrite() < 1) {}
  //Serial2.write((char *)"n");
  //while (Serial2.availableForWrite() < 1) {}
  Serial2.write(sendArray, 4);
}

void serial_send_note_off(uint8_t voice_n) {
  byte sendArray[2] = { (uint8_t)'o', voice_n };
  //while (Serial2.availableForWrite() < 1) {}
  //Serial2.write((char *)"o");
  while (Serial2.availableForWrite() < 1) {}
  Serial2.write(sendArray, 2);
}

void serial_send_generaldata(uint16_t data) {

  if (uart_is_writable(uart1) > 0) {

    uint8_t *b = (uint8_t *)&data;
    uart_putc(uart1, 'w');
    uart_putc_raw(uart1, b[0]);
    uart_putc_raw(uart1, b[1]);
    //uart_putc(uart1, 'z');
    return;
  }
}

void serial_send_freq(float f) {
  //     uint8_t *b = (uint8_t *)&f;
  //     uint8_t* ret = malloc(4 * sizeof(uint8_t));
  // unsigned int asInt = *((int*)&f);

  // int i;
  // for (i = 0; i < 4; i++) {
  //     ret[i] = (asInt >> 8 * i) & 0xFF;
  // }

  // uint8_t *b = (uint8_t *)&f;
  // while (Serial2.availableForWrite() == 0) {}

  // Serial2.write((char *)'f');
  // Serial2.write(b, 2);
  return;
}

// void serial_send_voice_freq() {
//   for (uint8_t i = 0; i < NUM_VOICES; i++) {
//     float f = voiceFreq[i];
//     if (uart_is_writable(uart1) > 0) {
//       uint8_t *b = (uint8_t *)&f;
//       uart_putc(uart1, 'v');
//       uart_putc(uart1, i);
//       uart_putc(uart1, (b[0]));
//       uart_putc(uart1, (b[1]));
//       uart_putc(uart1, (b[2]));
//       uart_putc(uart1, (b[3]));
//       uart_putc(uart1, '6');
//       return;
//     }
//   }
// }
