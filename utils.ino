long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

char * uintToStr( const uint64_t num, char *str )
{
  uint8_t i = 0;
  uint64_t n = num;
  
  do
    i++;
  while ( n /= 10 );
  
  str[i] = '\0';
  n = num;
 
  do
    str[--i] = ( n % 10 ) + '0';
  while ( n /= 10 );

  return str;
}


uint16_t faderExpConverter(uint16_t readingValue) {
  uint16_t pow3Calc = readingValue / 4;
  uint16_t expValOut = pow3Calc * pow3Calc * pow3Calc / 20000;
  return expValOut;
}

float expConverterFloat(uint16_t readingValue, uint16_t curve) {
  uint16_t pow3Calc = readingValue;
  float expValOut = (float)pow3Calc * pow3Calc / curve;
  if (expValOut < 0.005) {
    expValOut = 0;
  }
  return expValOut;
}

uint16_t expConverter(uint16_t readingValue, uint16_t curve) {
  uint16_t pow3Calc = readingValue;
  uint16_t expValOut = (float)pow3Calc * pow3Calc / curve;
  if (expValOut < 0.1) {
    expValOut = 0;
  }
  return expValOut;
}

uint16_t expConverterReverse(uint16_t readingValue, uint16_t curve) {
  uint16_t expValOut = sqrt((float)readingValue / curve);
  return expValOut;
}

uint16_t expConverterFloatReverse(float readingValue, uint16_t curve) {
  uint16_t expValOut = sqrt(readingValue / curve);
  return expValOut;
}

uint16_t expConverter2(uint16_t readingValue, uint16_t curve) {
  uint16_t pow3Calc = readingValue;
  uint16_t expValOut = pow3Calc * pow3Calc * pow3Calc / curve;
  return expValOut;
}

//1 VCFKeytrack
//2 ADSR2toVCF
//3 LFO1toVCF
//4 LFO2toVCF
//5 ADSR3toPWM
//6 LFO1toPWM
//7 LFO1toVCA
//8 ADSR1toVCA
//9 LFO1toDCO
//10 ADSR3toDETUNE1

void formula_update(byte formulaN) {
  // switch (formulaN) {
  //   case 1:
  //     if (VCFKeytrack > 0) {
  //       VCFKeytrackModifier = (float)VCFKeytrack / 8000;
  //     } else {
  //       VCFKeytrackModifier = 1;
  //     }
  //     break;
  //   case 2:
  //     ADSR2toVCF_formula = (float)1 / 512 * ADSR2toVCF;
  //     break;
  //   case 3:
  //     LFO1toVCF_formula = (float)1 / 512 * LFO1toVCF;
  //     break;
  //   case 4:
  //     LFO2toVCF_formula = (float)1 / 512 * LFO2toVCF;
  //     break;
  //   case 5:
  //     ADSR3toPWM_formula = (float)1 / 512 * ADSR3toPWM;
  //     break;
  //   case 6:
  //     LFO1toPWM_formula = (float)1 / 512 * LFO1toPWM;
  //     break;
  //   case 7:
  //     LFO1toVCA_formula = (float)1 / 512 * LFO1toVCA;
  //     break;
  //   case 8:
  //     ADSR1toVCA_formula = (float)1 / 512 * ADSR1toVCA;
  //     break;
  //   case 9:
  //     LFO1toDCO_formula = (float)1 / 1080000 * LFO1toDCO;
  //     break;
  //   case 10:
  //     ADSR3toDETUNE1_formula = (float)1 / 1080000 * ADSR3toDETUNE1;
  //     break;
  //   case 11:
  //   LFO2toPWM_formula = (float)1 / 512 * LFO2toPWM;
  //   break;
  // }
}

void controls_formula_update(byte formulaN) {
  switch (formulaN) {
    case 1:
      LFO1Speed = expConverterFloat(LFO1SpeedVal, 5000);
      //LFO1_class.setMode0Freq(LFO1Speed, micros());
      break;
          case 2:
      LFO2Speed = expConverterFloat(LFO2SpeedVal, 5000);
      //LFO2_class.setMode0Freq(LFO2Speed, micros());
      break;
      case 3:
      LFO1toDCO = expConverterFloat(LFO1toDCOVal, 500);
      break;
  }
}


void led_blinking_task() {
  if (millis() - LED_BLINK_START < 50)
    return;
  digitalWrite(LED_BUILTIN, LOW);
}