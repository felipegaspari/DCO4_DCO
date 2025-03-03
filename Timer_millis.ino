inline void millisTimer()
{

  timer99microsFlag = 0;
  timer223microsFlag = 0;
  timer1msFlag = 0;
  // timer2340microsFlag = 0;
  // timer3543microsFlag = 0;
  // timer5msFlag = 0;
  // timer11msFlag = 0;
  // timer23msFlag = 0;
  // timer31msFlag = 0;
  // timer67msFlag = 0;
  timer200msFlag = 0;
  //  timer500msFlag = 0;

  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();

  if (currentMicros - timer99micros > 99)
  {
    timer99micros = currentMicros;
    timer99microsFlag = 1;
  }

  if (currentMicros - timer223micros > 223)
  {
    timer223micros = currentMicros;
    timer223microsFlag = 1;
  }

  if ( currentMicros - timer1ms > 1001) {
    timer1ms = currentMicros;
    timer1msFlag = 1;
  }

  // if ( currentMicros - timer2340micros > 2340) {
  //   timer2340micros = currentMicros;
  //   timer2340microsFlag = 1;
  // }

  // if ( currentMicros - timer3543micros > 3543) {
  //   timer3543micros = currentMicros;
  //   timer3543microsFlag = 1;
  // }

  // if ( currentMillis - timer5ms > 5) {
  //   timer5ms = currentMillis;
  //   timer5msFlag = 1;
  // }

  // if ( currentMillis - timer11ms > 11) {
  //   timer11ms = currentMillis;
  //   timer11msFlag = 1;
  // }

  // if ( currentMillis - timer23ms > 23) {
  //   timer23ms = currentMillis;
  //   timer23msFlag = 1;
  // }

  // if ( currentMillis - timer31ms > 31) {
  //   timer31ms = currentMillis;
  //   timer31msFlag = 1;
  // }

  // if ( currentMillis - timer67ms > 67) {
  //   timer67ms = currentMillis;
  //   timer67msFlag = 1;
  // }

  if (currentMillis - timer200ms > 200)
  {
    timer200ms = currentMillis;
    timer200msFlag = 1;
  }

  //    if ( currentMillis - timer500ms > 500) {
  //    timer500ms = currentMillis;
  //    timer500msFlag = 1;
  //  }
}