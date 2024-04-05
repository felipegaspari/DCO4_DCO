//void init_tuner() {
//  
//
//    // Only the PWM B pins can be used as inputs.
//    assert(pwm_gpio_to_channel(MFPIN) == PWM_CHAN_B);
//    tuner_slice_num = pwm_gpio_to_slice_num(MFPIN);
//
//    // Count once for every 100 cycles the PWM B input is high
//    pwm_config cfg = pwm_get_default_config();
//    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
//    pwm_config_set_clkdiv(&cfg, 1.f); //set by default, increment count for each rising edge
//    pwm_init(tuner_slice_num, &cfg, false);  //false means don't start pwm
//    gpio_set_function(MFPIN, GPIO_FUNC_PWM);
//
//}
//
//void tuner_callback(uint gpio, uint32_t events) {
//
//  if (irqTunerSamples < IRQ_TUNER_MAX_SAMPLES) {
//irqTunerSamplesBuffer[irqTunerSamples] = time_us_64();
//irqTunerSamples++;
////Serial.println(time_us_64());
////Serial.println(irqTunerSamples);
//  }
//
//}
//
//void irq_tuner() {
//  irqTunerSamples = 0;
//  //gpio_set_irq_enabled_with_callback(16, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &tuner_callback);
//  gpio_set_irq_enabled_with_callback(16, GPIO_IRQ_EDGE_FALL, true, &tuner_callback);
//while (irqTunerSamples < IRQ_TUNER_MAX_SAMPLES) {
//}
//  gpio_set_irq_enabled_with_callback(16, 0x04, 0, &tuner_callback);
// //gpio_set_irq_enabled_with_callback(16, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false, &tuner_callback);
//irqTunerSamples = 0;
//char str[21];
////Serial.println( uintToStr( irqTunerSamplesBuffer[0], str ) );
//float result = (float)1 / (irqTunerSamplesBuffer[3] - irqTunerSamplesBuffer[2]) * 1000000;
//Serial.println(irqTunerSamplesBuffer[3] - irqTunerSamplesBuffer[2]);
//Serial.println(result,2);
//
//}
//
//float measure_frequency(uint gpio) {
//
//    
//    pwm_set_enabled(tuner_slice_num, true);
//    sleep_ms(10);
//    pwm_set_enabled(tuner_slice_num, false);
//    
//    uint16_t counter = (uint16_t) pwm_get_counter(tuner_slice_num);
//    float freq =   counter / 10.f;
//    Serial.println((uint16_t)freq);
//    return freq;
//
//}
