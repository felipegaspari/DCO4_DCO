void init_pwm()
{
  for (int i = 0; i < NUM_VOICES * 2; i++)
  {
    gpio_set_function(RANGE_PINS[i], GPIO_FUNC_PWM);
    RANGE_PWM_SLICES[i] = pwm_gpio_to_slice_num(RANGE_PINS[i]);
    pwm_set_wrap(RANGE_PWM_SLICES[i], DIV_COUNTER);
    pwm_set_enabled(RANGE_PWM_SLICES[i], true);
  }
  // For VCO:
  // gpio_set_function(22, GPIO_FUNC_PWM);
  // VCO_PWM_SLICES[0] = pwm_gpio_to_slice_num(22);
  // pwm_set_wrap(VCO_PWM_SLICES[0], 10000);
  // pwm_set_enabled(VCO_PWM_SLICES[0], true);
}