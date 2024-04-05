void init_pio()
{

  offset[0] = pio_add_program(pio[0], &frequency_program);
  offset[1] = pio_add_program(pio[1], &frequency_program);

  for (int i = 0; i < (NUM_VOICES * 2); i++) // REVISAR !!
  {
    init_sm(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], offset[VOICE_TO_PIO[i]], RESET_PINS[i]);
    // init_sm(pio[VOICE_TO_PIO[i + 1]], VOICE_TO_SM[i + 1], offset[VOICE_TO_PIO[i + 1]], RESET_PINS[i + 1]);

    pio_sm_put(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], pioPulseLength);
    // pio_sm_put(pio[VOICE_TO_PIO[i + 1]], VOICE_TO_SM[i + 1], pioPulseLength);

    pio_sm_exec(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], pio_encode_pull(false, false));
    // pio_sm_exec(pio[VOICE_TO_PIO[i + 1]], VOICE_TO_SM[i + 1], pio_encode_pull(false, false));

    pio_sm_exec(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], pio_encode_out(pio_y, 31));
    // pio_sm_exec(pio[VOICE_TO_PIO[i + 1]], VOICE_TO_SM[i + 1], pio_encode_out(pio_y, 31));
  }

  // pio_enable_sm_mask_in_sync(pio1, (1 << 0) + (1 << 1));

  // pio_sm_put(pio1, 0, pioPulseLength);
  // pio_sm_put(pio1, 1, pioPulseLength);
  // pio_sm_exec(pio1, 0, pio_encode_pull(false, false));
  // pio_sm_exec(pio1, 1, pio_encode_pull(false, false));
  // pio_sm_exec(pio1, 0, pio_encode_out(pio_y, 31));
  // pio_sm_exec(pio1, 1, pio_encode_out(pio_y, 31));
}

void init_sm(PIO pio, uint sm, uint offset, uint pin)
{
  init_sm_pin(pio, sm, offset, pin);
  pio_sm_set_enabled(pio, sm, true);
}

void set_frequency(PIO pio, uint sm, float freq)
{
  uint32_t clk_div = sysClock_Hz / freq;
  if (freq == 0)
    clk_div = 0;
  pio_sm_put(pio, sm, clk_div);
  pio_sm_exec(pio, sm, pio_encode_pull(false, false));
  pio_sm_exec(pio, sm, pio_encode_out(pio_osr, 31));
}