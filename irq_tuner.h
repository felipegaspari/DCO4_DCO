#ifndef __IRQ_TUNER_H__
#define __IRQ_TUNER_H__

#define IRQ_TUNER_MAX_SAMPLES 16
volatile uint16_t irqTunerSamples;
volatile uint64_t irqTunerSamplesBuffer[IRQ_TUNER_MAX_SAMPLES];
uint tuner_slice_num;

#define MFPIN 11
#endif
