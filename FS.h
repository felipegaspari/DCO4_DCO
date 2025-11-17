#ifndef __FS_H__
#define __FS_H__

static constexpr uint16_t FSVoiceDataSize = 22 * 2 * 4;

static constexpr uint16_t FSPWDataSize = 2;

static constexpr uint16_t FSBankSize = FSVoiceDataSize * NUM_OSCILLATORS;
static constexpr uint16_t FSPWBankSize = FSPWDataSize * NUM_VOICES_TOTAL;

static constexpr uint16_t chanLevelVoiceDataSize = FSVoiceDataSize / 4;

// Calibration buffers (FS-local)
uint8_t voiceTablesCalibrationBuffer[FSVoiceDataSize];
uint8_t PWCenterCalibrationBuffer[FSPWDataSize];
uint8_t PWHighLimitCalibrationBuffer[FSPWDataSize];
uint8_t PWLowLimitCalibrationBuffer[FSPWDataSize];

uint8_t voiceTablesBankBuffer[FSBankSize];
uint8_t PWCenterBankBuffer[FSPWBankSize];
uint8_t PWHighLimitBankBuffer[FSPWBankSize];
uint8_t PWLowLimitBankBuffer[FSPWBankSize];

File fileVoiceTablesFS;
File filePWCenterFS;
File filePWHighLimitFS;
File filePWLowLimitFS;

#endif