#ifndef __FS_H__
#define __FS_H__


static const uint16_t FSVoiceDataSize = 22 * 2 * 4;

static const uint16_t FSPWDataSize = 2;

static const uint16_t FSBankSize = FSVoiceDataSize * NUM_OSCILLATORS;
static const uint16_t FSPWBankSize = FSPWDataSize * NUM_VOICES_TOTAL;

static const uint16_t chanLevelVoiceDataSize = FSVoiceDataSize / 4;

byte voiceTablesCalibrationBuffer[FSVoiceDataSize];
byte PWCenterCalibrationBuffer[FSPWDataSize];
byte PWHighLimitCalibrationBuffer[FSPWDataSize];
byte PWLowLimitCalibrationBuffer[FSPWDataSize];

byte voiceTablesBankBuffer[FSBankSize];
byte PWCenterBankBuffer[FSPWBankSize];
byte PWHighLimitBankBuffer[FSPWBankSize];
byte PWLowLimitBankBuffer[FSPWBankSize];

File fileVoiceTablesFS;
File filePWCenterFS;
File filePWHighLimitFS;
File filePWLowLimitFS;

#endif