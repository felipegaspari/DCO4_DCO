#ifndef __FS_H__
#define __FS_H__

static const uint16_t FSVoiceDataSize = 22 * 2 * 4;

static const uint16_t FSBankSize = FSVoiceDataSize * NUM_OSCILLATORS;

static const uint16_t chanLevelVoiceDataSize  = FSVoiceDataSize / 4;

byte voiceTablesCalibrationBuffer[FSVoiceDataSize];

byte voiceTablesBankBuffer[FSBankSize];

File fileVoiceTablesFS;

#endif