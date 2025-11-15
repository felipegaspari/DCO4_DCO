//----------------------------------//
// LFO class for Arduino
// by mo-thunderz
// version 1.1
// last update: 29.12.2020
//----------------------------------//

#include "Arduino.h"
#include <stdint.h>

// -------------------------------------------------
// Configuration macros
// -------------------------------------------------
// Configure the sine lookup table resolution:
//   LFO_SINE_TABLE_BITS = log2(table_size)
//   e.g. 8 -> 256 samples, 9 -> 512 samples, 10 -> 1024 samples
// You can override this with a #define before including this header.
#ifndef LFO_SINE_TABLE_BITS
#define LFO_SINE_TABLE_BITS 9
#endif

#define LFO_SINE_TABLE_SIZE (1u << LFO_SINE_TABLE_BITS)

#ifndef lfo_h
#define lfo_h


class lfo
{
    public:
        // constructor
        lfo(int dacSize);

        void setAmpl(int l_ampl);                                                                                           // use this function to set the amplitude from 0 to DACSIZE-1
        void setAmplOffset(int l_ampl_offset);                      // use this function to set the amplitude offset from 0 to DACSIZE-1
        void setWaveForm(int l_waveForm);                           // 0 -> off, 1 -> saw, 2 -> triangle, 3 -> sin, 4 -> square [0,4]
        void setMode(bool l_freq_sync);                             // use this function to set sync to free running (false) or BPM locked (true)
        void setMode0Freq(float l_mode0_freq);                      // set Freq in Hz of free-running mode
        void setMode0Freq(float l_mode0_freq, unsigned long l_t);   // set Freq in Hz of free-running mode, but by adding the current time the freq will be changed without phase jump -> just use micros() as second parameter
        void setMode1Bpm(float l_mode1_bpm);                        // set BPM of track for sync mode
        void setMode1Rate(float l_mode1_rate);                      // l_model_rate represents the lfo cycle duration in quarter notes -> see table at the bottom of this file
        void setMode1Phase(float l_mode1_phase_offset);             // set phase offset for sync mode (free mode does not have phase offset as it is free running, though you can use sync(micros) in free mode to set phase to 0)
        void sync(unsigned long l_t);                               // function to sync LFO to external trigger -> use sync(micros())

        int getWaveForm();                                          // simple get functions as variables are private
        int getAmpl();
        int getAmplOffset();
        bool getMode();
        float getMode0Freq();
        float getMode1Rate();
        float getPhase();                                           // returns relative phase of output signal -> good for triggering LED
        int getWave(unsigned long l_t);                             // main function that gives the waveformshape at time l_t -> use with getWave(micros())

    private:
        int             _dacSize;                           // DAC size
        int             _waveForm = 1;                      // 0 -> off, 1 -> saw, 2 -> triangle, 3 -> sin, 4 -> square [0,4]
        int             _ampl = 0;                          // amplitude, scales from 0 to _dacSize
        int             _ampl_offset = 0;                   // amplitude offset, scales from 0 to _dacSize
        bool            _mode = 0;                          // sync mode: false -> free, true -> synced to track
        float           _mode0_freq = 30;                   // frequency in Hz
        float           _mode1_bpm = 120;                   // BPM of track
        float           _mode1_rate = 1;                    // Rate to link to BPM of track

        // Fixed-point phase accumulator (0..2^32-1 represents 0..1 cycle)
        uint32_t        _phase = 0;

        // Phase increment per microsecond for free-running and synced modes
        uint32_t        _phase_inc_free = 0;
        uint32_t        _phase_inc_sync = 0;

        // Last timestamp used for phase advancement
        unsigned long   _t_last = 0;

        // Internal flag to detect first call to getWave()
        bool            _initialized = false;

        // Internal helpers to compute phase increments
        void            _updatePhaseIncFree();
        void            _updatePhaseIncSync();
};

#endif

// -----------------------------------------------------
// Additional explanation to function:
//
// void setMode1Rate(float l_mode1_rate);   
// -> relevant only if _mode is 1
// 
//
//l_mode1_rate | lfo cycle duration
//---------------------------------
//         .125|   2 bars
//         .25 |   1 bar
//         .5  |   half note
//         1   |   quarter note
//         2   |   1/8 note
//         3   |   1/12 note
//         4   |   1/16 note
//         5   |   1/20 note
//         6   |   1/24 note
//         7   |   1/28 note
//         8   |   1/32 note
//         9   |   1/36 note
//        10   |   1/40 note
//        11   |   1/44 note
//        12   |   1/48 note
//        13   |   1/52 note
//        14   |   1/56 note
//        15   |   1/60 note
//        16   |   1/64 note
//
// Big THANKS to othmar52 for providing the table :-)
