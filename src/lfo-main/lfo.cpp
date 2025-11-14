//----------------------------------//
// LFO class for Arduino
// by mo-thunderz
// version 1.1
// last update: 22.05.2021
//----------------------------------//


#include "Arduino.h"
#include "lfo.h"
#include <math.h>

// -----------------------------------------------
// Internal helpers (file-local)
// -----------------------------------------------

// Convert frequency in Hz to phase increment per microsecond for 32-bit phase
static uint32_t lfo_compute_phase_inc_from_freq(float freq_hz)
{
    if (freq_hz <= 0.0f)
        return 0;

    // 2^32 / 1'000'000  (cycles -> phase ticks per microsecond)
    const double scale = 4294.967296; // ~= (double)UINT32_MAX + 1 / 1e6
    double v = (double)freq_hz * scale;
    if (v < 0.0)
        v = 0.0;
    if (v > 4294967295.0)
        v = 4294967295.0;

    return (uint32_t)(v + 0.5);
}

// Simple sine lookup table, initialized once at runtime
static int16_t s_sineTable[LFO_SINE_TABLE_SIZE];
static bool s_sineTableInitialized = false;

static void lfo_initSineTable()
{
    if (s_sineTableInitialized)
        return;

    for (int i = 0; i < (int)LFO_SINE_TABLE_SIZE; ++i)
    {
        double angle = (2.0 * 3.14159265358979323846 * (double)i) / (double)LFO_SINE_TABLE_SIZE;
        s_sineTable[i] = (int16_t)lrint(sin(angle) * 32767.0);
    }

    s_sineTableInitialized = true;
}

// -----------------------------------------------
// lfo class implementation
// -----------------------------------------------

lfo::lfo(int dacSize)
{
    _dacSize      = dacSize;
    _ampl         = dacSize - 1;
    _ampl_offset  = 0;

    // Initialize sine lookup table (once)
    lfo_initSineTable();

    // Precompute phase increments for default parameters
    _updatePhaseIncFree();
    _updatePhaseIncSync();
}

void lfo::setWaveForm(int l_waveForm)
{
    if(l_waveForm < 0)
        l_waveForm = 0;
    if(l_waveForm > 4)
        l_waveForm = 4;
    _waveForm = l_waveForm;
}

void lfo::setAmpl(int l_ampl)
{
    if(l_ampl < 0)
        l_ampl = 0;
    if(l_ampl >= _dacSize)
        l_ampl = _dacSize - 1;
    _ampl = l_ampl;
}

void lfo::setAmplOffset(int l_ampl_offset)
{
    if(l_ampl_offset < 0)
        l_ampl_offset = 0;
    if(l_ampl_offset >= _dacSize)
        l_ampl_offset = _dacSize - 1;
    _ampl_offset = l_ampl_offset;
}

void lfo::setMode(bool l_mode)
{
    _mode = l_mode;
}

void lfo::setMode0Freq(float l_mode0_freq)
{
    if(l_mode0_freq < 0)
        l_mode0_freq = 0;
    _mode0_freq = l_mode0_freq;
    _updatePhaseIncFree();
}

void lfo::setMode0Freq(float l_mode0_freq, unsigned long l_t)
{
    (void)l_t; // timestamp ignored for speed; no phase continuity on freq change

    if(l_mode0_freq < 0)
        l_mode0_freq = 0;

    _mode0_freq = l_mode0_freq;
    _updatePhaseIncFree();
}

void lfo::setMode1Bpm(float l_mode1_bpm)
{
    if(l_mode1_bpm < 0)
        l_mode1_bpm = 0;
    _mode1_bpm = l_mode1_bpm;
    _updatePhaseIncSync();
}

void lfo::setMode1Rate(float l_mode1_rate)
{
    if(l_mode1_rate < 0)
        l_mode1_rate = 0;
    _mode1_rate = l_mode1_rate;
    _updatePhaseIncSync();
}

void lfo::setMode1Phase(float l_mode1_phase_offset)
{
    (void)l_mode1_phase_offset; // phase offset not used in this configuration
}

void lfo::sync(unsigned long l_t)
{
    // Reset timestamp and phase to zero (no phase offset handling)
    _t_last      = l_t;
    _initialized = true;
    _phase       = 0;
}

int lfo::getWaveForm()
{
    return _waveForm;
}

int lfo::getAmpl()
{
    return _ampl;
}

int lfo::getAmplOffset()
{
    return _ampl_offset;
}

bool lfo::getMode()
{
    return _mode;
}

float lfo::getMode0Freq()
{
    return _mode0_freq;
}

float lfo::getMode1Rate()
{
    return _mode1_rate;
}

float lfo::getPhase()
{
    // Return normalized phase in [0,1)
    return (float)_phase * (1.0f / 4294967296.0f);
}

int lfo::getWave(unsigned long l_t)
{
    int result = 0;
    int l_ampl = _ampl;

    // Initialize timestamp on first call
    if (!_initialized)
    {
        _t_last = l_t;
        _initialized = true;
    }

    // Advance phase based on elapsed time in microseconds
    uint32_t dt = (uint32_t)(l_t - _t_last);
    _t_last = l_t;

    uint32_t phase_inc = _mode ? _phase_inc_sync : _phase_inc_free;
    _phase += phase_inc * dt; // natural 32-bit wrap-around

    // Compute correct _ampl_offsetoffset (wave not to exceed 0 to dacSize)
    int l_ampl_offset = 0;
    int l_ampl_half   = (int) l_ampl / 2;
    if(_ampl_offset < _dacSize / 2)
        l_ampl_offset = ( _ampl_offset > l_ampl_half ) ? _ampl_offset : l_ampl_half;                                                // l_ampl_offset must be large enough not to go below 0
    else
        l_ampl_offset = ( _dacSize - _ampl_offset > l_ampl_half ) ? _ampl_offset : _dacSize - l_ampl_half - 1;

    // 16-bit ramp derived from phase (0..65535)
    uint16_t ramp16 = (uint16_t)(_phase >> 16);

    switch(_waveForm)
    {
        case 0: // Off
            result = _ampl_offset;
            return result;
            break;
        case 1: // Saw
        {
            // Descending saw from +ampl/2 to -ampl/2
            uint32_t inv_ramp = (uint32_t)(0xFFFFu - ramp16);              // 0..65535
            int32_t scaled    = (int32_t)(inv_ramp * (uint32_t)l_ampl);    // up to ~2.8e9, but we only care modulo >> 16
            scaled >>= 16;                                                 // 0..l_ampl
            result = (int)scaled + l_ampl_offset - l_ampl_half;
            return result;
        }
            break;
        case 2: // Triangle
        {
            // Triangle from -ampl/2 to +ampl/2 using folded ramp
            uint16_t tri16 = (ramp16 & 0x8000u) ? (uint16_t)(0xFFFFu - ramp16) : ramp16; // 0..32767
            int32_t scaled = (int32_t)tri16 * (int32_t)l_ampl;
            scaled >>= 15;                                                                // 0..l_ampl
            result = (int)scaled + l_ampl_offset - l_ampl_half;
            return result;
        }
            break;
        case 3: // Sin
        {
            // Sine using lookup table: value in [-32767, 32767]
            // Use top LFO_SINE_TABLE_BITS bits of ramp16 as index
            uint16_t idx     = (uint16_t)(ramp16 >> (16 - LFO_SINE_TABLE_BITS));
            int16_t s        = s_sineTable[idx];
            int32_t scaled   = (int32_t)l_ampl_half * (int32_t)s;
            scaled >>= 15;                            // scale back to amplitude
            result = (int)scaled + l_ampl_offset;
            return result;
        }
            break;
        case 4: // Square
        {
            // MSB of ramp16 selects polarity
            if (ramp16 & 0x8000u)
                result = l_ampl_offset - l_ampl_half;
            else
                result = l_ampl_offset + l_ampl_half;
            return result;
        }
            break;
    }
    return result;
}

// -----------------------------------------------
// Internal helpers
// -----------------------------------------------

void lfo::_updatePhaseIncFree()
{
    _phase_inc_free = lfo_compute_phase_inc_from_freq(_mode0_freq);
}

void lfo::_updatePhaseIncSync()
{
    // Effective LFO frequency in Hz in sync mode:
    // frequency = _mode1_rate * _mode1_bpm / 60  (same as original implementation)
    float freq_hz = 0.0f;
    if (_mode1_rate > 0.0f && _mode1_bpm > 0.0f)
        freq_hz = (_mode1_rate * _mode1_bpm) / 60.0f;

    _phase_inc_sync = lfo_compute_phase_inc_from_freq(freq_hz);
}
