# Fixed-Point Migration Plan (FixMath)

Reference: Arduino FixMath library docs: `FixMath.h` ([documentation link](https://tomcombriat.github.io/FixMath/docs/html/FixMath_8h.html)).

## Library Overview (Key Points)
- Types:
  - `UFix<NI, NF [, RANGE]>` unsigned fixed-point
  - `SFix<NI, NF [, RANGE]>` signed fixed-point
- Creation & conversions:
  - `toUInt(val)`, `toSInt(val)`, `toUFraction(val)`, `toSFraction(val)`
  - Compile-time constants: `UFixAuto<V>()` (and shift via `.sR<N>()` / `.sL<N>()`)
- Operations: +, -, *, shifts, comparisons supported.
- Critical limitation: division is not implemented. Replace `a/b` with `a * inv(b)` and compute or cache inverses at control rate (outside audio-rate), per docs.

## Global Migration Rules
1. No runtime divisions: precompute reciprocals (or use table/approx) and multiply.
2. Keep audio-rate operations to adds/muls/shifts only.
3. Choose types per signal range and headroom; prefer consistent Q formats across interconnected values.
4. Ensure intermediate products use widened precision (64-bit) when evaluating polynomials or `x*x`.
5. Clamp and round at boundaries; avoid silent wrap/overflow.

## Proposed Fixed-Point Type Aliases (to add later in code)
- Frequency domain (Hz up to ~4000): `using q16_16 = SFix<16,16>;` (alt: `UFix<16,16>` if always positive)
- Small coefficients/modulators (|val| < ~4): `using q2_30 = SFix<2,30>;`
- PWM/levels (0..DIV_COUNTER, integer): `using q16_0 = UFix<16,0>;`
- General fractional (0..1): `using q0_31 = UFix<0,31>;`

Note: These are design targets; the actual typedefs will be added near `globals.h` for shared visibility.

## Mapping Float Sites to Fixed-Point

1) Pitch bend
- Current: `calcPitchbend = ((bend/8190.99) - 1.0) * pitchBendMultiplier;`
- Fixed:
  - Replace `/8190.99` with multiply by `inv_8192 = UFixAuto<1>().sR<13>()` (binary-friendly)
  - Compute: `q = toUInt(midi_pitch_bend) * inv_8192; calc = (q - UFixAuto<1>()) * pitchBendMultiplier_q;`

2) OSC2 detune factor
- Current: `OSC2_detune = 1.0 + 0.0002f * (256 - OSC2DetuneVal);`
- Fixed: `detune = UFixAuto<1>() + detuneScale_q * toUInt(256 - OSC2DetuneVal);` with `detuneScale_q ≈ 0.0002` as `UFix`.

3) Portamento (glide)
- Current: float diffs, divides by `portamento_time`, per-sample lerp.
- Fixed: precompute `interval_q = (stop_q - start_q) * inv_porta_time_q` (control-rate if time changes rarely).
- Audio-rate: `cur_q = start_q + interval_q * elapsed;`

4) Modulations (ADSR, LFO drift, unison)
- Current: sums/products in float.
- Fixed: convert lookup outputs to fixed (e.g., `linToLogLookup` → integer; cast/scale to `q2_30`), keep constants as `UFixAuto` shifted.
- Example: `ADSRModifier_q = toUInt(linToLogLookup[val]).sL<k>() * ADSR1toDETUNE1_q` with chosen scaling `k`.

5) interpolatePitchMultiplier(x)
- Current: `x_float * multiplierTableScale` then integer interpolation.
- Fixed: change signature to accept fixed `x_q` already scaled to the same integer domain as table `xMultiplierTable` to eliminate the float multiply.

6) get_chan_level_lookup(x)
- Current: `y = a*x*x + b*x + c` with float coeffs and `x` in Hz*100.
- Fixed:
  - Pre-scale and store `a,b,c` as fixed (e.g., `SFix<2,30>`). Compute with 64-bit intermediates:
    - `t1 = a_q * (x_q * x_q)`; `t2 = b_q * x_q`; `y_q = (t1 + t2 + c_q)` with proper shifts.
  - Alternatively, precompute integer polynomial coefficients offline for the same integer domain.

7) Clock divider math (`eightSysClock_Hz / freq` and `sysClock_Hz / freq2`)
- Replace `/freq` with `* inv_freq`.
- Compute `inv_freq` per-voice in control loop (or per-note table). Maintain `inv_freq_q` as `UFix` with sufficient precision.

8) ADSR/LFO → PWM modulation
- Replace divisions/mults with fixed products. Predefine `ADSR1toPWM_formula_q`, `LFO2toPWM_formula_q`.
- Keep final `PW_PWM[i]` clamped to integer range.

## Control-Rate vs Audio-Rate Separation
- Control-rate (e.g., `loop0`):
  - Update type-converted parameters
  - Precompute reciprocals: `inv_8192`, `inv_porta_time`, `inv_freq` (tables per voice/note)
  - Update scaled constants (detune scales, ADSR multipliers)
- Audio-rate (`voice_task`):
  - Apply only adds/muls/shifts, table lookups, and PWM writes

## Risks & Mitigations
- Overflow in `x*x`: use widened intermediates (e.g., 64-bit), and scale carefully.
- Precision loss in polynomial: select NF large enough; validate vs float with tests.
- Division removal: ensure every divide site gets a valid inverse updated at control rate.

## Next Steps
- Add FixMath includes and project typedefs
- Introduce constants via `UFixAuto` for common reciprocals and scales
- Refactor `voice_task` in the following order:
  1) Pitch bend, OSC2 detune
  2) Portamento lerp
  3) Modulation sums/products
  4) `interpolatePitchMultiplier` input
  5) Clock divider inverse frequency
  6) Amplitude compensation quadratic
- Bench precision/performance on RP2040

Cited documentation: https://tomcombriat.github.io/FixMath/docs/html/FixMath_8h.html
