#ifndef __BENCH_H__
#define __BENCH_H__

// Realtime profiler for the DCO hot path.
//
// Enabled by RUNNING_AVERAGE in DCO4_DCO.ino; every macro below compiles to nothing when it is
// off, so the shipping build carries no cost. RUNNING_AVERAGE_FINE additionally enables the
// probes around the smallest stages (a few multiplies each) — useful for A/B-ing how much
// the instrumentation itself distorts the totals, since every probe is an optimisation
// barrier the compiler cannot move work across.
//
// Time source: SysTick, read as a free-running 24-bit down-counter clocked from clk_sys.
// RP2040's Cortex-M0+ has no DWT cycle counter, so SysTick is the only single-cycle source
// available on both chips. Resolution is one clk_sys cycle (4.4 ns at 225 MHz).
//
// Range: 24 bits wraps after 2^24 cycles — ~74.5 ms at 225 MHz, ~126 ms at 133 MHz. Anything
// that can outlast that must use BENCH_PERIOD(), which reads the 1 us timer instead. The
// report flags any probe whose max lands near the wrap so a silent overflow is visible.

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pico/platform.h"
#include "hardware/clocks.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/timer.h"

// Defined in voices.ino; prints the CLKDIV_BENCHMARK comparison when that flag is on.
void print_clkdiv_bench();

// Set to 0 to fall back to the 1 us timer for every probe, e.g. if SysTick turns out to be
// claimed by the core (a FreeRTOS build uses it; plain arduino-pico drives millis()/micros()
// from time_us_64() and leaves it alone).
#ifndef BENCH_USE_SYSTICK
#define BENCH_USE_SYSTICK 1
#endif

// ---------------------------------------------------------------------------
// Probe table — the single source of truth.
//
// Storage, enum ids, core ownership, parent/child nesting and the printed labels are all
// generated from this list, so a probe cannot exist in one place and be missing in another.
// Adding a probe means adding one line here plus the BENCH_BEGIN/BENCH_END pair at the site.
//
//   X(id, core, kind, tier, parent, label)
// ---------------------------------------------------------------------------
#define BENCH_PROBES(X)                                                                       \
  /* --- Core 0: loop() --- */                                                                \
  X(loop0_period,      0, BENCH_US,  BENCH_T_MAIN, BENCH_NONE,          "loop period")         \
  X(loop0_io,          0, BENCH_CYC, BENCH_T_MAIN, BENCH_loop0_period,  "MIDI+serial+LFO1")    \
  X(loop0_lfo2,        0, BENCH_CYC, BENCH_T_MAIN, BENCH_loop0_period,  "LFO2")                \
  X(loop0_drift,       0, BENCH_CYC, BENCH_T_MAIN, BENCH_loop0_period,  "drift LFOs")          \
  X(loop0_fifo_push,   0, BENCH_CYC, BENCH_T_MAIN, BENCH_loop0_period,  "FIFO push")           \
  /* --- Core 1: loop1() --- */                                                               \
  X(loop1_period,      1, BENCH_US,  BENCH_T_MAIN, BENCH_NONE,          "loop1 period")        \
  X(loop1_millis,      1, BENCH_CYC, BENCH_T_MAIN, BENCH_loop1_period,  "millisTimer")         \
  X(loop1_adsr,        1, BENCH_CYC, BENCH_T_MAIN, BENCH_loop1_period,  "ADSR_update")         \
  X(loop1_fifo_pop,    1, BENCH_CYC, BENCH_T_MAIN, BENCH_loop1_period,  "FIFO pop")            \
  X(voice_task,        1, BENCH_CYC, BENCH_T_MAIN, BENCH_loop1_period,  "voice_task TOTAL")    \
  /* --- Core 1: inside voice_task --- */                                                     \
  X(vt_pitchbend,      1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "pitch bend")          \
  X(vt_osc_detune,     1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "OSC2/3 detune")       \
  X(vt_portamento,     1, BENCH_CYC, BENCH_T_MAIN, BENCH_voice_task,    "portamento")          \
  X(vt_adsr_mod,       1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "ADSR modifier")       \
  X(vt_unison_mod,     1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "unison modifier")     \
  X(vt_drift_mod,      1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "drift modifier")      \
  X(vt_modifiers,      1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "modifier sum")        \
  X(vt_freq_scale_x,   1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "table x scaling")     \
  X(vt_ratio_interp,   1, BENCH_CYC, BENCH_T_MAIN, BENCH_voice_task,    "ratio interpolate")   \
  X(vt_freq_scale_post,1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "apply ratio")         \
  X(vt_clk_div,        1, BENCH_CYC, BENCH_T_MAIN, BENCH_voice_task,    "clkdiv math")         \
  X(vt_phase_align,    1, BENCH_CYC, BENCH_T_MAIN, BENCH_voice_task,    "phase align")         \
  X(vt_chan_level,     1, BENCH_CYC, BENCH_T_MAIN, BENCH_voice_task,    "amp comp")            \
  X(vt_pio_write,      1, BENCH_CYC, BENCH_T_MAIN, BENCH_voice_task,    "PIO put/exec")        \
  X(vt_note_retrig,    1, BENCH_CYC, BENCH_T_MAIN, BENCH_voice_task,    "note-on retrigger")   \
  X(vt_pwm_calc,       1, BENCH_CYC, BENCH_T_FINE, BENCH_voice_task,    "PW arithmetic")       \
  X(vt_pw_update,      1, BENCH_CYC, BENCH_T_MAIN, BENCH_voice_task,    "PW level + write")

enum BenchId {
#define BENCH_X(id, core, kind, tier, parent, label) BENCH_##id,
  BENCH_PROBES(BENCH_X)
#undef BENCH_X
  BENCH_COUNT
};
#define BENCH_NONE BENCH_COUNT

// Probe unit. BENCH_CYC values are clk_sys cycles, BENCH_US values are microseconds.
#define BENCH_CYC 0
#define BENCH_US  1

// Probe tier. FINE probes only collect when RUNNING_AVERAGE_FINE is defined.
#define BENCH_T_MAIN 0
#define BENCH_T_FINE 1

// ---------------------------------------------------------------------------
// Time source. Kept outside the RUNNING_AVERAGE guard so CLKDIV_BENCHMARK can use it
// on its own; with no probes compiled in, none of this is referenced and none of it
// costs anything at runtime.
// ---------------------------------------------------------------------------

// Cost of one back-to-back bench_now() pair, subtracted from every cycle sample.
uint32_t bench_overhead_cyc = 0;
uint32_t bench_cycles_per_us = 1;

// Wall-clock span each core's current set of samples was collected over, used for the
// "share of the core's time" column.
uint32_t bench_window_start_us[2] = { 0, 0 };
uint32_t bench_window_us[2] = { 0, 0 };

#if BENCH_USE_SYSTICK
#define BENCH_TIMER_MASK 0x00FFFFFFu
// SysTick counts down, so the earlier reading is the larger one.
static inline uint32_t bench_now(void) {
  return systick_hw->cvr;
}
static inline uint32_t bench_span(uint32_t start, uint32_t end) {
  return (start - end) & BENCH_TIMER_MASK;
}
#else
#define BENCH_TIMER_MASK 0xFFFFFFFFu
static inline uint32_t bench_now(void) {
  return timer_hw->timerawl;
}
static inline uint32_t bench_span(uint32_t start, uint32_t end) {
  return end - start;
}
#endif

// Free-running 1 us reading for the loop-period probes, which can outlast the 24-bit wrap.
static inline uint32_t bench_us_now(void) {
  return timer_hw->timerawl;
}

// Per core, at boot. SysTick is a core-local peripheral: core 0 and core 1 each have their
// own, and arming one does nothing for the other.
inline void bench_init_core() {
  bench_cycles_per_us = clock_get_hz(clk_sys) / 1000000u;
  if (bench_cycles_per_us == 0u) bench_cycles_per_us = 1u;

#if BENCH_USE_SYSTICK
  systick_hw->csr = 0u;
  systick_hw->rvr = BENCH_TIMER_MASK;
  systick_hw->cvr = 0u;
  systick_hw->csr = 0x5u;  // ENABLE | CLKSOURCE=processor, interrupt off
#endif

  // Minimum of a run of back-to-back reads: the cleanest estimate of the probe's own cost,
  // since any larger sample picked up an interrupt. Both cores compute the same value.
  uint32_t best = 0xFFFFFFFFu;
  for (int i = 0; i < 32; ++i) {
    const uint32_t a = bench_now();
    const uint32_t b = bench_now();
    const uint32_t d = bench_span(a, b);
    if (d < best) best = d;
  }
  bench_overhead_cyc = best;

#ifdef RUNNING_AVERAGE
  bench_window_start_us[get_core_num() & 1u] = bench_us_now();
#endif
}

#ifdef RUNNING_AVERAGE

struct BenchDesc {
  uint8_t core;
  uint8_t kind;
  uint8_t tier;
  uint8_t parent;
  const char *label;
};

static const BenchDesc bench_desc[BENCH_COUNT] = {
#define BENCH_X(id, core, kind, tier, parent, label) { core, kind, tier, parent, label },
  BENCH_PROBES(BENCH_X)
#undef BENCH_X
};

// 20 bytes per probe. The whole table costs less than a tenth of what one
// RunningAverage(2000) buffer did, and the add is integer-only — which matters on the
// RP2040, where every float operation is an __aeabi_* library call.
struct BenchStat {
  uint32_t n;
  uint32_t min;
  uint32_t max;
  uint64_t sum;
};

BenchStat bench_stats[BENCH_COUNT];
BenchStat bench_snap[BENCH_COUNT];

// Cross-core report handshake. Each core snapshots and clears its own probes, so no lock is
// needed and neither core ever reads a counter the other is mid-update on.
volatile bool bench_dump_request = false;
volatile bool bench_core_ready[2] = { false, false };
volatile bool bench_periodic = true;

// Report is formatted into RAM then drained in small Serial.write slices so loop() is not
// blocked for milliseconds. While active, BENCH_PERIOD on both cores skips samples.
volatile bool bench_out_active = false;

#define BENCH_OUT_CAP   6144
#define BENCH_OUT_CHUNK 128

char bench_out_buf[BENCH_OUT_CAP];
uint16_t bench_out_len = 0;
uint16_t bench_out_pos = 0;

inline void bench_out_reset() {
  bench_out_len = 0;
  bench_out_pos = 0;
}

inline void bench_out_puts(const char *s) {
  while (*s != '\0' && bench_out_len + 1u < BENCH_OUT_CAP) {
    bench_out_buf[bench_out_len++] = *s++;
  }
}

inline void bench_out_printf(const char *fmt, ...) {
  char tmp[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  bench_out_puts(tmp);
}

inline void bench_out_println(const char *s) {
  bench_out_puts(s);
  bench_out_puts("\n");
}

static inline void bench_stat_add(BenchStat *s, uint32_t d) {
  s->sum += d;
  if (s->n == 0u || d < s->min) s->min = d;
  if (d > s->max) s->max = d;
  s->n++;
}

// Close a cycle probe. Reads the counter first so none of the bookkeeping below lands
// inside the measured span.
static inline void bench_add_cyc(uint8_t id, uint32_t start) {
  const uint32_t d = bench_span(start, bench_now());
  bench_stat_add(&bench_stats[id], (d > bench_overhead_cyc) ? (d - bench_overhead_cyc) : 0u);
}

static inline void bench_add_raw(uint8_t id, uint32_t d) {
  bench_stat_add(&bench_stats[id], d);
}

#define BENCH_BEGIN(id) const uint32_t bench_t_##id = bench_now()
#define BENCH_END(id)   bench_add_cyc(BENCH_##id, bench_t_##id)

// Interval between successive arrivals. While a report is draining over USB, only refresh
// the previous timestamp — never record — so dump TX cannot inflate period max on either core.
#define BENCH_PERIOD(id)                                                  \
  do {                                                                    \
    static uint32_t bench_prev_##id = 0u;                                 \
    const uint32_t bench_us_##id = bench_us_now();                        \
    if (bench_prev_##id != 0u && !bench_out_active) {                     \
      bench_add_raw(BENCH_##id, bench_us_##id - bench_prev_##id);         \
    }                                                                     \
    bench_prev_##id = bench_us_##id;                                      \
  } while (0)

#ifdef RUNNING_AVERAGE_FINE
#define BENCH_FBEGIN(id) BENCH_BEGIN(id)
#define BENCH_FEND(id)   BENCH_END(id)
#else
#define BENCH_FBEGIN(id) ((void)0)
#define BENCH_FEND(id)   ((void)0)
#endif

// Snapshot and clear this core's probes, then hand off to the printer on core 0.
inline void bench_service(uint8_t core) {
  if (!bench_dump_request || bench_core_ready[core]) return;

  const uint32_t now_us = bench_us_now();
  bench_window_us[core] = now_us - bench_window_start_us[core];
  bench_window_start_us[core] = now_us;

  for (uint8_t i = 0; i < BENCH_COUNT; ++i) {
    if (bench_desc[i].core != core) continue;
    bench_snap[i] = bench_stats[i];
    bench_stats[i] = BenchStat{};
  }
  bench_core_ready[core] = true;
}

inline void bench_reset_all() {
  for (uint8_t i = 0; i < BENCH_COUNT; ++i) {
    bench_stats[i] = BenchStat{};
    bench_snap[i] = BenchStat{};
  }
  bench_window_start_us[0] = bench_window_start_us[1] = bench_us_now();
}

// Raw probe value -> microseconds, as hundredths, without touching the FPU. Keeping this
// integer matters on RP2040 where float is emulated in software.
inline uint64_t bench_to_us100(uint64_t raw, uint8_t kind) {
  if (kind == BENCH_US) return raw * 100u;
  return ((raw * 100u) + (bench_cycles_per_us / 2u)) / bench_cycles_per_us;
}

inline void bench_fmt_us(char *out, size_t n, uint64_t raw, uint8_t kind) {
  const uint64_t h = bench_to_us100(raw, kind);
  snprintf(out, n, "%lu.%02lu", (unsigned long)(h / 100u), (unsigned long)(h % 100u));
}

inline void bench_fmt_cyc(char *out, size_t n, uint64_t raw) {
  snprintf(out, n, "%luc", (unsigned long)raw);
}

// Fixed columns so nested indents and large totals stay under the header.
// name(28) count(8) mean(9) min(9) max(9) total(14) win(6)
#define BENCH_ROW_FMT "%-28s %8s %9s %9s %9s %14s %6s"

inline void bench_fmt_win(char *out, size_t n, uint32_t permille) {
  snprintf(out, n, "%lu.%lu",
           (unsigned long)(permille / 10u), (unsigned long)(permille % 10u));
}

inline void bench_print_row(uint8_t id, const char *indent) {
  const BenchDesc &d = bench_desc[id];
  const BenchStat &s = bench_snap[id];
  if (s.n == 0u) return;

  char name[32], mean[16], mn[16], mx[16], tot[20], cnt[12], win[8], line[160];
  snprintf(name, sizeof(name), "%s%s", indent, d.label);
  snprintf(cnt, sizeof(cnt), "%lu", (unsigned long)s.n);

  const uint64_t mean_raw = s.sum / s.n;
  // Sub-microsecond cycle probes print as 0.00 us forever at 225 MHz hundredths; show
  // mean/min/max in cycles instead. total and %win stay in us so the budget still adds.
  const bool as_cyc = (d.kind == BENCH_CYC) && (bench_to_us100(mean_raw, BENCH_CYC) < 100u);
  if (as_cyc) {
    bench_fmt_cyc(mean, sizeof(mean), mean_raw);
    bench_fmt_cyc(mn, sizeof(mn), s.min);
    bench_fmt_cyc(mx, sizeof(mx), s.max);
  } else {
    bench_fmt_us(mean, sizeof(mean), mean_raw, d.kind);
    bench_fmt_us(mn, sizeof(mn), s.min, d.kind);
    bench_fmt_us(mx, sizeof(mx), s.max, d.kind);
  }
  bench_fmt_us(tot, sizeof(tot), s.sum, d.kind);

  const uint32_t window = bench_window_us[d.core];
  const uint64_t total_us = bench_to_us100(s.sum, d.kind) / 100u;
  const uint32_t permille = (window > 0u) ? (uint32_t)((total_us * 1000u) / window) : 0u;
  bench_fmt_win(win, sizeof(win), permille);

  snprintf(line, sizeof(line), BENCH_ROW_FMT, name, cnt, mean, mn, mx, tot, win);
  bench_out_println(line);

  // ~70% of the 24-bit range (~52 ms at 225 MHz). Past that the span is almost certainly
  // a wrap, not a real stage duration — say so rather than printing a plausible lie.
  if (d.kind == BENCH_CYC && s.max > ((BENCH_TIMER_MASK * 7u) / 10u)) {
    bench_out_println("                             ^^ max near 24-bit wrap: use BENCH_PERIOD");
  }
}

// Whatever the parent measured that none of its children claimed. Makes gaps in coverage
// visible instead of quietly absorbing them.
inline void bench_print_unattributed(uint8_t parent, const char *indent) {
  const BenchStat &p = bench_snap[parent];
  if (p.n == 0u) return;

  uint64_t children_us100 = 0;
  bool any = false;
  for (uint8_t i = 0; i < BENCH_COUNT; ++i) {
    if (bench_desc[i].parent != parent || bench_snap[i].n == 0u) continue;
    children_us100 += bench_to_us100(bench_snap[i].sum, bench_desc[i].kind);
    any = true;
  }
  if (!any) return;

  const uint64_t parent_us100 = bench_to_us100(p.sum, bench_desc[parent].kind);
  const uint64_t rest = (parent_us100 > children_us100) ? (parent_us100 - children_us100) : 0u;
  const uint32_t window = bench_window_us[bench_desc[parent].core];
  const uint32_t permille = (window > 0u) ? (uint32_t)(((rest / 100u) * 1000u) / window) : 0u;

  char name[32], tot[20], win[8], line[160];
  snprintf(name, sizeof(name), "%s%s", indent, "(unattributed)");
  snprintf(tot, sizeof(tot), "%lu.%02lu",
           (unsigned long)(rest / 100u), (unsigned long)(rest % 100u));
  bench_fmt_win(win, sizeof(win), permille);
  snprintf(line, sizeof(line), BENCH_ROW_FMT, name, "-", "-", "-", "-", tot, win);
  bench_out_println(line);
}

inline void bench_print_core(uint8_t core) {
  char line[160];
  snprintf(line, sizeof(line), "-- Core %u  (window %lu.%03lu ms) --", (unsigned)core,
           (unsigned long)(bench_window_us[core] / 1000u),
           (unsigned long)(bench_window_us[core] % 1000u));
  bench_out_println(line);
  snprintf(line, sizeof(line), BENCH_ROW_FMT,
           "probe", "count", "mean", "min", "max", "total", "%win");
  bench_out_println(line);

  for (uint8_t i = 0; i < BENCH_COUNT; ++i) {
    if (bench_desc[i].core != core || bench_desc[i].parent != BENCH_NONE) continue;
    bench_print_row(i, "");

    for (uint8_t j = 0; j < BENCH_COUNT; ++j) {
      if (bench_desc[j].parent != i) continue;
      bench_print_row(j, "  ");

      for (uint8_t k = 0; k < BENCH_COUNT; ++k) {
        if (bench_desc[k].parent != j) continue;
        bench_print_row(k, "    ");
      }
      bench_print_unattributed(j, "    ");
    }
    bench_print_unattributed(i, "  ");
  }
}

// Format the report into bench_out_buf (no Serial I/O).
inline void bench_print_report() {
#ifdef RUNNING_AVERAGE_FINE
  const char *fine = "on";
#else
  const char *fine = "off";
#endif

  char line[128];
  bench_out_puts("\n");
  bench_out_println("=================== DCO4 BENCH ===================");
  snprintf(line, sizeof(line), "clk_sys %lu MHz   probe overhead %lu cyc   fine probes %s",
           (unsigned long)bench_cycles_per_us, (unsigned long)bench_overhead_cyc, fine);
  bench_out_println(line);
  bench_out_println("Times in us (mean/min/max as cycles when mean < 1 us). %win = share of wall clock.");
  bench_out_puts("\n");
  bench_print_core(0);
  bench_out_puts("\n");
  bench_print_core(1);
  bench_out_println("=================================================");
  bench_out_puts("\n");
}

// Drain up to BENCH_OUT_CHUNK bytes. Returns true while more remains.
inline bool bench_out_drain_chunk() {
  if (!bench_out_active) return false;
  const uint16_t remain = (uint16_t)(bench_out_len - bench_out_pos);
  const uint16_t n = (remain > BENCH_OUT_CHUNK) ? (uint16_t)BENCH_OUT_CHUNK : remain;
  if (n > 0u) {
    Serial.write(reinterpret_cast<const uint8_t *>(bench_out_buf + bench_out_pos), n);
    bench_out_pos = (uint16_t)(bench_out_pos + n);
  }
  if (bench_out_pos >= bench_out_len) {
    bench_out_active = false;
    bench_out_reset();
    return false;
  }
  return true;
}

// Core 0 side of the handshake: format into RAM when both cores are ready, then pace
// Serial TX across loop iterations. Printing never happens on core 1.
inline void bench_poll_core0() {
  if (bench_out_drain_chunk()) {
    return;
  }

  if (bench_periodic && !bench_dump_request) {
    static uint32_t last_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_ms >= 1000u) {
      last_ms = now_ms;
      bench_dump_request = true;
    }
  }

  bench_service(0);

  if (bench_dump_request && bench_core_ready[0] && bench_core_ready[1]) {
    bench_out_reset();
    bench_print_report();
    print_clkdiv_bench();
    bench_core_ready[0] = false;
    bench_core_ready[1] = false;
    bench_dump_request = false;
    bench_out_active = (bench_out_len > 0u);
    bench_out_drain_chunk();
  }
}

#else  // !RUNNING_AVERAGE

#define BENCH_BEGIN(id)  ((void)0)
#define BENCH_END(id)    ((void)0)
#define BENCH_PERIOD(id) ((void)0)
#define BENCH_FBEGIN(id) ((void)0)
#define BENCH_FEND(id)   ((void)0)

inline void bench_service(uint8_t) {}
inline void bench_poll_core0() {}
inline void bench_reset_all() {}

#endif  // RUNNING_AVERAGE

#endif  // __BENCH_H__
