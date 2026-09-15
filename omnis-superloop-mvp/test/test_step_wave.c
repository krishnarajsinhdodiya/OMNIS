/**
 * @file    test_step_wave.c
 * @brief   Host-side proof of the STEP waveform guarantees in step_wave.h.
 *
 * Every chunk the generator produces is expanded back into a continuous
 * (level, duration) timeline — exactly what the STEP pin would show — and the
 * guarantees are checked on that timeline, across chunk boundaries, over many
 * seconds of simulated output. These are the properties that would otherwise be
 * found with a logic analyser on the bench: a lost or doubled step is invisible
 * to the robot's open-loop estimator.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "step_wave.h"

static int g_fail = 0, g_pass = 0;

static void chk(const char *name, double got, double want, double tol)
{
    if (fabs(got - want) <= tol) { ++g_pass; printf("  ok    %-52s %12.3f\n", name, got); }
    else { ++g_fail; printf("  FAIL  %-52s %12.3f (want %.3f +- %.3f)\n", name, got, want, tol); }
}

static void chk_true(const char *name, int cond)
{
    if (cond) { ++g_pass; printf("  ok    %-52s\n", name); }
    else      { ++g_fail; printf("  FAIL  %-52s\n", name); }
}

/* ---- Timeline reconstruction ------------------------------------------- */
typedef struct {
    int      started;
    int      cur_lvl;
    uint64_t cur_dur;
    int      seen_high;      /* a low run only counts once a pulse has occurred */
    uint64_t total_us;
    uint64_t rises;
    uint64_t min_high, max_high, min_low;
    uint64_t zero_halves;
    uint64_t first_rise_us;  /* absolute time of the first rising edge */
    int      have_first_rise;
} timeline_t;

static void tl_init(timeline_t *t)
{
    memset(t, 0, sizeof *t);
    t->min_high = UINT64_MAX;
    t->min_low  = UINT64_MAX;
}

static void tl_close_run(timeline_t *t)
{
    if (!t->started) return;
    if (t->cur_lvl) {
        if (t->cur_dur < t->min_high) t->min_high = t->cur_dur;
        if (t->cur_dur > t->max_high) t->max_high = t->cur_dur;
        t->seen_high = 1;
    } else if (t->seen_high) {
        if (t->cur_dur < t->min_low) t->min_low = t->cur_dur;
    }
}

static void tl_add(timeline_t *t, int lvl, uint32_t dur)
{
    if (dur == 0u) { ++t->zero_halves; return; }
    if (t->started && lvl == t->cur_lvl) {
        t->cur_dur += dur;
    } else {
        /* a low run that ends in a high counts only if bounded by pulses on
         * both sides, i.e. it started after a pulse */
        if (t->started && !t->cur_lvl && lvl) {
            tl_close_run(t);
        } else if (t->started) {
            tl_close_run(t);
        }
        if (lvl) {
            ++t->rises;
            if (!t->have_first_rise) { t->have_first_rise = 1; t->first_rise_us = t->total_us; }
        }
        t->started = 1;
        t->cur_lvl = lvl;
        t->cur_dur = dur;
    }
    t->total_us += dur;
}

/* Feed one chunk and check the per-chunk guarantees. Returns 0 if any failed. */
typedef struct {
    uint64_t chunks, bad_sum, bad_end, bad_len, over_cap;
} chunk_checks_t;

static void feed_chunk(timeline_t *t, chunk_checks_t *cc, const step_symbol_t *s,
                       size_t n, uint32_t dur, size_t cap,
                       const step_wave_cfg_t *cfg, int expect_full_len)
{
    uint64_t sum = 0;
    for (size_t i = 0; i < n; ++i) {
        tl_add(t, s[i].lvl0, s[i].dur0);
        tl_add(t, s[i].lvl1, s[i].dur1);
        sum += (uint64_t)s[i].dur0 + s[i].dur1;
    }
    ++cc->chunks;
    if (sum != dur) ++cc->bad_sum;
    if (n > 0 && s[n - 1].lvl1 != 0) ++cc->bad_end;
    if (n > cap) ++cc->over_cap;
    /* Never longer than nominal + pulse + minimum low + pad, at any rate. Never
     * SHORTER only when the rate is within the full-chunk limit (guarantee 5). */
    const uint32_t hi = cfg->chunk_us + cfg->pulse_us
                      + (cfg->min_low_us < 2 ? 2 : cfg->min_low_us) + 1u;
    if (dur > hi || dur == 0u) ++cc->bad_len;
    if (expect_full_len && dur < cfg->chunk_us) ++cc->bad_len;
}

/* Run `seconds` of output at a constant rate. */
static void run_constant(const step_wave_cfg_t *cfg, float rate, double seconds,
                         timeline_t *t, chunk_checks_t *cc)
{
    step_wave_t w;
    step_wave_init(&w);
    tl_init(t);
    memset(cc, 0, sizeof *cc);
    step_symbol_t buf[64];
    const int full = rate <= step_wave_max_full_chunk_rate(cfg, 64);
    while ((double)t->total_us < seconds * 1e6) {
        uint32_t dur = 0, np = 0;
        size_t n = step_wave_fill(&w, cfg, rate, buf, 64, &dur, &np);
        feed_chunk(t, cc, buf, n, dur, 64, cfg, full);
    }
}

static const step_wave_cfg_t CFG = { .chunk_us = 2000u, .pulse_us = 10u, .min_low_us = 10u };

static void case_constant_rates(void)
{
    puts("\nConstant rates, 2 s each: pulse shape, gaps, chunk framing, count");
    const float rates[] = { 25.0f, 100.0f, 999.9f, 3395.305f, 7000.0f, 20000.0f, 30000.0f, 50000.0f };
    for (size_t i = 0; i < sizeof rates / sizeof rates[0]; ++i) {
        timeline_t t; chunk_checks_t cc;
        run_constant(&CFG, rates[i], 2.0, &t, &cc);
        char name[96];
        const double expected = (double)rates[i] * (double)t.total_us / 1e6;

        snprintf(name, sizeof name, "%.1f st/s: pulses vs rate x time", (double)rates[i]);
        chk(name, (double)t.rises, expected, 1.5);
        snprintf(name, sizeof name, "%.1f st/s: every pulse exactly 10 us", (double)rates[i]);
        chk_true(name, t.min_high == 10 && t.max_high == 10);
        if (t.rises > 1) {
            snprintf(name, sizeof name, "%.1f st/s: every gap >= 10 us (incl. across chunks)", (double)rates[i]);
            chk_true(name, t.min_low >= 10);
        }
        snprintf(name, sizeof name, "%.1f st/s: no zero-duration halves", (double)rates[i]);
        chk_true(name, t.zero_halves == 0);
        snprintf(name, sizeof name, "%.1f st/s: every chunk ends LOW", (double)rates[i]);
        chk_true(name, cc.bad_end == 0);
        snprintf(name, sizeof name, "%.1f st/s: durations sum to reported length", (double)rates[i]);
        chk_true(name, cc.bad_sum == 0);
        if (rates[i] <= step_wave_max_full_chunk_rate(&CFG, 64)) {
            snprintf(name, sizeof name, "%.1f st/s: chunk length within [2000, 2021] us", (double)rates[i]);
        } else {
            snprintf(name, sizeof name, "%.1f st/s: over buffer limit - chunks end early, never overrun", (double)rates[i]);
        }
        chk_true(name, cc.bad_len == 0);
    }
}

static void case_average_rate_exact(void)
{
    puts("\nRounding residue is carried: no accumulated rate error");
    /* 3395.305 st/s is kinematics Case A. Its period is 294.527 us; rounding each
     * edge independently to 295 us would lose ~55 steps over 10 s. */
    timeline_t t; chunk_checks_t cc;
    run_constant(&CFG, 3395.305f, 10.0, &t, &cc);
    const double expected = 3395.305 * (double)t.total_us / 1e6;
    chk("10 s at 3395.305 st/s: pulses within 1.5 of exact", (double)t.rises, expected, 1.5);
    chk("  naive per-edge rounding would give (for contrast)",
        floor((double)t.total_us / 295.0) + 1.0, expected, 60.0);
}

static void case_idle_and_restart(void)
{
    puts("\nIdle, restart, and invalid rates");
    step_wave_t w; step_wave_init(&w);
    step_symbol_t buf[64];
    uint32_t dur = 0, np = 0;

    size_t n = step_wave_fill(&w, &CFG, 0.0f, buf, 64, &dur, &np);
    chk("rate 0: chunk length", dur, 2000, 0);
    chk("rate 0: no pulses", np, 0, 0);
    int all_low = 1;
    for (size_t i = 0; i < n; ++i) all_low &= (buf[i].lvl0 == 0 && buf[i].lvl1 == 0);
    chk_true("rate 0: entirely low", all_low);
    chk("rate 0: generator armed (phase = 1)", w.phase, 1.0, 1e-6);

    n = step_wave_fill(&w, &CFG, 100.0f, buf, 64, &dur, &np);
    chk_true("restart at 100 st/s: first pulse at t = 0 (no stall)",
             n > 0 && buf[0].lvl0 == 1 && buf[0].dur0 == 10);

    const float bad[] = { NAN, -500.0f, INFINITY };
    const char *names[] = { "NaN rate -> idle", "negative rate -> idle", "inf rate -> idle" };
    for (int i = 0; i < 3; ++i) {
        step_wave_init(&w);
        step_wave_fill(&w, &CFG, bad[i], buf, 64, &dur, &np);
        chk_true(names[i], np == 0 && dur == 2000);
    }

    chk_true("invalid args: NULL state -> 0 symbols",
             step_wave_fill(NULL, &CFG, 100.0f, buf, 64, &dur, &np) == 0);
    chk_true("invalid args: capacity < 2 -> 0 symbols",
             step_wave_fill(&w, &CFG, 100.0f, buf, 1, &dur, &np) == 0);
}

static void case_rate_step_no_stall(void)
{
    puts("\nSpeeding up from a crawl does not wait out the old period");
    step_wave_t w; step_wave_init(&w);
    step_symbol_t buf[64];
    uint32_t dur = 0, np = 0;

    /* One chunk at 20 st/s (50 ms period): pulse at t=0, then phase ~0.04. */
    step_wave_fill(&w, &CFG, 20.0f, buf, 64, &dur, &np);
    chk("crawl chunk: one pulse", np, 1, 0);

    size_t n = step_wave_fill(&w, &CFG, 7000.0f, buf, 64, &dur, &np);
    uint32_t t = 0, first_rise = UINT32_MAX;
    for (size_t i = 0; i < n && first_rise == UINT32_MAX; ++i) {
        if (buf[i].lvl0) { first_rise = t; break; }
        t += buf[i].dur0;
        if (buf[i].lvl1) { first_rise = t; break; }
        t += buf[i].dur1;
    }
    chk_true("first 7000 st/s pulse within one new period (143 us), not ~48 ms",
             first_rise <= 143u);
    printf("        (first pulse at %u us)\n", (unsigned)first_rise);
}

static uint32_t lcg(uint32_t *s) { *s = *s * 1664525u + 1013904223u; return *s; }

static void case_random_rate_changes(void)
{
    puts("\nRandom rate changes every chunk for 20 s: guarantees must survive");
    step_wave_t w; step_wave_init(&w);
    step_symbol_t buf[64];
    timeline_t t; tl_init(&t);
    chunk_checks_t cc; memset(&cc, 0, sizeof cc);
    uint32_t seed = 12345u;
    const float rmax = step_wave_max_rate(&CFG);
    while (t.total_us < 20000000ull) {
        uint32_t r = lcg(&seed);
        float rate;
        switch (r % 5u) {
            case 0:  rate = 0.0f; break;                                   /* stops      */
            case 1:  rate = rmax; break;                                   /* flat out   */
            case 2:  rate = 20.0f + (float)(r % 200u); break;              /* crawl      */
            default: rate = (float)((r >> 8) % (uint32_t)rmax); break;     /* anything   */
        }
        uint32_t dur = 0, np = 0;
        size_t n = step_wave_fill(&w, &CFG, rate, buf, 64, &dur, &np);
        feed_chunk(&t, &cc, buf, n, dur, 64, &CFG,
                   rate <= step_wave_max_full_chunk_rate(&CFG, 64));
    }
    printf("        (%llu chunks, %llu pulses)\n",
           (unsigned long long)cc.chunks, (unsigned long long)t.rises);
    chk_true("every pulse exactly 10 us", t.min_high == 10 && t.max_high == 10);
    chk_true("every gap >= 10 us, across rate changes and chunk edges", t.min_low >= 10);
    chk_true("no zero-duration halves", t.zero_halves == 0);
    chk_true("every chunk ends LOW (no merged or split pulses)", cc.bad_end == 0);
    chk_true("chunk length full where the buffer allows, never overrun", cc.bad_len == 0);
    chk_true("durations always sum to the reported length", cc.bad_sum == 0);
}

static void case_small_buffer_and_caps(void)
{
    puts("\nSmall buffers, rate cap, and minimum-low clamping");
    chk("full-chunk rate, 64 symbols x 2 ms chunks", step_wave_max_full_chunk_rate(&CFG, 64),
        30000.0, 1e-2);
    chk("full-chunk rate, capacity 4 guarantees nothing", step_wave_max_full_chunk_rate(&CFG, 4),
        0.0, 0.0);
    chk_true("firmware max_step_rate 7000 is far inside the full-chunk limit",
             7000.0f <= step_wave_max_full_chunk_rate(&CFG, 64));
    step_wave_t w; step_wave_init(&w);
    step_symbol_t buf[64];
    timeline_t t; tl_init(&t);
    chunk_checks_t cc; memset(&cc, 0, sizeof cc);
    for (int i = 0; i < 200; ++i) {
        uint32_t dur = 0, np = 0;
        size_t n = step_wave_fill(&w, &CFG, 50000.0f, buf, 4, &dur, &np);
        feed_chunk(&t, &cc, buf, n, dur, 4, &CFG, 0);
    }
    chk_true("capacity 4: never writes more than 4 symbols", cc.over_cap == 0);
    chk_true("capacity 4: still ends every chunk LOW", cc.bad_end == 0);
    chk_true("capacity 4: pulses still exactly 10 us", t.min_high == 10 && t.max_high == 10);
    chk_true("capacity 4: gaps still >= 10 us", t.min_low >= 10);

    step_wave_init(&w);
    uint32_t dur = 0, np = 0;
    step_wave_fill(&w, &CFG, 1.0e9f, buf, 64, &dur, &np);
    chk("1e9 st/s is capped at max (1e6/20 x 2000 us + 1)",
        np, 50000.0 * dur / 1e6, 1.5);

    const step_wave_cfg_t tight = { .chunk_us = 2000u, .pulse_us = 10u, .min_low_us = 0u };
    chk("min_low 0 is clamped to 2 us (max rate 1e6/12)", step_wave_max_rate(&tight),
        1.0e6 / 12.0, 1e-2);
    timeline_t t2; chunk_checks_t cc2;
    run_constant(&tight, 1.0e9f, 1.0, &t2, &cc2);
    chk_true("min_low 0 config: gaps never below 2 us", t2.min_low >= 2);
    chk_true("min_low 0 config: no zero-duration halves", t2.zero_halves == 0);
}

int main(void)
{
    puts("OMNIS STEP waveform generator verification");
    case_constant_rates();
    case_average_rate_exact();
    case_idle_and_restart();
    case_rate_step_no_stall();
    case_random_rate_changes();
    case_small_buffer_and_caps();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
