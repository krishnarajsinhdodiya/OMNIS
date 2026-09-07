/**
 * @file    test_kinematics.c
 * @brief   Host-side verification of the kinematics port and the drive pipeline.
 *
 * Runs on the Mac with plain cc — no ESP-IDF, no hardware. The expected values
 * in Cases A-D are the verified numbers from
 * assets/kinematics/mecanum-kinematics-reference.md §7, which is the authority:
 * if a refactor breaks one of these, the reference is right and the code is
 * wrong.
 *
 *   ./run_host_tests.sh
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "mecanum_kinematics.h"
#include "drive.h"
#include "omnis_params.h"

static int g_fail = 0, g_pass = 0;

static void chk(const char *name, double got, double want, double tol)
{
    const double err = fabs(got - want);
    if (err <= tol) {
        ++g_pass;
        printf("  ok    %-44s %12.4f\n", name, got);
    } else {
        ++g_fail;
        printf("  FAIL  %-44s %12.4f (want %.4f, err %.3g)\n",
               name, got, want, err);
    }
}

static void chk_true(const char *name, int cond)
{
    if (cond) { ++g_pass; printf("  ok    %-44s\n", name); }
    else      { ++g_fail; printf("  FAIL  %-44s\n", name); }
}

/* OMNIS as-built geometry. */
static const omnis_geometry_t GEOM = { 30.0f, 223.0f, 230.0f };

#define R  (GEOM.wheel_radius_mm)
#define L  (GEOM.wheelbase_mm)
#define W  (GEOM.track_width_mm)

/* Step rates near 7000 hold ~0.001 of float32 resolution; 0.01 is comfortably
 * inside that and far tighter than the reference quotes. */
#define TOL_STEPS  0.01
#define TOL_VEL    0.01

static void case_abcd(void)
{
    puts("\nCases A-D - verified numbers from kinematics reference §7");

    /* --- Case A: pure forward, vx = 200 mm/s ------------------------------ */
    puts("  Case A: vx = 200 (pure forward, all four identical)");
    wheel_rates_t a = mecanum_inverse(200.0f, 0.0f, 0.0f, R, L, W);
    chk("A  FL", a.fl,  3395.305, TOL_STEPS);
    chk("A  FR", a.fr,  3395.305, TOL_STEPS);
    chk("A  RL", a.rl,  3395.305, TOL_STEPS);
    chk("A  RR", a.rr,  3395.305, TOL_STEPS);
    chk("A  null space", mecanum_null_space(&a), 0.0, 1e-3);

    /* --- Case B: pure strafe left, vy = 200 mm/s -------------------------- */
    puts("  Case B: vy = 200 (strafe left, splits along DIAGONALS)");
    wheel_rates_t b = mecanum_inverse(0.0f, 200.0f, 0.0f, R, L, W);
    chk("B  FL", b.fl, -3395.305, TOL_STEPS);
    chk("B  FR", b.fr,  3395.305, TOL_STEPS);
    chk("B  RL", b.rl,  3395.305, TOL_STEPS);
    chk("B  RR", b.rr, -3395.305, TOL_STEPS);
    chk("B  null space", mecanum_null_space(&b), 0.0, 1e-3);
    chk_true("B  diagonal split: FL,RR negative and FR,RL positive",
             b.fl < 0 && b.rr < 0 && b.fr > 0 && b.rl > 0);

    /* --- Case C: pure rotation CCW, w = 1.0 rad/s ------------------------- */
    puts("  Case C: w = 1.0 (rotate CCW, splits along SIDES)");
    wheel_rates_t c = mecanum_inverse(0.0f, 0.0f, 1.0f, R, L, W);
    chk("C  FL", c.fl, -3845.183, TOL_STEPS);
    chk("C  FR", c.fr,  3845.183, TOL_STEPS);
    chk("C  RL", c.rl, -3845.183, TOL_STEPS);
    chk("C  RR", c.rr,  3845.183, TOL_STEPS);
    chk("C  null space", mecanum_null_space(&c), 0.0, 1e-3);
    chk_true("C  side split: left pair negative, right pair positive",
             c.fl < 0 && c.rl < 0 && c.fr > 0 && c.rr > 0);

    /* --- Case D: combined, vx=200 vy=100 w=0.5 ---------------------------- */
    puts("  Case D: vx=200 vy=100 w=0.5 (combined; k*w = 113.25 mm/s)");
    wheel_rates_t d = mecanum_inverse(200.0f, 100.0f, 0.5f, R, L, W);
    chk("D  FL", d.fl,  -224.939, TOL_STEPS);
    chk("D  FR", d.fr,  7015.550, TOL_STEPS);
    chk("D  RL", d.rl,  3170.366, TOL_STEPS);
    chk("D  RR", d.rr,  3620.244, TOL_STEPS);
    chk("D  null space", mecanum_null_space(&d), 0.0, 1e-3);

    /* IK -> FK round trip. Reference: "Any sign error shows up immediately as a
     * failed round-trip." */
    body_vel_t fk = mecanum_forward(&d, R, L, W);
    chk("D  round-trip vx", fk.vx, 200.0, TOL_VEL);
    chk("D  round-trip vy", fk.vy, 100.0, TOL_VEL);
    chk("D  round-trip w",  fk.w,    0.5, 1e-4);

    /* Reference's practical-limits note: FR is near this build's ceiling. */
    chk("D  FR in rev/s", d.fr / 3200.0, 2.192, 1e-3);
    chk("D  FR in RPM",   d.fr / 3200.0 * 60.0, 131.5, 0.1);
}

static void case_geometry(void)
{
    puts("\nDerived geometry");
    const float k = 0.5f * (L + W);
    chk("k = (L+W)/2                        [mm]", k, 226.5, 1e-4);
    chk("k / wheel_radius", k / R, 7.55, 1e-4);
    chk("STEPS_PER_RAD", STEPS_PER_RAD, 509.295818, 1e-3);
    chk("MICROSTEPS_PER_REV", MICROSTEPS_PER_REV, 3200.0, 1e-6);
}

static void case_bench_diagonal(void)
{
    puts("\nBench test from reference §4 - the one that catches a wrong wheel");
    /* "command vx = vy (diagonal forward-left). FL and RR must be completely
     * stationary. If they creep, a handedness assignment or a DIR invert flag
     * is wrong." */
    wheel_rates_t r = mecanum_inverse(200.0f, 200.0f, 0.0f, R, L, W);
    chk_true("vx = vy: FL is EXACTLY zero", r.fl == 0.0f);
    chk_true("vx = vy: RR is EXACTLY zero", r.rr == 0.0f);
    chk_true("vx = vy: FR and RL are driven", r.fr > 0.0f && r.rl > 0.0f);
    chk("vx = vy: FR", r.fr, 6790.611, 0.02);
    chk("vx = vy: RL", r.rl, 6790.611, 0.02);
}

static void case_clamp(void)
{
    puts("\nClamp - COMMON factor must preserve direction, not clip per wheel");

    /* Case D saturates a 3000-step limit (its FR is 7015). */
    wheel_rates_t r = mecanum_inverse(200.0f, 100.0f, 0.5f, R, L, W);
    const wheel_rates_t before = r;

    mecanum_clamp(&r, 3000.0f);

    chk("peak is exactly at the limit", fabsf(r.fr), 3000.0, 1e-2);

    /* Every wheel scaled by the SAME factor: ratios survive. */
    const float s = r.fr / before.fr;
    chk("FL scaled by the same factor", r.fl / before.fl, s, 1e-4);
    chk("RL scaled by the same factor", r.rl / before.rl, s, 1e-4);
    chk("RR scaled by the same factor", r.rr / before.rr, s, 1e-4);

    /* Therefore the FK direction is unchanged, only the magnitude. */
    body_vel_t fk = mecanum_forward(&r, R, L, W);
    chk("clamped vx is scaled, not clipped", fk.vx, 200.0 * (double)s, 1e-2);
    chk("clamped vy is scaled, not clipped", fk.vy, 100.0 * (double)s, 1e-2);
    chk("clamped w  is scaled, not clipped", fk.w,    0.5 * (double)s, 1e-4);
    chk("null space survives the clamp", mecanum_null_space(&r), 0.0, 1e-2);

    /* Below the limit, nothing is touched at all. */
    wheel_rates_t q = mecanum_inverse(10.0f, 0.0f, 0.0f, R, L, W);
    const wheel_rates_t q0 = q;
    mecanum_clamp(&q, 7000.0f);
    chk_true("under the limit: untouched", q.fl == q0.fl && q.fr == q0.fr);
}

static void case_deadband(void)
{
    puts("\nDeadband - hold near-zero wheels stopped (A4988 DIR chatter)");

    /* Case D's FL is -224.9 steps/s: above a 20 step/s deadband, so it must
     * survive. This is the reference's own example of a wheel sitting close to
     * a direction reversal. */
    wheel_rates_t d = mecanum_inverse(200.0f, 100.0f, 0.5f, R, L, W);
    mecanum_deadband(&d, 20.0f);
    chk_true("Case D FL (-224.9) survives a 20 step/s deadband", d.fl != 0.0f);

    /* A genuinely tiny command is zeroed. */
    wheel_rates_t t = mecanum_inverse(0.5f, 0.0f, 0.0f, R, L, W);  /* ~8.5 st/s */
    chk_true("8.5 steps/s is above zero before deadband", fabsf(t.fl) > 1.0f);
    mecanum_deadband(&t, 20.0f);
    chk_true("... and zeroed by a 20 step/s deadband",
             t.fl == 0.0f && t.fr == 0.0f && t.rl == 0.0f && t.rr == 0.0f);
}

static void case_rc_mapping(void)
{
    puts("\nRC mapping - reference §5");

    omnis_params_t p;
    omnis_params_defaults(&p);

    rc_sticks_t s = { 0 };
    body_vel_t v;

    /* Centred sticks. */
    v = drive_rc_to_body(&s, &p.rc_scale);
    chk("centred: vx", v.vx, 0.0, 1e-6);
    chk("centred: vy", v.vy, 0.0, 1e-6);
    chk("centred: w",  v.w,  0.0, 1e-6);

    /* Full throttle only. */
    s = (rc_sticks_t){ .throttle = 1.0f };
    v = drive_rc_to_body(&s, &p.rc_scale);
    chk("full throttle -> vx", v.vx, 300.0, 1e-3);

    /* Full roll left -> vy positive (LEFT is positive in this frame). */
    s = (rc_sticks_t){ .roll = 1.0f };
    v = drive_rc_to_body(&s, &p.rc_scale);
    chk("full roll left -> vy", v.vy, 300.0, 1e-3);

    /* Full yaw left -> w positive (CCW). */
    s = (rc_sticks_t){ .yaw = 1.0f };
    v = drive_rc_to_body(&s, &p.rc_scale);
    chk("full yaw left -> w (CCW)", v.w, 1.5, 1e-4);

    /* Throttle and pitch are ADDITIVE into vx, and deliberately not clamped
     * here — the wheel clamp handles saturation. See drive.h. */
    s = (rc_sticks_t){ .throttle = 1.0f, .pitch = 1.0f };
    v = drive_rc_to_body(&s, &p.rc_scale);
    chk("throttle + pitch are additive", v.vx, 450.0, 1e-3);
    chk_true("  ... and exceed VX_MAX on purpose", v.vx > p.rc_scale.vx_max_mmps);

    /* Out-of-range sticks are clamped to the unit interval. */
    s = (rc_sticks_t){ .throttle = 5.0f };
    v = drive_rc_to_body(&s, &p.rc_scale);
    chk("stick of 5.0 clamps to 1.0", v.vx, 300.0, 1e-3);

    /* A NaN stick must read as centred, not propagate to a step rate. */
    s = (rc_sticks_t){ .throttle = NAN, .roll = NAN };
    v = drive_rc_to_body(&s, &p.rc_scale);
    chk_true("NaN stick reads as centred", v.vx == 0.0f && v.vy == 0.0f);

    /* Polarity is corrected HERE and nowhere else. */
    p.rc_scale.invert_yaw = true;
    s = (rc_sticks_t){ .yaw = 1.0f };
    v = drive_rc_to_body(&s, &p.rc_scale);
    chk("invert_yaw flips the sign", v.w, -1.5, 1e-4);
}

static void case_pipeline(void)
{
    puts("\ndrive_solve - full pipeline and its health metrics");

    omnis_params_t p;
    omnis_params_defaults(&p);
    drive_solution_t sol;

    /* Case D again, but through the pipeline with the real 7000 step limit.
     * Its peak (7015) is barely over, so it clamps by a hair. */
    body_vel_t v = { 200.0f, 100.0f, 0.5f };
    drive_solve(&v, &p.geometry, &p.step, &sol);

    chk("peak rate before clamping", sol.peak_rate, 7015.550, TOL_STEPS);
    chk_true("marked as clamped (7015 > 7000)", sol.clamped);
    chk("null space is clean", sol.null_space, 0.0, 1e-2);
    chk_true("no wheel deadbanded", sol.deadbanded == 0);
    chk("post-clamp peak sits at the limit", fabsf(sol.rates.fr), 7000.0, 1e-2);

    /* An unsaturated command passes through untouched and round-trips. */
    body_vel_t small = { 100.0f, 50.0f, 0.25f };
    drive_solve(&small, &p.geometry, &p.step, &sol);
    chk_true("small command not clamped", !sol.clamped);
    chk("round-trip vx", sol.achieved.vx, 100.0, TOL_VEL);
    chk("round-trip vy", sol.achieved.vy,  50.0, TOL_VEL);
    chk("round-trip w",  sol.achieved.w,   0.25, 1e-4);
    chk("null space is clean", sol.null_space, 0.0, 1e-2);

    /* A crawl gets fully deadbanded: four wheels stopped, zero velocity out. */
    body_vel_t crawl = { 0.5f, 0.0f, 0.0f };
    drive_solve(&crawl, &p.geometry, &p.step, &sol);
    chk_true("crawl deadbands all four wheels", sol.deadbanded == 4);
    chk("... so achieved vx is zero", sol.achieved.vx, 0.0, 1e-6);
    chk_true("... while commanded vx was not", sol.commanded.vx == 0.5f);

    /* Sticks straight through. */
    rc_sticks_t s = { .throttle = 1.0f };
    drive_from_sticks(&s, &p, &sol);
    chk("full throttle -> all four wheels equal", sol.rates.fl, 5092.958, 0.02);
    chk_true("  ... and identical",
             sol.rates.fl == sol.rates.fr &&
             sol.rates.fl == sol.rates.rl &&
             sol.rates.fl == sol.rates.rr);
    chk("  ... round-tripping to 300 mm/s", sol.achieved.vx, 300.0, TOL_VEL);

    /* Null pointers must not crash, and must fail stopped. */
    drive_solve(NULL, &p.geometry, &p.step, &sol);
    chk_true("NULL velocity -> all wheels stopped",
             sol.rates.fl == 0.0f && sol.rates.fr == 0.0f &&
             sol.rates.rl == 0.0f && sol.rates.rr == 0.0f);
}

int main(void)
{
    puts("OMNIS kinematics + drive pipeline verification");
    puts("expected values from assets/kinematics/mecanum-kinematics-reference.md §7");

    case_geometry();
    case_abcd();
    case_bench_diagonal();
    case_clamp();
    case_deadband();
    case_rc_mapping();
    case_pipeline();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
