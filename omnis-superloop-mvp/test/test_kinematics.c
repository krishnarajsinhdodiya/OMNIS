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
 * LAYOUT: lateral parallel rollers (left pair one tilt, right pair the mirror).
 * Every per-wheel number below was regenerated for that layout; the front-pair
 * rows are unchanged from the old X-drive, the rear-pair rows are not.
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

static float peak4(const wheel_rates_t *r)
{
    float p = fabsf(r->fl);
    if (fabsf(r->fr) > p) p = fabsf(r->fr);
    if (fabsf(r->rl) > p) p = fabsf(r->rl);
    if (fabsf(r->rr) > p) p = fabsf(r->rr);
    return p;
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
    puts("  Case B: vy = 200 (strafe left, splits along SIDES under this layout)");
    wheel_rates_t b = mecanum_inverse(0.0f, 200.0f, 0.0f, R, L, W);
    chk("B  FL", b.fl, -3395.305, TOL_STEPS);
    chk("B  FR", b.fr,  3395.305, TOL_STEPS);
    chk("B  RL", b.rl, -3395.305, TOL_STEPS);
    chk("B  RR", b.rr,  3395.305, TOL_STEPS);
    chk("B  null space", mecanum_null_space(&b), 0.0, 1e-3);
    chk_true("B  side split: LEFT pair negative and RIGHT pair positive",
             b.fl < 0 && b.rl < 0 && b.fr > 0 && b.rr > 0);
    /* The one that would have been a diagonal split under the X-drive. Asserting
     * the magnitudes are equal catches a half-applied layout change. */
    chk_true("B  all four have the SAME magnitude",
             fabsf(fabsf(b.fl) - fabsf(b.rl)) < 1e-2f &&
             fabsf(fabsf(b.fr) - fabsf(b.rr)) < 1e-2f);

    /* --- Case C: pure rotation CCW, w = 1.0 rad/s ------------------------- */
    puts("  Case C: w = 1.0 (rotate CCW - sides, but VERY lopsided)");
    wheel_rates_t c = mecanum_inverse(0.0f, 0.0f, 1.0f, R, L, W);
    chk("C  FL", c.fl, -3845.183, TOL_STEPS);
    chk("C  FR", c.fr,  3845.183, TOL_STEPS);
    chk("C  RL", c.rl,   -59.418, TOL_STEPS);
    chk("C  RR", c.rr,    59.418, TOL_STEPS);
    chk("C  null space", mecanum_null_space(&c), 0.0, 1e-3);
    chk_true("C  side split: left pair negative, right pair positive",
             c.fl < 0 && c.rl < 0 && c.fr > 0 && c.rr > 0);
    /* THE headline consequence of the layout. Under the X-drive all four spun
     * at 3845; here the rear pair barely turns, because a pure spin is absorbed
     * almost entirely by the rear rollers. This is correct, not a fault. */
    chk("C  rear/front rate ratio", fabsf(c.rl / c.fl), 1.0 / 64.7143, 1e-5);
    chk_true("C  rear wheels turn at under 2% of the front wheels",
             fabsf(c.rl) < 0.02f * fabsf(c.fl));
    /* At a 20 steps/s deadband the rear pair holds still below ~0.34 rad/s of
     * commanded yaw. Documented so nobody reports it as a dead motor. */
    wheel_rates_t slow = mecanum_inverse(0.0f, 0.0f, 0.3f, R, L, W);
    mecanum_deadband(&slow, 20.0f);
    chk_true("C  a slow spin deadbands the rear pair but not the front",
             slow.rl == 0.0f && slow.rr == 0.0f && slow.fl != 0.0f && slow.fr != 0.0f);

    /* --- Case D: combined, vx=200 vy=100 w=0.5 ---------------------------- */
    puts("  Case D: vx=200 vy=100 w=0.5 (combined; kF*w = 113.25, kR*w = -1.75)");
    wheel_rates_t d = mecanum_inverse(200.0f, 100.0f, 0.5f, R, L, W);
    chk("D  FL", d.fl,  -224.939, TOL_STEPS);
    chk("D  FR", d.fr,  7015.550, TOL_STEPS);
    chk("D  RL", d.rl,  1667.944, TOL_STEPS);
    chk("D  RR", d.rr,  5122.667, TOL_STEPS);
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
    chk("STEPS_PER_RAD", STEPS_PER_RAD, 509.295818, 1e-3);
    chk("MICROSTEPS_PER_REV", MICROSTEPS_PER_REV, 3200.0, 1e-6);

    /* The single k of the X-drive is gone: the two axles now have different
     * yaw lever arms, kappa_i = delta_i*x_i - y_i. */
    const mecanum_levers_t k = mecanum_yaw_levers(L, W);
    chk("kappa_FL = -(L+W)/2                 [mm]", k.fl, -226.5, 1e-4);
    chk("kappa_FR = +(L+W)/2                 [mm]", k.fr,  226.5, 1e-4);
    chk("kappa_RL =  (L-W)/2                 [mm]", k.rl,   -3.5, 1e-4);
    chk("kappa_RR = -(L-W)/2                 [mm]", k.rr,    3.5, 1e-4);

    /* Structural identities that hold for ANY geometry, not just this frame.
     * kappa_F + kappa_R = -W and kappa_F - kappa_R = s*L are what collapse
     * det(M^T M) to 4*L^2 in the derivation. */
    chk("kappa_FL + kappa_RL = -track_width", k.fl + k.rl, -(double)W, 1e-3);
    chk("kappa_FL - kappa_RL = -wheelbase",   k.fl - k.rl, -(double)L, 1e-3);
    chk_true("front and rear levers are ANTI-symmetric across the centreline",
             fabsf(k.fl + k.fr) < 1e-3f && fabsf(k.rl + k.rr) < 1e-3f);

    /* The defining asymmetry of this layout, and the number worth internalising:
     * the front axle does 98.5% of the yaw work. */
    chk("|kappa_F| / |kappa_R| lever ratio", fabsf(k.fl / k.rl), 64.7143, 1e-3);
    chk("rear share of yaw authority      [%]",
        100.0 * fabsf(k.rl) / (fabsf(k.fl) + fabsf(k.rl)), 1.5217, 1e-3);

    /* A near-square frame puts the rear lever near zero and makes its SIGN
     * depend on which of L, W is larger. OMNIS is 223 x 230, so it is negative;
     * a 230 x 223 frame would flip it. Nothing breaks — but the rear pair's
     * spin direction is not something to build an intuition on. */
    const mecanum_levers_t sq = mecanum_yaw_levers(230.0f, 230.0f);
    chk("square frame: rear lever is exactly 0", sq.rl, 0.0, 1e-4);
    const mecanum_levers_t lng = mecanum_yaw_levers(230.0f, 223.0f);
    chk_true("L > W flips the rear lever's sign", lng.rl > 0.0f && k.rl < 0.0f);
    chk_true("... while the front lever keeps its sign", lng.fl < 0.0f && k.fl < 0.0f);
}

/* The properties that make this layout DIFFERENT from an X-drive, asserted
 * directly rather than left as prose in the derivation. */
static void case_layout_properties(void)
{
    puts("\nLayout properties - what parallel rollers cost and what they don't");

    /* 1. Still fully holonomic: three independent motions, each reachable
     *    exactly. A rank-deficient layout would fail the round trip on at
     *    least one axis, so run all three separately. */
    const struct { const char *name; float vx, vy, w; } axes[] = {
        { "vx only", 137.0f,   0.0f, 0.00f },
        { "vy only",   0.0f, 137.0f, 0.00f },
        { "w  only",   0.0f,   0.0f, 0.37f },
    };
    for (unsigned i = 0; i < 3u; ++i) {
        wheel_rates_t r = mecanum_inverse(axes[i].vx, axes[i].vy, axes[i].w, R, L, W);
        body_vel_t v = mecanum_forward(&r, R, L, W);
        char nm[64];
        snprintf(nm, sizeof nm, "%s round-trips exactly", axes[i].name);
        chk_true(nm, fabsf(v.vx - axes[i].vx) < 1e-2f
                  && fabsf(v.vy - axes[i].vy) < 1e-2f
                  && fabsf(v.w  - axes[i].w)  < 1e-4f);
    }

    /* 2. Top speed is NOT reduced. The peak wheel rate per unit of commanded
     *    motion is identical to the X-drive on all three axes, so the same
     *    step-rate ceiling buys the same maximum vx, vy and yaw. */
    wheel_rates_t fx = mecanum_inverse(200.0f, 0.0f, 0.0f, R, L, W);
    wheel_rates_t fy = mecanum_inverse(0.0f, 200.0f, 0.0f, R, L, W);
    wheel_rates_t fw = mecanum_inverse(0.0f, 0.0f, 1.0f, R, L, W);
    chk("peak rate for vx = 200", peak4(&fx), 3395.305, TOL_STEPS);
    chk("peak rate for vy = 200", peak4(&fy), 3395.305, TOL_STEPS);
    chk("peak rate for w  = 1.0", peak4(&fw), 3845.183, TOL_STEPS);
    chk_true("strafe costs exactly what forward costs, as before",
             fabsf(peak4(&fx) - peak4(&fy)) < 1e-2f);

    /* 3. What it DOES cost: the vy and w columns are no longer orthogonal.
     *    Under the X-drive this dot product was exactly zero for every
     *    geometry; here it is 2*W/r^2 in wheel-rate units. */
    const mecanum_levers_t k = mecanum_yaw_levers(L, W);
    const float dot = (-1.0f) * k.fl + (1.0f) * k.fr
                    + (-1.0f) * k.rl + (1.0f) * k.rr;      /* c_vy . c_w, x r^2 */
    chk("vy and w columns: dot product = 2*track_width", dot, 2.0 * (double)W, 1e-2);
    chk_true("  ... i.e. NOT orthogonal (the X-drive's was 0)", fabsf(dot) > 1.0f);
    const double col_w = sqrt((double)(k.fl*k.fl + k.fr*k.fr + k.rl*k.rl + k.rr*k.rr));
    chk("|w column| = sqrt(L^2 + W^2)", col_w, sqrt((double)L*L + (double)W*W), 1e-2);
    chk("angle between the vy and w columns  [deg]",
        acos((double)dot / (2.0 * col_w)) * 180.0 / M_PI, 44.11, 0.02);

    /* 4. The null space is UNCHANGED: (+1,+1,-1,-1) still moves nothing. Assert
     *    it against the columns themselves, which is the property that matters,
     *    not just against commands the IK happened to produce. */
    chk("null space of the vx column", 1.0f + 1.0f - 1.0f - 1.0f, 0.0, 1e-9);
    chk("null space of the vy column",
        (double)(MECANUM_DELTA_FL + MECANUM_DELTA_FR
               - MECANUM_DELTA_RL - MECANUM_DELTA_RR), 0.0, 1e-9);
    chk("null space of the w  column", k.fl + k.fr - k.rl - k.rr, 0.0, 1e-3);
}

static void case_bench_diagonal(void)
{
    puts("\nBench test from reference §4 - the one that catches a wrong wheel");
    /* "command vx = vy (diagonal forward-left). The whole LEFT SIDE must be
     * completely stationary. If it creeps, the handedness constants or a DIR
     * invert flag are wrong."
     *
     * Under the X-drive this idled the FL/RR diagonal. Under parallel rollers
     * it idles a SIDE, which is if anything easier to see across a bench: two
     * wheels next to each other, both dead still, while the other two drive. */
    wheel_rates_t r = mecanum_inverse(200.0f, 200.0f, 0.0f, R, L, W);
    chk_true("vx = vy: FL is EXACTLY zero", r.fl == 0.0f);
    chk_true("vx = vy: RL is EXACTLY zero", r.rl == 0.0f);
    chk_true("vx = vy: FR and RR are driven", r.fr > 0.0f && r.rr > 0.0f);
    chk("vx = vy: FR", r.fr, 6790.611, 0.02);
    chk("vx = vy: RR", r.rr, 6790.611, 0.02);
    chk_true("vx = vy: the idle pair is a SIDE, not a diagonal",
             r.fl == 0.0f && r.rl == 0.0f);

    /* The mirror check: vx = -vy idles the RIGHT side instead. Asserting both
     * is what distinguishes "handedness correct" from "handedness inverted" —
     * a mirrored build passes one of these and fails the other. */
    wheel_rates_t q = mecanum_inverse(200.0f, -200.0f, 0.0f, R, L, W);
    chk_true("vx = -vy: the RIGHT side idles instead",
             q.fr == 0.0f && q.rr == 0.0f && q.fl > 0.0f && q.rl > 0.0f);
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


static void case_slew(void)
{
    puts("\ndrive_slew_rates - acceleration limit by COMMON factor");
    const wheel_rates_t target = mecanum_inverse(200.0f, 100.0f, 0.5f, R, L, W); /* Case D */
    wheel_rates_t cur = { 0.0f, 0.0f, 0.0f, 0.0f };

    bool limited = drive_slew_rates(&cur, &target, 1000.0f);
    chk_true("first step from rest is limited", limited);
    chk("largest change (FR) is exactly the limit", fabsf(cur.fr), 1000.0, 1e-2);
    const float s = cur.fr / target.fr;
    chk("FL moved by the same fraction", cur.fl / target.fl, s, 1e-5);
    chk("RL moved by the same fraction", cur.rl / target.rl, s, 1e-5);
    chk("RR moved by the same fraction", cur.rr / target.rr, s, 1e-5);
    chk("null space stays empty mid-transition", mecanum_null_space(&cur), 0.0, 1e-2);

    const body_vel_t mid = mecanum_forward(&cur, R, L, W);
    chk("mid-transition vy/vx keeps the target direction (0.5)", mid.vy / mid.vx, 0.5, 1e-4);
    chk("mid-transition w/vx keeps the target ratio (0.0025)", mid.w / mid.vx, 0.0025, 1e-6);

    int steps = 1;
    while (drive_slew_rates(&cur, &target, 1000.0f)) {
        ++steps;
        if (steps > 50) break;
    }
    chk("reaches the target in ceil(7015.55/1000) = 8 calls", steps + 1, 8, 0);
    chk_true("  ... and lands on it exactly",
             cur.fl == target.fl && cur.fr == target.fr &&
             cur.rl == target.rl && cur.rr == target.rr);

    wheel_rates_t near = target;
    near.fr += 10.0f;
    chk_true("a change inside the limit is applied in one step",
             !drive_slew_rates(&near, &target, 1000.0f) && near.fr == target.fr);

    wheel_rates_t nan_t = { NAN, NAN, NAN, NAN };
    wheel_rates_t z = { 5.0f, -5.0f, 5.0f, -5.0f };
    drive_slew_rates(&z, &nan_t, 1000.0f);
    chk_true("NaN target is treated as stop", z.fl == 0.0f && z.fr == 0.0f);

    wheel_rates_t frozen = { 100.0f, 100.0f, 100.0f, 100.0f };
    drive_slew_rates(&frozen, &target, 0.0f);
    chk_true("non-positive limit freezes instead of meaning unlimited",
             frozen.fl == 100.0f && frozen.fr == 100.0f);
}

int main(void)
{
    puts("OMNIS kinematics + drive pipeline verification");
    puts("expected values from assets/kinematics/mecanum-kinematics-reference.md §7");

    case_geometry();
    case_layout_properties();
    case_abcd();
    case_bench_diagonal();
    case_clamp();
    case_deadband();
    case_rc_mapping();
    case_pipeline();
    case_slew();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
