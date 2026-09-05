/**
 * @file    test_control.c
 * @brief   Host-side verification of the EKF, fusion and PID modules.
 *
 * Builds and runs on the Mac with plain gcc — no ESP-IDF, no hardware. Every
 * expected value here was computed independently in Python and is recorded in
 * attitude-ekf-reference.md §6 / sensor-fusion-reference.md §5 / pid-reference.md §6.
 *
 *   cc -std=c99 -Wall -Wextra -O2 -o test_control \
 *      test_control.c attitude_ekf.c imu_fusion.c pid.c -lm && ./test_control
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "attitude_ekf.h"
#include "imu_fusion.h"
#include "pid.h"

#define DEG(r) ((r) * 180.0f / 3.14159265358979323846f)
#define RAD(d) ((d) * 3.14159265358979323846f / 180.0f)

static int g_fail = 0, g_pass = 0;

static void chk(const char *name, double got, double want, double tol)
{
    const double err = fabs(got - want);
    if (err <= tol) {
        ++g_pass;
        printf("  ok    %-46s %14.6f\n", name, got);
    } else {
        ++g_fail;
        printf("  FAIL  %-46s %14.6f (want %.6f, err %.3g)\n",
               name, got, want, err);
    }
}

static void chk_true(const char *name, int cond)
{
    if (cond) { ++g_pass; printf("  ok    %-46s\n", name); }
    else      { ++g_fail; printf("  FAIL  %-46s\n", name); }
}

/* ---------------------------------------------------------------- Case 1 */
static void case1_adaptive_r(void)
{
    puts("\nCase 1 - adaptive R inflation (reference table)");
    attitude_ekf_t e;
    attitude_ekf_init(&e, NULL);

    /* |a| = 1.20 g -> exactly at the threshold, no inflation yet */
    attitude_ekf_update_accel(&e, 0.0f, 0.0f, 1.20f);
    chk("|a|=1.20g  R factor", e.last_r_used / EKF_R_ACCEL_DEFAULT, 1.0, 1e-5);

    /* 1.30 g -> excess 0.10, factor (1+10*0.10)^2 = 4 */
    attitude_ekf_update_accel(&e, 0.0f, 0.0f, 1.30f);
    chk("|a|=1.30g  R factor", e.last_r_used / EKF_R_ACCEL_DEFAULT, 4.0, 1e-4);

    /* 1.40 g -> excess 0.20, factor (1+2)^2 = 9 */
    attitude_ekf_update_accel(&e, 0.0f, 0.0f, 1.40f);
    chk("|a|=1.40g  R factor", e.last_r_used / EKF_R_ACCEL_DEFAULT, 9.0, 1e-4);

    /* 1.50 g -> at the reject threshold, update skipped entirely */
    const bool applied = attitude_ekf_update_accel(&e, 0.0f, 0.0f, 1.50f);
    chk_true("|a|=1.50g  update rejected", applied == false);
    chk_true("|a|=1.50g  flag set", e.last_update_rejected == true);
}

/* ---------------------------------------------------------------- Case 2 */
static void case2_steady_state(void)
{
    puts("\nCase 2 - steady-state covariance and gain, level and static");
    attitude_ekf_t e;
    attitude_ekf_init(&e, NULL);

    for (int i = 0; i < 60000; ++i) {
        attitude_ekf_step(&e, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.002f);
    }

    chk("angle stays at zero            [rad]", attitude_ekf_roll(&e), 0.0, 1e-9);
    chk("bias  stays at zero          [rad/s]", attitude_ekf_bias_roll(&e), 0.0, 1e-9);
    chk("P00 steady state             [rad^2]", attitude_ekf_var_roll(&e), 9.978088e-05, 1e-7);
    chk("sigma_angle                    [deg]", DEG(sqrtf(attitude_ekf_var_roll(&e))), 0.5723, 1e-3);
}

/* ---------------------------------------------------------------- Case 3 */
static void case3_bias_recovery(void)
{
    puts("\nCase 3 - un-modelled constant gyro bias 0.02 rad/s, held level");
    attitude_ekf_t e;
    attitude_ekf_init(&e, NULL);

    const float TRUE_BIAS = 0.02f;
    for (int i = 0; i < 500; ++i) {              /* 1.00 s */
        attitude_ekf_step(&e, TRUE_BIAS, 0.0f, 0.0f, 0.0f, 1.0f, 0.002f);
    }
    chk("t=1s   bias estimate         [rad/s]", attitude_ekf_bias_roll(&e), 0.018474, 2e-5);
    chk("t=1s   angle error             [deg]", DEG(attitude_ekf_roll(&e)), 0.0172, 2e-3);

    for (int i = 0; i < 4500; ++i) {             /* to 10.0 s */
        attitude_ekf_step(&e, TRUE_BIAS, 0.0f, 0.0f, 0.0f, 1.0f, 0.002f);
    }
    chk("t=10s  bias estimate         [rad/s]", attitude_ekf_bias_roll(&e), 0.019897, 2e-5);
    chk_true("t=10s  bias within 1% of truth",
             fabsf(attitude_ekf_bias_roll(&e) - TRUE_BIAS) < 0.01f * TRUE_BIAS);
}

/* ---------------------------------------------------------------- Case 4 */
static void case4_step_response(void)
{
    puts("\nCase 4 - step tilt to 30 deg, gyro silent (accel-only correction)");
    attitude_ekf_t e;
    attitude_ekf_init(&e, NULL);

    for (int i = 0; i < 3000; ++i) {             /* settle at zero */
        attitude_ekf_step(&e, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.002f);
    }

    /* Hold at 30 deg roll: a = (0, sin30, cos30) g */
    const float ay = sinf(RAD(30.0f)), az = cosf(RAD(30.0f));

    float t63 = -1.0f;
    for (int i = 1; i <= 2000; ++i) {
        attitude_ekf_step(&e, 0.0f, 0.0f, 0.0f, ay, az, 0.002f);
        if (t63 < 0.0f && DEG(attitude_ekf_roll(&e)) >= 30.0f * 0.632f) {
            t63 = i * 0.002f;
        }
        if (i == 100) chk("t=0.200s angle                 [deg]", DEG(attitude_ekf_roll(&e)), 19.6100, 5e-3);
        if (i == 500) chk("t=1.000s angle                 [deg]", DEG(attitude_ekf_roll(&e)), 30.6913, 5e-3);
    }
    chk("63% rise time                    [s]", t63, 0.190, 3e-3);
}

/* ---------------------------------------------------------------- Case 5 */
static void case5_fusion(void)
{
    puts("\nCase 5 - inverse-covariance fusion");
    attitude_ekf_t a, b;
    attitude_ekf_init(&a, NULL);
    attitude_ekf_init(&b, NULL);
    imu_fusion_result_t r;

    /* equal confidence -> plain midpoint */
    a.roll.x[0] = RAD(2.0f);  a.roll.P[0][0] = 1.0e-4f;
    b.roll.x[0] = RAD(3.0f);  b.roll.P[0][0] = 1.0e-4f;
    imu_fusion_combine(&a, &b, IMU_DISAGREE_THRESH_RAD, &r);
    chk("equal P     -> fused           [deg]", DEG(r.roll), 2.5000, 1e-4);
    chk("equal P     -> fused variance       ", r.var_roll, 5.0e-5, 1e-8);

    /* B four times less certain -> pulled toward A */
    b.roll.P[0][0] = 4.0e-4f;
    imu_fusion_combine(&a, &b, IMU_DISAGREE_THRESH_RAD, &r);
    chk("P_b = 4*P_a -> fused           [deg]", DEG(r.roll), 2.2000, 1e-4);
    chk("P_b = 4*P_a -> fused variance       ", r.var_roll, 8.0e-5, 1e-8);

    /* B nearly worthless -> fused sits almost on A */
    b.roll.x[0] = RAD(10.0f); b.roll.P[0][0] = 1.0e-2f;
    imu_fusion_combine(&a, &b, IMU_DISAGREE_THRESH_RAD, &r);
    chk("P_b = 100*P_a -> fused         [deg]", DEG(r.roll), 2.0792, 1e-4);
}

/* ---------------------------------------------------------------- Case 6 */
static void case6_disagreement(void)
{
    puts("\nCase 6 - disagreement as gravity-vector angle (singularity-free)");

    chk("identical                      [deg]",
        DEG(imu_disagreement_rad(0, 0, 0, 0)), 0.0, 1e-4);
    chk("10 deg of roll apart           [deg]",
        DEG(imu_disagreement_rad(0, 0, RAD(10), 0)), 10.0, 1e-3);
    chk("10 deg of pitch apart          [deg]",
        DEG(imu_disagreement_rad(0, 0, 0, RAD(10))), 10.0, 1e-3);
    chk("10 roll + 10 pitch             [deg]",
        DEG(imu_disagreement_rad(0, 0, RAD(10), RAD(10))), 14.1060, 1e-3);

    /* The case that justifies the whole approach: in balance mode (pitch ~90)
     * roll is gimbal-locked, so a 60 deg roll difference is only 3.46 deg of
     * real disagreement. A naive |roll_a - roll_b| test would fault here. */
    const float d = DEG(imu_disagreement_rad(RAD(30), RAD(88), RAD(-30), RAD(92)));
    chk("balance mode, 60 deg roll diff [deg]", d, 3.4639, 1e-3);
    chk_true("  ... and does NOT trip the 15 deg fault", d < 15.0f);

    /* Fault threshold behaviour */
    attitude_ekf_t a, b;
    attitude_ekf_init(&a, NULL);
    attitude_ekf_init(&b, NULL);
    imu_fusion_result_t r;

    b.roll.x[0] = RAD(14.0f);
    imu_fusion_combine(&a, &b, IMU_DISAGREE_THRESH_RAD, &r);
    chk_true("14 deg apart -> no fault", r.fault == false);

    b.roll.x[0] = RAD(16.0f);
    imu_fusion_combine(&a, &b, IMU_DISAGREE_THRESH_RAD, &r);
    chk_true("16 deg apart -> FAULT", r.fault == true);
}

/* ---------------------------------------------------------------- Case 7 */
static void case7_mounting(void)
{
    puts("\nCase 7 - sensor-to-body axis remap");

    const imu_mount_t id   = IMU_MOUNT_IDENTITY;
    const imu_mount_t z180 = IMU_MOUNT_ROT_Z_180;
    const imu_mount_t z90  = IMU_MOUNT_ROT_Z_90;
    const imu_mount_t bad  = { { +1, +1, +3 } };
    const imu_mount_t mirr = { { +2, +1, +3 } };

    float in[3] = { 0.10f, 0.20f, 0.97f }, out[3];

    imu_apply_mount(&id, in, out);
    chk("identity  x", out[0],  0.10, 1e-6);
    chk("identity  y", out[1],  0.20, 1e-6);

    imu_apply_mount(&z180, in, out);
    chk("rot Z 180 x", out[0], -0.10, 1e-6);
    chk("rot Z 180 y", out[1], -0.20, 1e-6);
    chk("rot Z 180 z", out[2],  0.97, 1e-6);

    imu_apply_mount(&z90, in, out);
    chk("rot Z 90  x (= +sensor y)", out[0],  0.20, 1e-6);
    chk("rot Z 90  y (= -sensor x)", out[1], -0.10, 1e-6);

    chk_true("identity is valid",              imu_mount_is_valid(&id));
    chk_true("duplicate axis rejected",       !imu_mount_is_valid(&bad));
    chk_true("identity is right-handed",       imu_mount_is_right_handed(&id));
    chk_true("rot Z 180 is right-handed",      imu_mount_is_right_handed(&z180));
    chk_true("rot Z 90 is right-handed",       imu_mount_is_right_handed(&z90));
    chk_true("mirrored map rejected",         !imu_mount_is_right_handed(&mirr));

    /* The failure this whole layer exists to prevent: an un-corrected 180 deg
     * mounting makes two healthy, identical sensors look catastrophically
     * broken. */
    float raw[3] = { 0.0f, sinf(RAD(10.0f)), cosf(RAD(10.0f)) };
    float flipped[3];
    imu_apply_mount(&z180, raw, flipped);
    const float bad_disagree = DEG(imu_disagreement_rad(
        atan2f(raw[1], raw[2]), atan2f(-raw[0], hypotf(raw[1], raw[2])),
        atan2f(flipped[1], flipped[2]), atan2f(-flipped[0], hypotf(flipped[1], flipped[2]))));
    chk("un-corrected 180 mount reads as[deg]", bad_disagree, 20.0, 1e-3);
    chk_true("  ... which WOULD trip the 15 deg fault", bad_disagree > 15.0f);
}

/* ---------------------------------------------------------------- Case 8 */
static void case8_side_detect(void)
{
    puts("\nCase 8 - down-side detection with rest gate");

    chk_true("flat, at rest        -> Z_UP",
             imu_detect_side(0.0f, 0.0f, 1.0f, 0.0f) == IMU_SIDE_Z_UP);
    chk_true("inverted, at rest    -> Z_DOWN",
             imu_detect_side(0.0f, 0.0f, -1.0f, 0.0f) == IMU_SIDE_Z_DOWN);
    chk_true("balance pose (nose up)-> X_DOWN",
             imu_detect_side(-1.0f, 0.0f, 0.0f, 0.0f) == IMU_SIDE_X_DOWN);
    chk_true("spinning             -> UNKNOWN",
             imu_detect_side(0.0f, 0.0f, 1.0f, 1.0f) == IMU_SIDE_UNKNOWN);
    chk_true("accelerating (1.5g)  -> UNKNOWN",
             imu_detect_side(0.0f, 0.0f, 1.5f, 0.0f) == IMU_SIDE_UNKNOWN);
    chk_true("45 deg corner        -> UNKNOWN",
             imu_detect_side(0.0f, 0.7071f, 0.7071f, 0.0f) == IMU_SIDE_UNKNOWN);
}

/* ---------------------------------------------------------------- Case 9 */
static void case9_pid(void)
{
    puts("\nCase 9 - PID arithmetic (kp=8, ki=2, kd=0.4, out +-100, imax=30)");
    pid_config_t c;
    pid_config_defaults(&c);
    c.kp = 8.0f; c.ki = 2.0f; c.kd = 0.4f;
    c.out_min = -100.0f; c.out_max = 100.0f; c.integral_max = 30.0f;

    pid_t p;
    pid_init(&p, &c);

    float u;
    u = pid_update(&p, 0.0f,  0.000f,  0.00f, 0.002f);
    chk("n=1 out", u, 0.00000, 1e-5);
    u = pid_update(&p, 0.0f, -0.050f,  0.00f, 0.002f);
    chk("n=2 out", u, 0.40020, 1e-5);
    u = pid_update(&p, 0.0f, -0.050f, -0.10f, 0.002f);
    chk("n=3 out", u, 0.44040, 1e-5);
    u = pid_update(&p, 0.0f, -0.050f, -0.10f, 0.002f);
    chk("n=4 out", u, 0.44060, 1e-5);
    u = pid_update(&p, 0.0f, -0.050f, -0.10f, 0.002f);
    chk("n=5 out", u, 0.44080, 1e-5);
    u = pid_update(&p, 0.0f,  0.000f,  0.20f, 0.002f);
    chk("n=6 out", u, -0.07920, 1e-5);
    u = pid_update(&p, 0.0f,  0.100f,  0.20f, 0.002f);
    chk("n=7 out", u, -0.87960, 1e-5);
}

/* --------------------------------------------------------------- Case 10 */
static void case10_antiwindup(void)
{
    puts("\nCase 10 - anti-windup: integrator frozen while saturated");
    pid_config_t c;
    pid_config_defaults(&c);
    c.kp = 8.0f; c.ki = 2.0f; c.kd = 0.4f;
    c.out_min = -100.0f; c.out_max = 100.0f; c.integral_max = 30.0f;

    pid_t p;
    pid_init(&p, &c);

    /* Error of -20 rad -> P = -160, hard against the -100 floor for 6 s. */
    float u = 0.0f;
    for (int i = 0; i < 3000; ++i) {
        u = pid_update(&p, 0.0f, 20.0f, 0.0f, 0.002f);
    }
    chk("output pinned at floor", u, -100.0, 1e-4);
    chk("integrator after 6s saturated", pid_integral(&p), 0.0, 1e-12);
    chk_true("saturation flag set", pid_saturated(&p));

    /* Error flips sign: the integrator must be free again immediately. */
    u = pid_update(&p, 0.0f, -1.0f, 0.0f, 0.002f);
    chk("recovery n=1 integral", pid_integral(&p), 0.004, 1e-6);
    chk("recovery n=1 out",      u,                8.004, 1e-4);
    chk_true("saturation flag cleared", !pid_saturated(&p));
}

/* --------------------------------------------------------------- Case 11 */
static void case11_velocity_bias(void)
{
    puts("\nCase 11 - outer velocity-bias loop (lean target from step integral)");
    vel_bias_config_t c;
    vel_bias_config_defaults(&c);
    c.gain           = 1.2e-5f;
    c.lean_limit_rad = RAD(6.0f);
    c.trim_rad       = 0.0f;

    vel_bias_t v;
    vel_bias_init(&v, &c);

    float tgt = 0.0f;
    for (int i = 1; i <= 100; ++i) {              /* 20 Hz outer loop */
        tgt = vel_bias_update(&v, 3000.0f, 0.05f);
        if (i == 20) chk("t=1.00s lean target        [deg]", DEG(tgt), -2.0626, 1e-3);
    }
    chk("t=5.00s lean target clamped[deg]", DEG(tgt), -6.0000, 1e-4);
    chk_true("clamp engaged", fabsf(DEG(tgt)) <= 6.0f + 1e-4f);

    /* Trim adds a static offset — this is where §7d's IMU trim pot lands. */
    vel_bias_init(&v, &c);
    v.cfg.trim_rad = RAD(1.5f);
    tgt = vel_bias_update(&v, 0.0f, 0.05f);
    chk("zero drive, trim only      [deg]", DEG(tgt), 1.5000, 1e-4);
}


/* --------------------------------------------------------------- Case 12 */
/* Synthesise what a sensor would read under a KNOWN mounting, then check the
 * resolver recovers that mounting. Exhaustive over all 24 right-handed signed
 * permutations — if it works for every physically possible mounting, it works. */
static void to_sensor(const imu_mount_t *m, const float body[3], float sensor[3])
{
    /* body[i] = sign(code) * sensor[|code|-1]   =>   sensor[|code|-1] = sign*body[i] */
    for (int i = 0; i < 3; ++i) {
        const int code = m->map[i];
        const int idx  = (code < 0 ? -code : code) - 1;
        sensor[idx] = (code < 0) ? -body[i] : body[i];
    }
}

static void case12_mount_resolver(void)
{
    puts("\nCase 12 - mount resolver, exhaustive over all right-handed mountings");

    /* Body-frame truth for the two calibration poses. */
    const float level[3]     = { 0.0f, 0.0f, 1.0f };            /* flat        */
    const float tilt         = RAD(35.0f);                       /* nose down   */
    const float nose_down[3] = { -sinf(tilt), 0.0f, cosf(tilt) };

    int tested = 0, recovered = 0;

    for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
    for (int sz = -1; sz <= 1; sz += 2)
    for (int px = 1; px <= 3; ++px)
    for (int py = 1; py <= 3; ++py)
    for (int pz = 1; pz <= 3; ++pz) {
        if (px == py || py == pz || px == pz) continue;

        imu_mount_t truth;
        truth.map[0] = (signed char)(sx * px);
        truth.map[1] = (signed char)(sy * py);
        truth.map[2] = (signed char)(sz * pz);
        if (!imu_mount_is_right_handed(&truth)) continue;

        ++tested;

        float s_level[3], s_nose[3];
        to_sensor(&truth, level,     s_level);
        to_sensor(&truth, nose_down, s_nose);

        imu_mount_t got;
        if (!imu_mount_resolve(s_level, s_nose, &got)) continue;

        if (got.map[0] == truth.map[0] &&
            got.map[1] == truth.map[1] &&
            got.map[2] == truth.map[2]) {
            ++recovered;
        }
    }

    printf("  tested %d right-handed mountings, recovered %d\n", tested, recovered);
    chk_true("all 24 right-handed mountings enumerated", tested == 24);
    chk_true("every one recovered exactly", recovered == tested);

    /* The two mountings that actually matter on this board. */
    const imu_mount_t want_a = IMU_MOUNT_IDENTITY;
    const imu_mount_t want_b = IMU_MOUNT_ROT_Z_180;
    float sl[3], sn[3];
    imu_mount_t got;

    to_sensor(&want_a, level, sl); to_sensor(&want_a, nose_down, sn);
    chk_true("IMU A (identity) resolves", imu_mount_resolve(sl, sn, &got)
             && got.map[0]==want_a.map[0] && got.map[1]==want_a.map[1] && got.map[2]==want_a.map[2]);

    to_sensor(&want_b, level, sl); to_sensor(&want_b, nose_down, sn);
    chk_true("IMU B (rot Z 180) resolves", imu_mount_resolve(sl, sn, &got)
             && got.map[0]==want_b.map[0] && got.map[1]==want_b.map[1] && got.map[2]==want_b.map[2]);

    /* Failure modes must fail, not guess. */
    const float not_level[3] = { 0.60f, 0.0f, 0.60f };   /* 45 deg, not level  */
    chk_true("non-level 'level' pose rejected",
             !imu_mount_resolve(not_level, sn, &got));

    const float barely[3] = { -0.05f, 0.0f, 0.999f };    /* ~3 deg of tilt     */
    chk_true("insufficient tilt rejected",
             !imu_mount_resolve(level, barely, &got));

    chk_true("NULL args rejected", !imu_mount_resolve(NULL, sn, &got));

    /* Sign convention: nose-down must read POSITIVE pitch. Counter-intuitive
     * against the aerospace convention, so assert it before someone "fixes" it. */
    const float p = DEG(atan2f(-nose_down[0], hypotf(nose_down[1], nose_down[2])));
    chk("nose-down 35 deg reads pitch    [deg]", p, 35.0, 1e-3);
    chk_true("  ... POSITIVE pitch = nose DOWN", p > 0.0f);
}

int main(void)
{
    puts("OMNIS control-stack verification");
    puts("expected values independently computed in Python; see the reference docs");

    case1_adaptive_r();
    case2_steady_state();
    case3_bias_recovery();
    case4_step_response();
    case5_fusion();
    case6_disagreement();
    case7_mounting();
    case8_side_detect();
    case9_pid();
    case10_antiwindup();
    case11_velocity_bias();
    case12_mount_resolver();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
