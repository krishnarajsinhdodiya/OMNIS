/**
 * @file    omnis_params.c
 * @brief   Compile-time parameter defaults for the OMNIS superloop MVP.
 *
 * Stands in for §9f's params.json, which needs the SD card this build excludes.
 */

#include <stddef.h>
#include <math.h>

#include "omnis_params.h"

static omnis_params_t s_params;

void omnis_params_defaults(omnis_params_t *p)
{
    if (p == NULL) {
        return;
    }

    p->schema_version = OMNIS_PARAMS_SCHEMA_VERSION;

    /* --- Geometry: as-built, measured, not negotiable ------------------- */
    p->geometry.wheel_radius_mm = 30.0f;
    p->geometry.wheelbase_mm    = 223.0f;
    p->geometry.track_width_mm  = 230.0f;

    /* --- Step limits ----------------------------------------------------
     * max_step_rate comes from the kinematics reference §7 Case D: FR at
     * 7016 microsteps/s is 2.19 rev/s ~ 131 RPM, described there as "near the
     * practical ceiling of this build" because a NEMA17 on 12 V is well off its
     * torque curve by then. 7000 is that ceiling, rounded down.
     *
     * deadband is the reference §6 figure: below ~20 steps/s a wheel dithers
     * across the direction reversal and chatters, because the A4988 needs DIR
     * stable >=200 ns before a STEP edge.
     *
     * max_accel is a placeholder pending bench measurement — it is the §13c
     * lost-step defence, and the correct value depends on the Vref current
     * limit that has not been set yet (§3f). Deliberately conservative.
     * ------------------------------------------------------------------- */
    p->step.max_step_rate      = 7000.0f;
    p->step.deadband_steps     = 20.0f;
    p->step.max_accel_steps_s2 = 20000.0f;

    /* --- RC scaling -----------------------------------------------------
     * Derived so that a full-deflection single-axis command lands inside
     * max_step_rate rather than being clamped:
     *
     *   pure vx:  f = vx * STEPS_PER_RAD / r = vx * 509.2958 / 30 = 16.98 * vx
     *             7000 / 16.98 = 412 mm/s, so 300 leaves ~27% headroom for
     *             combined commands.
     *   pure w:   f = k*w * 509.2958 / r = 7.55 * 509.2958 * w = 3845 * w
     *             7000 / 3845 = 1.82 rad/s, so 1.5 leaves headroom.
     *
     * Unverified on hardware — see PLAN.md open question 6.
     * ------------------------------------------------------------------- */
    p->rc_scale.vx_max_mmps       = 300.0f;
    p->rc_scale.vx_secondary_mmps = 150.0f;
    p->rc_scale.vy_max_mmps       = 300.0f;
    p->rc_scale.w_max_radps       = 1.5f;

    /* Assume the transmitter matches the reference convention (stick left ->
     * positive vy and positive/CCW yaw). Unverified until a radio is bound in
     * Stage 4 — if a stick turns out reversed, flip it here, never in the IK. */
    p->rc_scale.invert_throttle   = false;
    p->rc_scale.invert_pitch      = false;
    p->rc_scale.invert_roll       = false;
    p->rc_scale.invert_yaw        = false;

    /* --- CRSF channel map: §9f rc.channel_map, 1-based ------------------ */
    /* NOTE: EdgeTX's default channel order for a new model is AETR — ch1 aileron
     * (roll), ch2 elevator (pitch), ch3 throttle, ch4 rudder (yaw). This map is
     * §9f's T-E-A-R order. Either reorder the mixer on the RadioMaster Pocket or
     * change these four numbers; TESTING.md Stage 4 has the check. */
    p->channel_map.throttle          = 1;
    p->channel_map.pitch             = 2;
    p->channel_map.roll              = 3;
    p->channel_map.yaw               = 4;
    p->channel_map.kill_switch       = 6;
    p->channel_map.drive_mode        = 8;
    p->channel_map.speed_limiter     = 9;
    p->channel_map.tune_pot          = 10;
    p->channel_map.arm_switch_invert = false;

    /* --- Balance ---------------------------------------------------------
     * ZERO GAINS ON PURPOSE. omnis-info.md §13d declines to guess PID gains
     * without mass, CG height and wheel radius, and that refusal is correct: a
     * zero-gain controller does nothing, which is visible and safe, whereas a
     * guessed-gain controller looks like it works and falls over. The tuning
     * procedure is in assets/control/pid-reference.md §5.
     * ------------------------------------------------------------------- */
    p->balance.kp             = 0.0f;
    p->balance.ki             = 0.0f;
    p->balance.kd             = 0.0f;
    p->balance.integral_max   = 0.0f;
    p->balance.trim_rad       = 0.0f;
    p->balance.lean_limit_rad = 0.10472f;   /* 6 degrees */
    p->balance.vel_bias_gain  = 0.0f;

    /* --- IMU -------------------------------------------------------------
     * disagree 15 deg: omnis-info.md §11c.
     * comm_fail 25 reads: 50 ms at 500 Hz. Long enough to ride out a single
     *   bus glitch, short enough that a dead IMU is caught within a few of the
     *   balance loop's own time constants.
     * frozen 50 reads: a live MPU6050's noise floor changes the 14-byte burst
     *   every sample; 100 ms of identical bytes is a latched or dead part. On
     *   board Rev 2.0 the shared INT line cannot reveal a dead IMU, so this and
     *   the error count are the only way to notice one.
     * cal 1000 samples = 2 s: §12 asks for ~5 s; 2 s already averages the gyro
     *   noise down ~30x, and a shorter wait is more likely to actually be still.
     * gyro sd 0.02 rad/s (~1.1 deg/s): ~20x the MPU6050 noise floor, so it
     *   passes on a table and fails when the robot is being handled.
     * flat tilt 45 deg: an armed robot in 4-wheel mode tipped this far has
     *   crashed, flipped, or is being picked up. Stop stepping.
     * ------------------------------------------------------------------- */
    p->imu.disagree_thresh_rad     = 0.2617994f;   /* 15 deg */
    p->imu.comm_fail_reads         = 25u;
    p->imu.frozen_reads            = 50u;
    p->imu.cal_samples             = 1000u;
    p->imu.cal_attempts            = 5u;
    p->imu.cal_gyro_std_max_radps  = 0.02f;
    p->imu.cal_accel_std_max_g     = 0.02f;
    p->imu.cal_accel_tol_g         = 0.10f;
    p->imu.flat_tilt_fault_rad     = 0.7853982f;   /* 45 deg */

    /* --- Radio -----------------------------------------------------------
     * 420000 baud is the ExpressLRS CRSF default. The 4% deadzone absorbs
     * gimbal centring error; arming requires all sticks within 10% so a
     * non-centring throttle stick left at the bottom cannot arm the robot into
     * full reverse. LQ floor off by default: the frame timeout is the primary
     * link-loss detector, and a floor that is too high disarms on a fringe link.
     * ------------------------------------------------------------------- */
    p->rc.crsf_baud        = 420000u;
    p->rc.stick_deadzone   = 0.04f;
    p->rc.center_tolerance = 0.10f;
    p->rc.pin_probe_ms     = 1500u;
    p->rc.min_link_quality = 0u;

    /* §9f control block. Start on LOW for every first test. */
    p->control.speed_low  = 0.3f;
    p->control.speed_med  = 0.6f;
    p->control.speed_high = 1.0f;

    /* --- Motors ----------------------------------------------------------
     * dir_invert: right side inverted, on the assumption that all four motors
     * are wired identically and mounted shaft-outward, which makes the right
     * side mirror the left. UNVERIFIED — the Stage 5 bench pattern's "forward"
     * segment settles it wheel by wheel (TESTING.md).
     *
     * 10 us pulses and 10 us minimum lows: ten times the A4988's 1 us minimum,
     * margin for slow edges on long harnesses, and still a 50 000 steps/s
     * ceiling against a 7000 steps/s operating limit.
     * ------------------------------------------------------------------- */
    p->motor.dir_invert[0]   = false;   /* FL */
    p->motor.dir_invert[1]   = true;    /* FR */
    p->motor.dir_invert[2]   = false;   /* RL */
    p->motor.dir_invert[3]   = true;    /* RR */
    p->motor.step_pulse_us   = 10u;
    p->motor.step_min_low_us = 10u;

    /* --- Safety / supervisor ---------------------------------------------
     * settle 400 ms: two EKF time constants (tau = 0.198 s) after the filters
     *   are re-seeded in the balance frame, before the balance loop may drive.
     * upright 60 deg: "standing on a pair" — comfortably past anything a flat
     *   robot on a ramp reaches, comfortably short of the 90 deg balance pose.
     * arm flat 30 deg: a robot tipped further than this is being carried or is
     *   on its side, and must not start driving four wheels.
     * overruns 25 per 500-tick window: 5% of deadlines missed means the loop's
     *   dt, and so the EKF and PID, can no longer be trusted. The telemetry line's
     *   own overruns are excluded (main.c).
     * ------------------------------------------------------------------- */
    p->safety.balance_settle_ms     = 400u;
    p->safety.upright_min_rad       = 1.0471976f;   /* 60 deg */
    p->safety.arm_flat_max_tilt_rad = 0.5235988f;   /* 30 deg */
    p->safety.overrun_window_ticks  = 500u;
    p->safety.overrun_fault_count   = 25u;

    /* --- RC link loss (§7f) ---------------------------------------------
     * 250 ms is comfortably longer than the slowest ExpressLRS packet interval
     * (50 Hz = 20 ms), so a few dropped frames do not fire it, but short enough
     * that a runaway is caught fast. §7f is explicit that hold-last-value is the
     * wrong behaviour here.
     * ------------------------------------------------------------------- */
    p->rc_timeout_us = 250000u;
}

bool omnis_params_valid(const omnis_params_t *p)
{
    if (p == NULL) {
        return false;
    }
    if (p->schema_version != OMNIS_PARAMS_SCHEMA_VERSION) {
        return false;
    }

    /* Geometry divides in the kinematics — a zero here is a divide-by-zero
     * that would surface as inf step rates rather than an obvious fault. */
    if (!(p->geometry.wheel_radius_mm > 0.0f) ||
        !(p->geometry.wheelbase_mm    > 0.0f) ||
        !(p->geometry.track_width_mm  > 0.0f)) {
        return false;
    }
    if (!isfinite(p->geometry.wheel_radius_mm) ||
        !isfinite(p->geometry.wheelbase_mm) ||
        !isfinite(p->geometry.track_width_mm)) {
        return false;
    }

    if (!(p->step.max_step_rate > 0.0f)) {
        return false;
    }
    if (p->rc_timeout_us == 0u) {
        return false;
    }

    if (!(p->imu.disagree_thresh_rad > 0.0f) || p->imu.cal_samples < 50u ||
        p->imu.cal_attempts == 0u || p->imu.comm_fail_reads == 0u ||
        p->imu.frozen_reads == 0u || !(p->imu.cal_gyro_std_max_radps > 0.0f) ||
        !(p->imu.flat_tilt_fault_rad > 0.0f)) {
        return false;
    }

    if (p->motor.step_pulse_us < 2u || p->motor.step_pulse_us > 1000u ||
        p->motor.step_min_low_us < 2u || !(p->step.max_accel_steps_s2 > 0.0f) ||
        !(p->step.deadband_steps >= 0.0f)) {
        return false;
    }

    if (!(p->safety.arm_flat_max_tilt_rad > 0.0f) ||
        !(p->safety.upright_min_rad > p->safety.arm_flat_max_tilt_rad) ||
        p->safety.upright_min_rad > 1.5708f || p->safety.overrun_window_ticks == 0u ||
        p->safety.overrun_fault_count == 0u) {
        return false;
    }

    /* Channel numbers are 1-based into a 16-channel CRSF frame. */
    if (p->rc.crsf_baud == 0u || !(p->rc.stick_deadzone >= 0.0f) ||
        p->rc.stick_deadzone >= 0.5f || !(p->rc.center_tolerance > 0.0f) ||
        p->rc.center_tolerance >= 1.0f) {
        return false;
    }
    if (!(p->control.speed_low > 0.0f) || p->control.speed_low > 1.0f ||
        !(p->control.speed_med > 0.0f) || p->control.speed_med > 1.0f ||
        !(p->control.speed_high > 0.0f) || p->control.speed_high > 1.0f) {
        return false;
    }

    const uint8_t ch[] = { p->channel_map.throttle,    p->channel_map.pitch,
                           p->channel_map.roll,        p->channel_map.yaw,
                           p->channel_map.kill_switch, p->channel_map.drive_mode,
                           p->channel_map.speed_limiter, p->channel_map.tune_pot };
    for (size_t i = 0; i < sizeof(ch) / sizeof(ch[0]); ++i) {
        if (ch[i] < 1u || ch[i] > 16u) {
            return false;
        }
    }

    return true;
}

void omnis_params_init(void)
{
    omnis_params_defaults(&s_params);
}

const omnis_params_t *omnis_params(void)
{
    return &s_params;
}
