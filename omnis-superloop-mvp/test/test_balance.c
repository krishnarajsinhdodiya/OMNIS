/**
 * @file    test_balance.c
 * @brief   Host-side verification of the balance controller, including a
 *          closed-loop inverted-pendulum simulation on BOTH wheel pairs.
 *
 * The plant below maps wheel omega to cart acceleration with the rolling
 * relation derived in balance.h (sigma = +1 front, -1 rear). A controller whose
 * pair mapping disagreed with that plant would make one pair fall over here.
 * What this cannot prove is that the derivation matches the physical robot —
 * that is the bench check in TESTING.md, and why output_invert exists.
 */

#include <math.h>
#include <stdio.h>

#include "balance.h"

#define RAD(d) ((d) * 3.14159265358979323846f / 180.0f)
#define DEG(r) ((r) * 180.0f / 3.14159265358979323846f)

static int g_fail = 0, g_pass = 0;

static void chk(const char *name, double got, double want, double tol)
{
    if (fabs(got - want) <= tol) { ++g_pass; printf("  ok    %-58s %12.4f\n", name, got); }
    else { ++g_fail; printf("  FAIL  %-58s %12.4f (want %.4f)\n", name, got, want); }
}

static void chk_true(const char *name, int cond)
{
    if (cond) { ++g_pass; printf("  ok    %-58s\n", name); }
    else      { ++g_fail; printf("  FAIL  %-58s\n", name); }
}

/* Plant constants for the simulation. Not the real robot — just a physically
 * plausible small balancer, so the controller's SIGNS and STRUCTURE are tested. */
#define WHEEL_R_M   0.030f
#define CG_HEIGHT_M 0.15f
#define G           9.81f
/* steps/s -> m/s at the wheel rim */
#define STEPS_TO_M  (WHEEL_R_M / (float)STEPS_PER_RAD)

static balance_cfg_t sim_cfg(void)
{
    /* Target closed loop: a = 30 m/s^2 per rad + 3 m/s^2 per rad/s, comfortably
     * above g / l for this pendulum. Converted to wheel steps/s^2. */
    balance_cfg_t c = {
        .kp = 30.0f / STEPS_TO_M,
        .ki = 0.0f,
        .kd = 3.0f / STEPS_TO_M,
        .integral_max = 0.0f,
        .max_wheel_accel = 1.0e7f,
        .max_wheel_rate = 7000.0f,
        .trim_rad = 0.0f,
        .stick_lean_max_rad = RAD(3.0f),
        .turn_max_steps = 600.0f,
        .lean_limit_rad = RAD(6.0f),
        .vel_bias_gain = 0.0f,
        .outer_period_s = 0.05f,
        .output_invert = false,
    };
    return c;
}

/* Grounded-pair mean wheel omega [steps/s], as the robot would realise it. */
static float grounded_omega(omnis_balance_pair_t pair, const wheel_rates_t *w)
{
    return (pair == OMNIS_BALANCE_ON_FRONT_PAIR) ? 0.5f * (w->fl + w->fr)
                                                 : 0.5f * (w->rl + w->rr);
}

/* Physical sign from the contact-point derivation in balance.h: forward speed
 * of the base (toward the top face) = sigma * omega. Written out again here,
 * separately from balance_pair_sign(), on purpose. */
static float plant_sigma(omnis_balance_pair_t pair)
{
    return (pair == OMNIS_BALANCE_ON_FRONT_PAIR) ? +1.0f : -1.0f;
}

typedef struct { float final_deg; float peak_deg; float peak_rate; } sim_result_t;

static sim_result_t simulate(omnis_balance_pair_t pair, const balance_cfg_t *cfg,
                             float theta0_deg, float seconds)
{
    balance_ctrl_t b;
    balance_init(&b, cfg);
    float theta = RAD(theta0_deg), theta_dot = 0.0f;
    float omega_prev = 0.0f;
    sim_result_t r = { 0.0f, 0.0f, 0.0f };
    const float dt = 0.002f;
    const int n = (int)(seconds / dt);
    for (int i = 0; i < n; ++i) {
        wheel_rates_t w;
        balance_step(&b, cfg, pair, theta, theta_dot, 0.0f, 0.0f, dt, &w);
        const float omega = grounded_omega(pair, &w);
        /* base forward acceleration from the change in wheel speed */
        const float a = plant_sigma(pair) * (omega - omega_prev) / dt * STEPS_TO_M;
        omega_prev = omega;
        /* inverted pendulum on a cart; + theta = falling toward the top face */
        const float theta_ddot = (G * sinf(theta) - a * cosf(theta)) / CG_HEIGHT_M;
        theta_dot += theta_ddot * dt;
        theta     += theta_dot * dt;
        if (fabsf(DEG(theta)) > r.peak_deg) r.peak_deg = fabsf(DEG(theta));
        if (fabsf(omega) > r.peak_rate) r.peak_rate = fabsf(omega);
        if (fabsf(theta) > RAD(80.0f)) break;   /* on the floor */
    }
    r.final_deg = DEG(theta);
    return r;
}

static void case_closed_loop(void)
{
    puts("\nClosed loop: inverted pendulum, released at 3 deg, 3 s");
    balance_cfg_t cfg = sim_cfg();

    sim_result_t f = simulate(OMNIS_BALANCE_ON_FRONT_PAIR, &cfg, 3.0f, 3.0f);
    chk("FRONT pair: lean after 3 s [deg]", f.final_deg, 0.0, 0.2);
    chk_true("FRONT pair: never fell (peak < 10 deg)", f.peak_deg < 10.0f);
    printf("        (peak lean %.2f deg, peak wheel %.0f steps/s)\n", f.peak_deg, f.peak_rate);

    sim_result_t r = simulate(OMNIS_BALANCE_ON_REAR_PAIR, &cfg, 3.0f, 3.0f);
    chk("REAR pair: lean after 3 s [deg]", r.final_deg, 0.0, 0.2);
    chk_true("REAR pair: never fell (peak < 10 deg)", r.peak_deg < 10.0f);
    printf("        (peak lean %.2f deg, peak wheel %.0f steps/s)\n", r.peak_deg, r.peak_rate);

    sim_result_t b = simulate(OMNIS_BALANCE_ON_FRONT_PAIR, &cfg, -3.0f, 3.0f);
    chk("FRONT pair, released leaning BACK: recovers [deg]", b.final_deg, 0.0, 0.2);

    balance_cfg_t inv = cfg;
    inv.output_invert = true;
    sim_result_t x = simulate(OMNIS_BALANCE_ON_FRONT_PAIR, &inv, 3.0f, 3.0f);
    chk_true("output_invert on a correct robot FALLS (proves the sign matters)",
             fabsf(x.final_deg) > 30.0f);

    balance_cfg_t zero = cfg;
    zero.kp = 0.0f; zero.kd = 0.0f;
    sim_result_t z = simulate(OMNIS_BALANCE_ON_FRONT_PAIR, &zero, 3.0f, 3.0f);
    chk_true("zero gains FALL (why the supervisor refuses to arm them)",
             fabsf(z.final_deg) > 30.0f);
}

static void case_open_loop_signs(void)
{
    puts("\nOpen-loop signs and mapping");
    balance_cfg_t cfg = sim_cfg();
    balance_ctrl_t b;
    wheel_rates_t w;

    balance_init(&b, &cfg);
    balance_step(&b, &cfg, OMNIS_BALANCE_ON_FRONT_PAIR, 0.0f, 0.0f, 0.0f, 0.0f, 0.002f, &w);
    chk_true("upright, no sticks -> all wheels zero",
             w.fl == 0.0f && w.fr == 0.0f && w.rl == 0.0f && w.rr == 0.0f);

    balance_init(&b, &cfg);
    balance_step(&b, &cfg, OMNIS_BALANCE_ON_FRONT_PAIR, RAD(2.0f), 0.0f, 0.0f, 0.0f, 0.002f, &w);
    chk_true("FRONT, falling forward -> FL,FR omega POSITIVE (roll toward top face)",
             w.fl > 0.0f && w.fr > 0.0f && w.fl == w.fr);
    chk_true("FRONT: airborne rear pair held at zero", w.rl == 0.0f && w.rr == 0.0f);

    balance_init(&b, &cfg);
    balance_step(&b, &cfg, OMNIS_BALANCE_ON_REAR_PAIR, RAD(2.0f), 0.0f, 0.0f, 0.0f, 0.002f, &w);
    chk_true("REAR, falling forward -> RL,RR omega NEGATIVE (the sign flips)",
             w.rl < 0.0f && w.rr < 0.0f && w.rl == w.rr);
    chk_true("REAR: airborne front pair held at zero", w.fl == 0.0f && w.fr == 0.0f);

    chk("pair sign FRONT", balance_pair_sign(OMNIS_BALANCE_ON_FRONT_PAIR),  1.0, 0);
    chk("pair sign REAR",  balance_pair_sign(OMNIS_BALANCE_ON_REAR_PAIR),  -1.0, 0);

    puts("\nTurning");
    balance_init(&b, &cfg);
    balance_step(&b, &cfg, OMNIS_BALANCE_ON_FRONT_PAIR, 0.0f, 0.0f, 0.0f, 1.0f, 0.002f, &w);
    chk("FRONT, full roll left: FL (left) = -turn", w.fl, -600.0, 1e-3);
    chk("FRONT, full roll left: FR (right) = +turn", w.fr, 600.0, 1e-3);

    balance_init(&b, &cfg);
    balance_step(&b, &cfg, OMNIS_BALANCE_ON_REAR_PAIR, 0.0f, 0.0f, 0.0f, 1.0f, 0.002f, &w);
    chk("REAR, full roll left: RR is the LEFT wheel, backward = +600", w.rr, 600.0, 1e-3);
    chk("REAR, full roll left: RL is the RIGHT wheel, forward = -600", w.rl, -600.0, 1e-3);
    chk_true("REAR turn: left wheel moves backward, right forward, in forward units",
             plant_sigma(OMNIS_BALANCE_ON_REAR_PAIR) * w.rr < 0.0f &&
             plant_sigma(OMNIS_BALANCE_ON_REAR_PAIR) * w.rl > 0.0f);

    puts("\nTargets, limits, reset");
    balance_init(&b, &cfg);
    balance_step(&b, &cfg, OMNIS_BALANCE_ON_FRONT_PAIR, 0.0f, 0.0f, 1.0f, 0.0f, 0.002f, &w);
    chk("full pitch stick -> lean target +3 deg", DEG(b.last_target), 3.0, 1e-4);
    /* Non-minimum-phase: from upright, the robot can only come to lean forward
     * if the base first moves BACK, out from under it. The first command is
     * therefore backward acceleration — the start of a controlled fall. */
    chk_true("  ... first moves the base BACK so the robot tips forward", b.last_accel < 0.0f);

    balance_init(&b, &cfg);
    for (int i = 0; i < 50; ++i) {
        balance_step(&b, &cfg, OMNIS_BALANCE_ON_FRONT_PAIR, RAD(1.0f), 0.0f, 0.0f, 0.0f, 0.002f, &w);
    }
    const float speed_before = b.fwd_speed;
    balance_set_kp(&b, cfg.kp * 0.5f);
    chk_true("balance_set_kp changes the gain without resetting state",
             b.pid.cfg.kp == cfg.kp * 0.5f && b.fwd_speed == speed_before && speed_before != 0.0f);
    balance_set_kp(&b, NAN);
    chk_true("balance_set_kp rejects NaN (gain becomes 0, not NaN)", b.pid.cfg.kp == 0.0f);

    balance_cfg_t lim = cfg;
    lim.trim_rad = RAD(5.0f);
    balance_init(&b, &lim);
    balance_step(&b, &lim, OMNIS_BALANCE_ON_FRONT_PAIR, 0.0f, 0.0f, 1.0f, 0.0f, 0.002f, &w);
    chk("trim 5 + stick 3 is clamped to lean_limit 6 [deg]", DEG(b.last_target), 6.0, 1e-4);

    balance_init(&b, &cfg);
    for (int i = 0; i < 2000; ++i) {
        balance_step(&b, &cfg, OMNIS_BALANCE_ON_FRONT_PAIR, RAD(20.0f), 0.0f, 0.0f, 0.0f, 0.002f, &w);
    }
    chk("sustained fall: forward speed clamps at max_wheel_rate", b.fwd_speed, 7000.0, 1e-2);
    chk_true("  ... and reports saturation", b.saturated);

    balance_reset(&b);
    chk_true("reset clears speed, saturation and integrators",
             b.fwd_speed == 0.0f && !b.saturated && pid_integral(&b.pid) == 0.0f);

    balance_init(&b, &cfg);
    balance_step(&b, &cfg, OMNIS_BALANCE_ON_FRONT_PAIR, NAN, 0.0f, 0.0f, 0.0f, 0.002f, &w);
    chk_true("NaN lean -> all wheels zero, state untouched",
             w.fl == 0.0f && w.fr == 0.0f && b.fwd_speed == 0.0f);
}

int main(void)
{
    puts("OMNIS balance controller verification");
    case_open_loop_signs();
    case_closed_loop();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
