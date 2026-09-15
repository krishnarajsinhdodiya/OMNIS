/**
 * @file    test_supervisor.c
 * @brief   Host-side verification of the fault latch, buzzer patterns and the
 *          arming supervisor — the code that decides whether motors may move.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "buzzer_pattern.h"
#include "fault.h"
#include "supervisor.h"

#define RAD(d) ((d) * 3.14159265358979323846f / 180.0f)

static int g_fail = 0, g_pass = 0;

static void chk_true(const char *name, int cond)
{
    if (cond) { ++g_pass; printf("  ok    %-60s\n", name); }
    else      { ++g_fail; printf("  FAIL  %-60s\n", name); }
}

static void chk_str(const char *name, const char *got, const char *want)
{
    if (strcmp(got, want) == 0) { ++g_pass; printf("  ok    %-60s \"%s\"\n", name, got); }
    else { ++g_fail; printf("  FAIL  %-60s \"%s\" (want \"%s\")\n", name, got, want); }
}

/* ------------------------------------------------------------------ faults */
static void case_faults(void)
{
    puts("\nFault latch: three clearing classes");
    fault_state_t f;
    fault_init(&f);
    chk_true("starts clear", !fault_any(&f));

    chk_true("raise IMU_DISAGREE reports newly raised", fault_raise(&f, FAULT_IMU_DISAGREE));
    chk_true("raising it again reports not new", !fault_raise(&f, FAULT_IMU_DISAGREE));
    fault_clear_on_disarm(&f);
    chk_true("FOREVER fault survives clear_on_disarm", f.active & FAULT_IMU_DISAGREE);

    fault_raise(&f, FAULT_TILT | FAULT_LOOP_OVERRUN);
    fault_clear_on_disarm(&f);
    chk_true("UNTIL-DISARM faults cleared by clear_on_disarm",
             !(f.active & (FAULT_TILT | FAULT_LOOP_OVERRUN)));
    chk_true("  ... but remembered in 'ever'", f.ever & FAULT_TILT);

    fault_set_condition(&f, FAULT_RC_LINK, true);
    chk_true("LIVE fault set while present", f.active & FAULT_RC_LINK);
    fault_set_condition(&f, FAULT_RC_LINK, false);
    chk_true("LIVE fault cleared when gone", !(f.active & FAULT_RC_LINK));
    fault_set_condition(&f, FAULT_IMU_DISAGREE, false);
    chk_true("set_condition cannot clear a FOREVER fault", f.active & FAULT_IMU_DISAGREE);

    chk_true("most severe: IMU_DISAGREE beats RC_LINK and TILT",
             fault_most_severe(FAULT_RC_LINK | FAULT_TILT | FAULT_IMU_DISAGREE) == FAULT_IMU_DISAGREE);
    chk_true("most severe: TILT beats RC_LINK",
             fault_most_severe(FAULT_RC_LINK | FAULT_TILT) == FAULT_TILT);

    char buf[64];
    fault_describe(FAULT_RC_LINK | FAULT_IMU_COMM, buf, sizeof buf);
    chk_str("describe lists most severe first", buf, "IMU_COMM|RC_LINK");
    fault_describe(0u, buf, sizeof buf);
    chk_str("describe of nothing", buf, "none");
    char tiny[6];
    fault_describe(FAULT_IMU_COMM | FAULT_RC_LINK, tiny, sizeof tiny);
    chk_true("describe truncates safely into a tiny buffer", strlen(tiny) < sizeof tiny);
    chk_true("unknown bits are ignored by raise", !fault_raise(&f, 1u << 30));
}

/* ------------------------------------------------------------------ buzzer */
static void case_patterns(void)
{
    puts("\nBuzzer patterns");
    chk_true("boot-ok on at 0 ms",    buzz_pattern_level(BUZZ_BOOT_OK, 0));
    chk_true("boot-ok on at 99 ms",   buzz_pattern_level(BUZZ_BOOT_OK, 99));
    chk_true("boot-ok off at 100 ms", !buzz_pattern_level(BUZZ_BOOT_OK, 100));
    chk_true("boot-ok done at 100 ms", buzz_pattern_done(BUZZ_BOOT_OK, 100));

    chk_true("rc-link: chirp 1 at 25 ms",   buzz_pattern_level(BUZZ_FAULT_RC_LINK, 25));
    chk_true("rc-link: gap at 75 ms",      !buzz_pattern_level(BUZZ_FAULT_RC_LINK, 75));
    chk_true("rc-link: chirp 2 at 125 ms",  buzz_pattern_level(BUZZ_FAULT_RC_LINK, 125));
    chk_true("rc-link: silence at 300 ms", !buzz_pattern_level(BUZZ_FAULT_RC_LINK, 300));
    chk_true("rc-link: repeats at 525 ms",  buzz_pattern_level(BUZZ_FAULT_RC_LINK, 525));
    chk_true("rc-link never 'done'",       !buzz_pattern_done(BUZZ_FAULT_RC_LINK, 100000));

    chk_true("fault-imu is continuous",
             buzz_pattern_level(BUZZ_FAULT_IMU, 0) && buzz_pattern_level(BUZZ_FAULT_IMU, 12345));
    chk_true("disarmed: one 400 ms tone",
             buzz_pattern_level(BUZZ_DISARMED, 399) && !buzz_pattern_level(BUZZ_DISARMED, 400));
    chk_true("silent is silent", !buzz_pattern_level(BUZZ_SILENT, 0));
    chk_true("out-of-range pattern is silent", !buzz_pattern_level((buzz_pattern_t)99, 0));

    /* All patterns are distinguishable: no two share bits+length+repeat. */
    int distinct = 1;
    for (int a = 1; a < BUZZ_PATTERN_COUNT; ++a) {
        for (int b = a + 1; b < BUZZ_PATTERN_COUNT; ++b) {
            int same = buzz_pattern_repeats((buzz_pattern_t)a) == buzz_pattern_repeats((buzz_pattern_t)b)
                    && buzz_pattern_length_ms((buzz_pattern_t)a) == buzz_pattern_length_ms((buzz_pattern_t)b);
            for (uint32_t t = 0; same && t < buzz_pattern_length_ms((buzz_pattern_t)a); t += BUZZ_SLOT_MS) {
                same = buzz_pattern_level((buzz_pattern_t)a, t) == buzz_pattern_level((buzz_pattern_t)b, t);
            }
            if (same) distinct = 0;
        }
    }
    chk_true("every audible pattern is distinguishable from every other", distinct);

    puts("\nFault -> pattern priority");
    chk_true("IMU fault outranks RC link",
             buzz_pattern_for_faults(FAULT_RC_LINK | FAULT_IMU_COMM) == BUZZ_FAULT_IMU);
    chk_true("config fault outranks tilt",
             buzz_pattern_for_faults(FAULT_TILT | FAULT_PARAMS) == BUZZ_FAULT_CONFIG);
    chk_true("tilt outranks overrun and link",
             buzz_pattern_for_faults(FAULT_TILT | FAULT_LOOP_OVERRUN | FAULT_RC_LINK) == BUZZ_FAULT_TILT);
    chk_true("RC link alone -> double chirp", buzz_pattern_for_faults(FAULT_RC_LINK) == BUZZ_FAULT_RC_LINK);
    chk_true("no faults -> silent", buzz_pattern_for_faults(0u) == BUZZ_SILENT);

    puts("\nPlayer: events play over faults, then the fault pattern resumes");
    buzz_player_t pl;
    buzz_player_init(&pl);
    chk_true("idle player is silent", !buzz_player_level(&pl, 0u, 1000));
    chk_true("RC fault at t=1000 starts its pattern (on)", buzz_player_level(&pl, FAULT_RC_LINK, 1000));
    chk_true("  ... gap 75 ms later", !buzz_player_level(&pl, FAULT_RC_LINK, 1075));
    buzz_player_event(&pl, BUZZ_DISARMED, 1100);
    chk_true("disarmed event overrides: on at 1300 (fault would be silent)",
             buzz_player_level(&pl, FAULT_RC_LINK, 1300));
    chk_true("event still playing at 1499", buzz_player_level(&pl, FAULT_RC_LINK, 1499));
    chk_true("event over at 1500: fault pattern restarts from its start (on)",
             buzz_player_level(&pl, FAULT_RC_LINK, 1500));
    chk_true("  ... and its gap follows 75 ms later", !buzz_player_level(&pl, FAULT_RC_LINK, 1575));
    chk_true("fault changes to IMU: continuous immediately",
             buzz_player_level(&pl, FAULT_IMU_DISAGREE, 1580) &&
             buzz_player_level(&pl, FAULT_IMU_DISAGREE, 1660));
    buzz_player_event(&pl, BUZZ_CAL_HOLD_STILL, 2000);
    buzz_player_event(&pl, BUZZ_SILENT, 2010);
    chk_true("a cancelled repeating event hands back to faults",
             buzz_player_level(&pl, FAULT_IMU_DISAGREE, 2020));
}

/* -------------------------------------------------------------- supervisor */
static const sup_config_t CFG = {
    .settle_ms = 400u, .upright_min_rad = 1.0471976f /* 60 */,
    .flat_max_tilt_rad = 0.5235988f /* 30 */, .balance_gains_set = true,
};

static sup_inputs_t good(uint32_t now)
{
    sup_inputs_t in = {
        .now_ms = now, .link_ok = true, .arm_switch = false, .sticks_centered = true,
        .mode_request = SUP_MODE_FLAT, .imu_ready = true,
        .flat_roll_rad = 0.0f, .flat_pitch_rad = 0.0f, .active_faults = 0u,
    };
    return in;
}

static void step(supervisor_t *s, const sup_config_t *c, sup_inputs_t *in, uint32_t now)
{
    in->now_ms = now;
    supervisor_update(s, c, in);
}

static void case_supervisor(void)
{
    puts("\nSupervisor: arming requires a deliberate LOW->HIGH edge");
    supervisor_t s;
    supervisor_init(&s);
    sup_inputs_t in = good(0);

    in.arm_switch = true;
    step(&s, &CFG, &in, 10);
    chk_true("switch HIGH at power-on does NOT arm", s.state == SUP_DISARMED && !s.ev_rejected);
    step(&s, &CFG, &in, 20);
    chk_true("  ... still not armed while it stays HIGH", s.state == SUP_DISARMED);

    in.arm_switch = false; step(&s, &CFG, &in, 30);
    in.arm_switch = true;  step(&s, &CFG, &in, 40);
    chk_true("LOW then HIGH with everything OK -> ARMED-FLAT", s.state == SUP_ARMED_FLAT && s.ev_armed);
    chk_true("  motors live in ARMED-FLAT", supervisor_motors_live(&s));
    chk_true("  flat frame in ARMED-FLAT", !supervisor_wants_balance_frame(&s));
    step(&s, &CFG, &in, 50);
    chk_true("  armed event lasts exactly one update", !s.ev_armed && s.state == SUP_ARMED_FLAT);

    in.arm_switch = false; step(&s, &CFG, &in, 60);
    chk_true("switch LOW -> DISARMED with event", s.state == SUP_DISARMED && s.ev_disarmed);
    chk_true("  motors not live", !supervisor_motors_live(&s));

    puts("\nSupervisor: rejections consume the edge");
    in.sticks_centered = false;
    in.arm_switch = true; step(&s, &CFG, &in, 70);
    chk_true("sticks not centred -> rejected", s.state == SUP_DISARMED && s.ev_rejected &&
             s.last_result == SUP_REJECT_STICKS_NOT_CENTRED);
    in.sticks_centered = true; step(&s, &CFG, &in, 80);
    chk_true("centring sticks with switch still HIGH does NOT arm", s.state == SUP_DISARMED);
    in.arm_switch = false; step(&s, &CFG, &in, 90);
    in.arm_switch = true;  step(&s, &CFG, &in, 100);
    chk_true("cycling the switch then arms", s.state == SUP_ARMED_FLAT);

    puts("\nSupervisor: disarm conditions, and no self re-arm");
    in.link_ok = false; step(&s, &CFG, &in, 110);
    chk_true("link lost while armed -> DISARMED", s.state == SUP_DISARMED && s.ev_disarmed);
    in.link_ok = true;  step(&s, &CFG, &in, 120);
    chk_true("link back with switch still HIGH -> stays DISARMED", s.state == SUP_DISARMED);
    in.arm_switch = false; step(&s, &CFG, &in, 130);
    in.arm_switch = true;  step(&s, &CFG, &in, 140);
    chk_true("re-arm needs the switch cycled", s.state == SUP_ARMED_FLAT);

    in.active_faults = FAULT_TILT; step(&s, &CFG, &in, 150);
    chk_true("fault while armed -> DISARMED", s.state == SUP_DISARMED);
    in.active_faults = 0u; step(&s, &CFG, &in, 160);
    chk_true("fault cleared with switch HIGH -> stays DISARMED", s.state == SUP_DISARMED);

    in.arm_switch = false; step(&s, &CFG, &in, 170);
    in.arm_switch = true; step(&s, &CFG, &in, 180);
    in.imu_ready = false; step(&s, &CFG, &in, 190);
    chk_true("IMU invalid while armed -> DISARMED", s.state == SUP_DISARMED);

    puts("\nSupervisor: each arm check rejects on its own");
    typedef struct { const char *name; sup_arm_result_t want; } rej_t;
    for (int k = 0; k < 3; ++k) {
        supervisor_init(&s);
        in = good(0);
        rej_t r;
        if (k == 0) { in.active_faults = FAULT_IMU_COMM; r = (rej_t){ "active fault -> REJECT_FAULT", SUP_REJECT_FAULT }; }
        if (k == 1) { in.link_ok = false;               r = (rej_t){ "no link -> REJECT_NO_LINK", SUP_REJECT_NO_LINK }; }
        if (k == 2) { in.imu_ready = false;             r = (rej_t){ "IMU not ready -> REJECT_IMU_NOT_READY", SUP_REJECT_IMU_NOT_READY }; }
        step(&s, &CFG, &in, 1);
        in.arm_switch = true; step(&s, &CFG, &in, 2);
        chk_true(r.name, s.state == SUP_DISARMED && s.last_result == r.want);
    }

    supervisor_init(&s); in = good(0);
    in.flat_pitch_rad = RAD(40.0f);
    step(&s, &CFG, &in, 1); in.arm_switch = true; step(&s, &CFG, &in, 2);
    chk_true("flat requested while tipped 40 deg -> REJECT_NOT_FLAT",
             s.last_result == SUP_REJECT_NOT_FLAT && s.state == SUP_DISARMED);

    puts("\nSupervisor: balance mode");
    supervisor_init(&s); in = good(0);
    in.mode_request = SUP_MODE_BALANCE;
    step(&s, &CFG, &in, 1); in.arm_switch = true; step(&s, &CFG, &in, 2);
    chk_true("balance requested while flat -> REJECT_NOT_UPRIGHT",
             s.last_result == SUP_REJECT_NOT_UPRIGHT);

    supervisor_init(&s); in = good(0);
    in.mode_request = SUP_MODE_BALANCE; in.flat_pitch_rad = RAD(80.0f);
    sup_config_t nogain = CFG; nogain.balance_gains_set = false;
    step(&s, &nogain, &in, 1); in.arm_switch = true; step(&s, &nogain, &in, 2);
    chk_true("balance with zero gains -> REJECT_NO_BALANCE_GAINS",
             s.last_result == SUP_REJECT_NO_BALANCE_GAINS && s.state == SUP_DISARMED);

    supervisor_init(&s); in = good(0);
    in.mode_request = SUP_MODE_BALANCE; in.flat_pitch_rad = RAD(85.0f);
    step(&s, &CFG, &in, 1000); in.arm_switch = true; step(&s, &CFG, &in, 1010);
    chk_true("upright nose-down (+85) -> SETTLE on FRONT pair",
             s.state == SUP_ARMED_BALANCE_SETTLE && s.pair == OMNIS_BALANCE_ON_FRONT_PAIR);
    chk_true("  motors NOT live during settle", !supervisor_motors_live(&s));
    chk_true("  but already in the balance frame", supervisor_wants_balance_frame(&s));
    step(&s, &CFG, &in, 1409);
    chk_true("  still settling at 399 ms", s.state == SUP_ARMED_BALANCE_SETTLE);
    step(&s, &CFG, &in, 1410);
    chk_true("  -> ARMED-BALANCE at 400 ms, with engaged event",
             s.state == SUP_ARMED_BALANCE && s.ev_balance_engaged);
    chk_true("  motors live", supervisor_motors_live(&s));
    in.mode_request = SUP_MODE_FLAT; step(&s, &CFG, &in, 1420);
    chk_true("mode switch moved while armed -> ignored (mode latched)",
             s.state == SUP_ARMED_BALANCE);

    supervisor_init(&s); in = good(0);
    in.mode_request = SUP_MODE_BALANCE; in.flat_pitch_rad = RAD(-85.0f);
    step(&s, &CFG, &in, 1); in.arm_switch = true; step(&s, &CFG, &in, 2);
    chk_true("upright nose-up (-85) -> REAR pair", s.pair == OMNIS_BALANCE_ON_REAR_PAIR);

    puts("\nSupervisor: AUTO chooses from the pose");
    supervisor_init(&s); in = good(0); in.mode_request = SUP_MODE_AUTO;
    step(&s, &CFG, &in, 1); in.arm_switch = true; step(&s, &CFG, &in, 2);
    chk_true("AUTO while flat -> ARMED-FLAT", s.state == SUP_ARMED_FLAT);
    supervisor_init(&s); in = good(0); in.mode_request = SUP_MODE_AUTO; in.flat_pitch_rad = RAD(75.0f);
    step(&s, &CFG, &in, 1); in.arm_switch = true; step(&s, &CFG, &in, 2);
    chk_true("AUTO while tipped 75 -> balance settle", s.state == SUP_ARMED_BALANCE_SETTLE);

    chk_str("state name", sup_state_name(SUP_ARMED_BALANCE), "ARMED-BALANCE");
    chk_str("reject reason is human-readable", sup_result_name(SUP_REJECT_NO_LINK), "no radio link");
}

int main(void)
{
    puts("OMNIS fault / buzzer / supervisor verification");
    case_faults();
    case_patterns();
    case_supervisor();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
