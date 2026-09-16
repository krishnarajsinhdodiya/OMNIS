# OMNIS Superloop MVP — Developer & Operations Guide

Everything needed to build the firmware, understand how it runs, find the file
that owns a behaviour, and change the numbers that usually need changing.

**Target:** ESP32-S3 (EdgeHax S3 Pro / DevKitC-1 class), board **Rev 2.0** ·
ESP-IDF **v6.0.2** · no FreeRTOS primitives in OMNIS-authored code.

| Companion document | Read it for |
|---|---|
| **[TESTING.md](TESTING.md)** | Bench bring-up, stage by stage. **Read §0 before applying 12 V.** |
| [README.md](README.md) | One-page overview and current status |
| [BUILD-LOG.md](BUILD-LOG.md) | Scope, the settled decisions, the stage-by-stage commit table, and the full construction history with every bug found |
| [../assets/documentation/omnis-info.md](../assets/documentation/omnis-info.md) | The project info file — the full spec, with per-section notes on what this build does |
| [../assets/kinematics/](../assets/kinematics/) | The drivetrain maths, derived from first principles |
| [../assets/control/](../assets/control/) | EKF, fusion and PID derivations |

> ### Safety, once, at the top
> Board Rev 2.0 **hardwires the A4988 `EN#` line**. The motors are energised
> whenever 12 V is present — before the firmware has run, and while it is
> halted in a fault. Nothing in software can de-energise them; only unplugging
> can. **Set the four A4988 `Vref` current limits before the first power-up**
> (TESTING.md §0), and keep the wheels off the ground for everything up to and
> including Stage 5.

---

## 1. Build and flash

### 1.1 One-time environment

ESP-IDF v6.0.2 is installed natively. Activate it with the **tools activation
script**, not `export.sh`:

```bash
. ~/.espressif/tools/activate_idf_v6.0.2.sh
```

> `export.sh` looks for a Python 3.11 virtualenv while this install runs on
> 3.14, so it fails where the activation script works. This is the reason the
> command looks unusual; it is not a typo.

Confirm the toolchain is live:

```bash
idf.py --version && xtensa-esp32s3-elf-gcc --version | head -1
```

### 1.2 Build

The project is a sub-directory of the repository, so build it with `-C` from
the repository root — or `cd` into it first; both work.

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp build
```

A clean build ends with the partition report. Expect roughly:

```
omnis_superloop_mvp.bin binary size 0x3bda0 bytes.
Smallest app partition is 0x100000 bytes. 0xc4260 bytes (77%) free.
```

The build must be **warning-free**. Warnings are not tolerated here because the
whole safety argument rests on the compiler having nothing to say.

### 1.3 Flash and monitor

Find the port first — the ESP32-S3's native USB enumerates as `usbmodem`:

```bash
ls /dev/cu.usbmodem* /dev/cu.SLAB* /dev/cu.wchusb* 2>/dev/null
```

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp -p /dev/cu.usbmodem101 flash monitor
```

Leave the monitor attached: the boot sequence prints the self-checks that tell
you whether the board is healthy, and the once-a-second telemetry line is the
main diagnostic instrument for everything in TESTING.md.

Exit the monitor with **Ctrl-]**.

If flashing fails to connect, hold **BOOT**, tap **RESET**, release **BOOT**,
and retry.

### 1.4 Host tests — no hardware needed

Every module with no ESP-IDF dependency is tested on the Mac with plain `cc`.
Run these before every commit and after every parameter change:

```bash
cd /Volumes/Projects/OMNIS/omnis-superloop-mvp && ./test/run_host_tests.sh
```

Expect `ALL 7 HOST TEST SUITES PASSED`. The runner exits non-zero if any suite
fails, so a single failure cannot hide behind later passes.

### 1.5 Useful variations

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp fullclean build
```

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp -p /dev/cu.usbmodem101 monitor
```

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp size-components
```

---

## 2. Superloop architecture and workflow

### 2.1 The rule that shapes everything

**No FreeRTOS primitive appears in OMNIS-authored code.** No `xTaskCreate`, no
queues, semaphores, mutexes, event groups or software timers. ESP-IDF driver
internals are treated as hardware: if the I²C or RMT driver uses an RTOS object
inside itself, that is the driver's business.

The rule is verified mechanically rather than trusted: neither the `main/`
sources nor the undefined symbols of the compiled `libmain.a` contain any
FreeRTOS task, queue, semaphore, event-group, timer or port symbol.

The consequence is that **the entire application is one `for (;;)` loop**, and
ordering is explicit rather than emergent. There is no scheduler to blame and
no priority inversion to debug; the cost is that every slow operation must be
made non-blocking by hand.

### 2.2 Timing

```
GPTimer, 1 MHz, alarm every 2000 counts
        │
        └─► ISR: g_tick_500hz = true;        ← that is the entire ISR
                 (plus a missed-tick counter)
        │
   tick_wait()  spins until the flag is set, clears it, returns the
                microseconds of headroom that were left over
```

500 Hz, so **2000 µs per tick**. The ISR does nothing but set a `volatile bool`
— no I²C, no floating point, no logging. All real work happens in the loop body
when it sees the flag.

`tick_wait()` returns the **headroom**: how long the loop was idle before the
tick arrived. That single number is the loop's health indicator. It is reported
once a second, along with the count of ticks that were missed entirely
(*overruns*). On the Stage 1 hardware run the loop used ~2 µs of the 2000 µs
budget with zero overruns.

The main task is pinned to **CPU1** (`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1=y`)
and CPU1's idle-task watchdog is disabled — a superloop never yields, so the
idle task never runs, and the watchdog would otherwise fire immediately. CPU0
keeps running the Wi-Fi/BT stack idle work and the IDF's own housekeeping.

### 2.3 The loop body

```
   app_main
     ├── gpio_safe_state()        STEP low FIRST — the drivers are already live
     ├── params defaults + validate
     ├── kinematics_selftest()    the verified cases, re-run on the FPU
     ├── peripheral bring-up      I2C, both MPU6050s, UART1/CRSF, RMT, buzzer
     ├── boot IMU calibration     holds the loop; radio and buzzer still serviced
     └── for (;;)
          │
          │  tick_wait()  ────────────────────────── blocks here until the tick
          │
          ├── 1. SENSE
          │      imu_update()      2× MPU6050 over I²C  →  mount correction
          │                        →  gyro bias  →  per-IMU EKF  →  fusion
          │      crsf_poll()       UART1 RX  →  CRSF parser  →  16 channels
          │                        →  rc_input  →  sticks, switches, arm request
          │
          ├── 2. JUDGE
          │      fault latch       IMU comm · IMU disagreement >15° · link loss
          │                        · tilt · loop overrun
          │      supervisor        DISARMED / ARMED-FLAT / BALANCE-SETTLE /
          │                        ARMED-BALANCE, and which wheel pair
          │
          ├── 3. ACT
          │      flat mode:     sticks → IK → common clamp → deadband → slew
          │      balance mode:  lean PID in the balance frame → one wheel pair
          │      otherwise:     step_gen_stop()
          │                        ↓
          │      step_gen_update()   queues ~2 ms RMT bursts per wheel;
          │                          the RMT driver's ISR chains them
          │
          └── 4. REPORT
                 buzzer_service()   every tick
                 telemetry          one line per second
```

**Why this order.** Sense before judge, judge before act: a fault raised this
tick stops the motors this tick, not next. Reporting last so the telemetry line
describes the decision that was actually taken. The blocking UART cost of the
telemetry line is excluded from the timing statistics and from the overrun
fault, so printing can never fault the loop it is measuring.

**I²C never runs in an ISR.** The IMU reads are ordinary blocking transactions
in the loop body, which is why the 2000 µs budget matters and why
`OMNIS_IMU_READ_ALTERNATE` exists as an escape hatch (read one IMU per tick,
each filter then running at 250 Hz on its own measured `dt`).

### 2.4 How STEP pulses actually get out

This is the part that would normally want a task, and does not have one.

Each wheel owns one of the ESP32-S3's four RMT TX channels. Every tick,
`step_gen_update()` tops up that channel's queue with short bursts (~2 ms) of
STEP pulses generated at the wheel's current rate; the RMT driver's own ISR
chains them back-to-back. **Nothing of ours runs in the pulse hot path.**

- **Queue regulation is exact, not estimated.** Pending chunks = submitted in
  the loop minus completed in the RMT done-callback. A late tick is absorbed by
  the ~6 ms lead; the next tick refills it.
- **Direction reversals drain first.** Chunks already queued were generated for
  the old direction, so a reversal stops queueing, lets the queue empty, sets
  `DIR`, waits a full tick, then resumes. The deadband has already held the
  wheel near zero by then, so the few-millisecond pause is invisible.
- **Stopping means "no more pulses"** — Rev 2.0 has no enable line.
  `step_gen_stop()` neutralises the level bits of every queued chunk in place;
  the copy encoder reads payloads lazily, so unencoded chunks play as silence.
  At most one chunk already inside RMT memory still plays. (`rmt_tx_disable()`
  was rejected: it recycles only the in-flight transaction, with no done
  callback, and leaves the rest queued to play on the next enable.)

### 2.5 Failsafe behaviour

Because `EN#` is hardwired, "safe" means **stopped and holding**, not limp:

| Condition | Result |
|---|---|
| Radio link lost | STEP pulses cease; motors hold position; buzzer sounds |
| IMU comm failure or >15° disagreement | Fault latched, disarmed, pulses cease |
| Tilt past the mode's limit | Fault latched, disarmed |
| Loop overruns past the window threshold | Fault latched while armed |
| Parameters or tick invalid at boot | Never arms; `fault_only_loop()` explains itself every 2 s |

Re-arming always requires the arm switch to be **cycled LOW→HIGH**. The
supervisor consumes the edge, so a fault that clears while the switch is still
up does not re-arm the robot on its own.

---

## 3. File index

Every file in `omnis-superloop-mvp/`, and what it is responsible for.

### 3.1 Project root

| File | Responsibility |
|---|---|
| `CMakeLists.txt` | Top-level ESP-IDF project definition |
| `sdkconfig.defaults` | The configuration that matters, in version control: CPU1 affinity, idle-WDT disable, FPU, log level. Delete `sdkconfig` to regenerate from it |
| `sdkconfig` | Generated. Not the source of truth |
| `README.md` | One-page overview and stage status |
| `OPERATIONS_GUIDE.md` | This file |
| `TESTING.md` | Bench bring-up procedure, stage by stage |
| `BUILD-LOG.md` | Construction history: what was built, what broke, what was learned. Also the scope, the settled decisions and the commit table, absorbed from `PLAN.md` |

The project info file is **not** duplicated here — there is one copy for the
whole project at [`../assets/documentation/omnis-info.md`](../assets/documentation/omnis-info.md),
carrying the full specification with `> **Superloop MVP:**` notes on what this
build actually does.

### 3.2 `main/` — configuration and support

| File | Responsibility |
|---|---|
| `CMakeLists.txt` | Source list and component requirements |
| `main.c` | Boot sequence, safe GPIO state, self-checks, the superloop itself, telemetry, and the bench pattern |
| `omnis_pins.h` | Board **Rev 2.0** GPIO map, traced from the KiCad schematic wire-by-wire rather than read off a drawing |
| `omnis_config.h` | Build-time switches (§5.1). Edit, rebuild, reflash |
| `omnis_params.{c,h}` | Every runtime-tunable number in one struct, with defaults and a validator. Stands in for the future `params.json` |
| `omnis_time.h` | Time helpers for a build with no RTOS delay primitives |

### 3.3 `main/` — timing

| File | Responsibility |
|---|---|
| `tick.{c,h}` | The 500 Hz GPTimer, its ISR (sets one flag), `tick_wait()`, headroom and overrun accounting |

### 3.4 `main/` — sensing and estimation

| File | Responsibility |
|---|---|
| `mpu6050.{c,h}` | Register-level MPU6050 driver on the ESP-IDF v6 `i2c_master` API |
| `mpu6050_regs.h` | Register map, configuration values, and the **pure** burst parser (host-tested) |
| `imu.{c,h}` | Both IMUs end to end: I²C bring-up, boot gyro-bias calibration with stillness gating, per-tick read, mount correction, frames, and the mount wizard |
| `omnis_imu_mounting.h` | As-built mounting descriptors and corner labelling. The two modules are **antiparallel**; uncorrected they read ~20° apart and trip the disagreement fault at every boot |
| `attitude_ekf.{c,h}` | Per-IMU 4-state attitude EKF. Byte-identical to `assets/control/` |
| `imu_fusion.{c,h}` | Inverse-covariance fusion of the two estimates, the gravity-vector disagreement metric, and down-side detection. Byte-identical to `assets/control/` |

### 3.5 `main/` — radio

| File | Responsibility |
|---|---|
| `crsf_parser.{c,h}` | **Pure** CRSF byte-stream parser: framing, CRC-8/DVB-S2, 16×11-bit channel unpacking. No IDF, fully host-tested against golden frames |
| `crsf.{c,h}` | UART1 glue: RX-only, non-blocking, 420000 baud, with RX-pin auto-detect |
| `rc_input.{c,h}` | Channels → sticks, switches and the arm request. Deadzone, centring checks, link-quality floor |

### 3.6 `main/` — drivetrain

| File | Responsibility |
|---|---|
| `mecanum_kinematics.{c,h}` | Inverse and forward kinematics for the **lateral parallel roller** layout, the roller-handedness constants, clamp, deadband and the slip metric. Byte-identical to `assets/kinematics/` |
| `drive.{c,h}` | Sticks → body velocity → wheel rates, in the one correct order, plus the acceleration slew limiter. Carries the ordering rationale in its header |
| `step_wave.{c,h}` | **Pure** STEP waveform generator: rate in, RMT-style symbols out, with proven guarantees (pulses never split, every chunk ends LOW, rounding residue carried) |
| `step_gen.{c,h}` | RMT glue for four wheels: burst re-arm, exact queue regulation, direction-reversal drain, and stop-by-neutralising |

### 3.7 `main/` — control and safety

| File | Responsibility |
|---|---|
| `pid.{c,h}` | Discrete PID with derivative-on-measurement and conditional-integration anti-windup, plus the outer velocity-bias loop. Byte-identical to `assets/control/` |
| `balance.{c,h}` | Two-wheel balance controller. Lean in, wheel rates out. Its header carries the per-pair wheel-sign derivation |
| `fault.{c,h}` | The fault latch: what is wrong, how long it stays wrong, and which of the three clearing classes it belongs to |
| `buzzer_pattern.{c,h}` | **Pure** buzzer patterns and the priority rules that choose between them |
| `buzzer.{c,h}` | Buzzer GPIO over the pure pattern player |
| `supervisor.{c,h}` | The arming state machine: when the motors may move, in which mode, on which wheel pair, and why an arm request was refused |

### 3.8 `test/` — host tests, no hardware

| File | Covers |
|---|---|
| `run_host_tests.sh` | Runs every suite; non-zero exit if any fails |
| `test_kinematics.c` | IK/FK against the verified reference numbers, the layout's own properties (holonomy per axis, lever ratio, column non-orthogonality, null space), clamp, deadband, RC mapping, the full `drive_solve` pipeline and the slew limiter |
| `test_control.c` | EKF, fusion and PID |
| `test_crsf.c` | Parser and RC input, against golden frames |
| `test_mpu6050.c` | Burst decoder |
| `test_step_wave.c` | The waveform guarantees, on a reconstructed timeline |
| `test_supervisor.c` | Fault latch, buzzer patterns, arming state machine |
| `test_balance.c` | Balance signs, limits, and a closed-loop inverted-pendulum simulation on both pairs |
| `crsf_golden.h` | Reference frames from an independent implementation |

---

## 4. Tuning cheat sheet

Everything here is a **parameter**, not a code change. Edit
`main/omnis_params.c` (defaults) or `main/omnis_config.h` (build switches),
rebuild, reflash.

> Run `./test/run_host_tests.sh` after any parameter edit. Several suites assert
> on the defaults and will tell you immediately if a change contradicts them.

### 4.1 A wheel turns the wrong way — `motor.dir_invert[]`

```c
p->motor.dir_invert[0] = false;   /* FL */
p->motor.dir_invert[1] = false;   /* FR */
p->motor.dir_invert[2] = false;   /* RL */
p->motor.dir_invert[3] = false;   /* RR */
```

The left and right motors are mirror images across the chassis, so the same
*wheel* direction needs opposite shaft rotation on each side. Fix it **here**.

> **Never fix a backwards wheel by flipping a sign in the kinematics.** It will
> appear to work for that one motion and will silently break the IK→FK round
> trip and every other motion involving that wheel.

**Procedure:** command plain FORWARD first (TESTING.md §5.1) and set the flags
until all four wheels drive the robot forward. Only then look at strafe or yaw.

### 4.2 The robot strafes the wrong way — roller handedness

If **forward is correct on all four wheels** but strafe goes the wrong way, the
problem is not a `dir_invert` flag; it is the roller handedness. In
`main/mecanum_kinematics.h`:

```c
#define MECANUM_DELTA_FL   (-1)
#define MECANUM_DELTA_FR   (+1)
#define MECANUM_DELTA_RL   (-1)
#define MECANUM_DELTA_RR   (+1)
```

**Negate all four together** and rebuild. That is the complete fix — both `vy`
signs and both yaw lever arms are computed from these, and the forward
kinematics follows automatically.

**The test that tells you which it is** — wheels off the ground, both
directions:

| Command | Correct handedness | Mirrored handedness |
|---|---|---|
| `vx = vy` (forward-left diagonal) | **FL and RL** completely still | FR and RR still instead |
| `vx = −vy` (forward-right diagonal) | **FR and RR** completely still | FL and RL still instead |

If a *diagonal* pair idles instead of a side, the wheels are not in the
parallel layout at all and the derivation does not describe your robot. If
neither pattern appears, go back to §4.1 — forward motion is not right yet.

The bench pattern (`OMNIS_BENCH_STEP_TEST 1`) runs both diagonals for you.

### 4.3 Chassis dimensions — `geometry`

```c
p->geometry.wheel_radius_mm = 30.0f;
p->geometry.wheelbase_mm    = 223.0f;   /* L: front-to-back wheel CENTRES */
p->geometry.track_width_mm  = 230.0f;   /* W: left-to-right wheel CENTRES */
```

Measure wheel **centre to centre**, not the frame. No numeric geometry constant
appears anywhere in the kinematics, so these three numbers are the whole story.

What each one moves:

| Change | Effect |
|---|---|
| `wheel_radius_mm` | Scales every wheel rate inversely. A wrong radius makes all speeds wrong by the same ratio — easy to miss, since the robot still moves correctly, just faster or slower than commanded |
| `wheelbase_mm` (L) | Front yaw lever `(L+W)/2`, rear yaw lever `(L−W)/2`, **and** the forward-kinematics yaw scale, which depends on `L` alone |
| `track_width_mm` (W) | Both lever arms, and the strafe row of the forward kinematics |

**Two warnings specific to this drivetrain:**

- The rear yaw lever is `(L−W)/2` = **−3.5 mm** as built. It is 1.5% of the
  front lever, it is **zero on a square chassis**, and it **changes sign** if
  the robot is ever made longer than it is wide. Nothing breaks — but the rear
  wheels' direction during a spin is not a stable fact to rely on.
- The firmware's boot self-check runs on **fixed reference geometry**
  (30/223/230), deliberately, so that editing these values cannot mask a
  genuine kinematics fault.

### 4.4 Speed and yaw rate — `rc_scale`

```c
p->rc_scale.vx_max_mmps       = 300.0f;   /* full throttle            */
p->rc_scale.vx_secondary_mmps = 150.0f;   /* pitch stick, additive    */
p->rc_scale.vy_max_mmps       = 300.0f;   /* full roll, + = LEFT      */
p->rc_scale.w_max_radps       = 1.5f;     /* full yaw,  + = CCW       */
```

These are derived, not guessed. With `max_step_rate = 7000`:

```
pure vx :  f = vx · 509.2958 / r = 16.98 · vx      →  ceiling 412 mm/s
pure vy :  same cost as vx                         →  ceiling 412 mm/s
pure w  :  f = k · w · 509.2958 / r = 3845 · w     →  ceiling 1.82 rad/s
```

The defaults sit ~27% below each ceiling so that *combined* commands have
headroom before the clamp bites. Raising one of these does not make the robot
faster past the clamp — it just makes the common-factor clamp engage sooner and
scale the whole motion down.

**Stick polarity** belongs here and nowhere else:

```c
p->rc_scale.invert_throttle = false;
p->rc_scale.invert_pitch    = false;
p->rc_scale.invert_roll     = false;   /* set if stick-LEFT reads negative */
p->rc_scale.invert_yaw      = false;   /* set if stick-LEFT reads negative */
```

### 4.5 Stepper limits — `step`

```c
p->step.max_step_rate      = 7000.0f;    /* per-wheel cap [µsteps/s]  */
p->step.deadband_steps     = 20.0f;      /* below this, hold stopped  */
p->step.max_accel_steps_s2 = 20000.0f;   /* slew limit                */
```

| Parameter | Raise it if | Lower it if |
|---|---|---|
| `max_step_rate` | You need more top speed **and** the motors are not skipping | Wheels stall or buzz at speed. 7000 µsteps/s ≈ 131 RPM, already well off a NEMA17's 12 V torque curve |
| `deadband_steps` | Wheels chatter or buzz when nearly stopped | A genuine crawl command is being swallowed |
| `max_accel_steps_s2` | Response feels sluggish | The robot loses steps when the stick snaps — the **only** defence against that, since there are no encoders |

`max_accel_steps_s2` is a deliberately conservative placeholder. The right value
depends on the `Vref` current limits, which are set on the bench.

> **Expected, not a fault:** during a slow yaw command the **rear pair sits
> still** while the front pair drives. Their yaw lever is 1.5% of the front's,
> so below about **0.34 rad/s** they fall under the deadband. Raising
> `deadband_steps` makes this happen at higher yaw rates.

### 4.6 STEP electrical timing — `motor`

```c
p->motor.step_pulse_us    = 10;   /* STEP high time; A4988 minimum is 1 µs */
p->motor.step_min_low_us  = 10;   /* floor on the low time between pulses  */
```

10 µs each is generous and deliberately so — it is robust to wiring capacitance
on a hand-built board. Both feed `step_wave.c`, which enforces the guarantees
that pulses are never split and every chunk ends LOW. `step_gen_init()` refuses
a `max_step_rate` above what the symbol buffer can express at these timings, so
an inconsistent pair fails loudly at boot rather than producing short chunks.

### 4.7 Balance gains — `balance`

Gains are **zero by default**, and the supervisor refuses to arm balance mode
until `kp > 0`. That is intentional: a zero-gain balancer simply falls over.

```c
p->balance.kp                = 0.0f;        /* start here */
p->balance.ki                = 0.0f;
p->balance.kd                = 0.0f;
p->balance.integral_max      = 0.0f;
p->balance.trim_rad          = 0.0f;        /* static lean offset      */
p->balance.lean_limit_rad    = 0.10472f;    /* 6°  — total target clamp */
p->balance.stick_lean_max_rad= 0.0523599f;  /* 3°  — full pitch stick   */
p->balance.turn_max_steps    = 600.0f;      /* full roll → differential */
p->balance.max_wheel_accel   = 60000.0f;    /* PID output clamp         */
p->balance.tilt_fault_rad    = 0.6108652f;  /* 35° — declared fallen    */
p->balance.output_invert     = false;
p->balance.vel_bias_gain     = 0.0f;        /* outer loop OFF by default */
```

Tune in this order (TESTING.md §7.4): `kp` until it oscillates, back off, then
`kd` to damp, then a little `ki` only if it drifts. `OMNIS_TUNE_KP_FROM_POT`
(§5.1) lets the S1 slider sweep `kp` live without resetting the integrators.

> **`output_invert` is not a general-purpose fix.** If the wheels drive *away*
> from the fall on **both** wheel pairs, set it. If **only one** pair is wrong,
> stop — that means the per-pair wheel-sign derivation in `balance.h` does not
> match your hardware, and the flag would hide the problem rather than fix it.

### 4.8 Radio — `channel_map`, `rc`

```c
p->channel_map.throttle   = 1;   p->channel_map.pitch        = 2;
p->channel_map.roll       = 3;   p->channel_map.yaw          = 4;
p->channel_map.kill_switch= 6;   /* ARM: HIGH = permitted, LOW/MID = never */
p->channel_map.drive_mode = 8;   /* LOW flat · MID balance · HIGH auto     */
p->channel_map.speed_limiter = 9;
p->channel_map.tune_pot      = 10;
p->channel_map.arm_switch_invert = false;
```

```c
p->rc.crsf_baud        = 420000u;  /* ExpressLRS default for CRSF        */
p->rc.stick_deadzone   = 0.04f;    /* rescaled, not just zeroed          */
p->rc.center_tolerance = 0.10f;    /* arming needs every stick this close */
p->rc.pin_probe_ms     = 1500u;    /* no frames -> try the other RX pin  */
p->rc.min_link_quality = 0u;       /* uplink LQ floor [%]; 0 disables    */
```

Channel order varies by transmitter model and mode. Read the live channel dump
in the telemetry (TESTING.md §4.1) and set the map to match rather than
re-mapping in the transmitter.

### 4.9 Safety thresholds — `imu`, `safety`

```c
p->imu.disagree_thresh_rad      = 0.2617994f;  /* 15° — the two IMUs disagreeing  */
p->imu.flat_tilt_fault_rad      = 0.7853982f;  /* 45° — armed flat and tipped     */
p->imu.comm_fail_reads          = 25u;         /* consecutive failed I²C reads    */
p->imu.frozen_reads             = 50u;         /* identical bursts = stuck sensor */
p->imu.cal_samples              = 1000u;       /* boot gyro-bias samples (2 s)    */
p->imu.cal_gyro_std_max_radps   = 0.02f;       /* stillness gate                  */

p->safety.upright_min_rad       = 1.0471976f;  /* 60° — standing on a wheel pair  */
p->safety.arm_flat_max_tilt_rad = 0.5235988f;  /* 30° — flat mode arms below this */
p->safety.balance_settle_ms     = 400u;        /* hold still while filters settle */
p->safety.overrun_window_ticks  = 500u;        /* one second at 500 Hz            */
p->safety.overrun_fault_count   = 25u;         /* overruns per window that fault  */
```

The validator enforces `upright_min_rad > arm_flat_max_tilt_rad`, so the two
modes can never both claim the same pose.

Loosen the 15° disagreement threshold only after confirming the IMU **mounting
descriptors** are right (TESTING.md §3.3 has a wizard that derives them from two
poses). A mounting error looks exactly like a disagreement fault, and widening
the threshold to "fix" it throws away the redundancy that justifies two IMUs.

---

## 5. Build-time switches

### 5.1 `main/omnis_config.h`

| Switch | Default | Effect |
|---|---|---|
| `OMNIS_TICK_RATE_HZ` | `500` | Superloop rate. Everything timing-related derives from it |
| `OMNIS_I2C_SCL_HZ` | `400000` | I²C clock. Lower it if the bus is marginal on a hand-built board |
| `OMNIS_IMU_READ_ALTERNATE` | `0` | `1` = read one IMU per tick, halving I²C time; each filter then runs at 250 Hz on its own `dt`. Use if the loop overruns |
| `OMNIS_RUN_MOUNT_WIZARD` | `0` | `1` = boot into the two-pose procedure that derives the IMU mounting constants and prints them to paste in. Never arms in this mode |
| `OMNIS_BENCH_STEP_TEST` | `0` | `1` = ignore the radio and cycle fixed motions through the real pipeline. **WHEELS OFF THE GROUND** — it starts ~3 s after boot whether or not anyone is watching |
| `OMNIS_TELEMETRY_ENABLED` | `1` | The once-a-second status line |
| `OMNIS_TELEMETRY_IN_BALANCE` | `0` | `1` = keep printing while balancing. A blocking log line every second is a periodic disturbance to a balance loop; off by default, with a summary printed on disarm instead |
| `OMNIS_TUNE_KP_FROM_POT` | `0` | `1` = the S1 slider sweeps balance `kp` live, without resetting integrators |

### 5.2 `sdkconfig.defaults` — the ones that matter

| Option | Why |
|---|---|
| `CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1=y` | The superloop owns CPU1; CPU0 keeps the IDF's housekeeping |
| `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n` | A superloop never yields, so CPU1's idle task never runs and the watchdog would fire immediately |

To regenerate `sdkconfig` from these, delete it and rebuild.

---

## 6. Troubleshooting index

| Symptom | Start at |
|---|---|
| Won't flash / won't connect | §1.3 — BOOT + RESET |
| Boot halts, buzzer repeating, log every 2 s | Parameters or tick invalid. The log names which |
| `kinematics self-check … *** FAIL ***` | The FPU or the port, not your parameters — the check uses fixed reference geometry |
| Disagreement fault at every boot | IMU mounting descriptors. TESTING.md §3.3 wizard — do **not** widen the threshold |
| One wheel backwards | §4.1 `dir_invert` |
| Forward fine, strafe backwards | §4.2 roller handedness — negate all four |
| Rear wheels don't turn during slow spins | **Expected.** §4.5 — their lever is 1.5% of the front's |
| All four wheels spin equally fast during a pure yaw | Stale X-drive kinematics — the rear pair should crawl at ~59 steps/s per rad/s |
| Wheels chatter near zero | §4.5 `deadband_steps` |
| Steps lost when the stick snaps | §4.5 `max_accel_steps_s2` — lower it |
| Robot curves during a commanded strafe | Per-wheel clipping has crept in somewhere. The null-space metric in the telemetry should be ~0 |
| Loop overruns | §5.1 `OMNIS_IMU_READ_ALTERNATE`, or lower `OMNIS_I2C_SCL_HZ` if the bus is retrying |
| Balance falls away from the lean, both pairs | §4.7 `output_invert` |
| Balance falls away on **one** pair only | **Do not set `output_invert`.** `balance.h`'s wheel-sign derivation does not match your hardware |
