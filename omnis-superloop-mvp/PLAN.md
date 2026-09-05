# OMNIS Superloop MVP — Build Plan

**Status:** awaiting approval. No code written yet.
**Created:** 2026-09-05
**Target:** ESP32-S3-WROOM-1-N16R8 on EdgeHax S3 Pro, ESP-IDF v6.0.2 (native macOS install at `/Users/krishnaraj/.espressif/v6.0.2/esp-idf`)

---

## 0. What this is

A deadline-driven bridge build of the OMNIS core: **plain superloop, no RTOS
tasks**. Hardware timer ISR sets a flag; `while(1)` in `app_main` does the work.
It exists to get the robot moving and balancing while the full FreeRTOS
architecture described in the main `omnis-info.md` is still being learned.

It is **not** a replacement for that architecture. Everything excluded here is
deferred, not abandoned.

### Confirmed chassis facts (2026-09-05)

Front edge is the OLED / button end. Body frame **+X forward, +Y left, +Z up**,
identical across the kinematics and control code.

| | Corner | I2C | Faces | Mount constant |
|---|---|---|---|---|
| IMU A | front-left | `0x68` | forward | `IMU_MOUNT_IDENTITY` |
| IMU B | rear-right | `0x69` | rearward | `IMU_MOUNT_ROT_Z_180` |

The two modules are **antiparallel**; the 180° correction must be applied before
the EKF or the 15° disagreement fault fires at every boot. Wheel labelling
follows from the front edge: FL = front-left, and FL/RR are the `δ = -1` roller
pair.

### In scope
| # | Item |
|---|---|
| 1 | 500 Hz balance/EKF loop — dual-IMU fusion, inverse-covariance weighted, >15° disagreement fault |
| 2 | 4-wheel mecanum IK/FK — the already-verified equations, ported unchanged |
| 3 | CRSF UART parser — throttle→`vx`, yaw→`w`, pitch→secondary `vx`, roll→`vy` |
| 4 | STEP-pulse generation via RMT only, all four A4988s, nothing in the hot path |
| 5 | Buzzer fault/failsafe indicator — IMU disagreement, RC link-loss |

### Explicitly excluded — not implemented, not stubbed
OLED/SSD1306 · all six buttons and the config menu (§14) · microSD/FATFS ·
`params.json` on card · data logging · OTA/Wi-Fi · battery ADC sense · any
FreeRTOS primitive authored by us.

---

## 1. Three things to settle before Stage 1

These change what gets built. I recommend the marked option in each case.

### 1a. "No FreeRTOS" has a hard floor — confirm the line I'm drawing

ESP-IDF **is** FreeRTOS. `app_main` is itself a FreeRTOS task; the UART, I2C,
RMT and GPTimer drivers all use FreeRTOS internals (ring buffers, ISR-to-task
notifications) inside themselves. There is no way to call `rmt_transmit()` and
have zero FreeRTOS underneath it, short of writing bare-metal register drivers
for every peripheral — which is a much bigger job than this deadline allows.

**The line I propose:** *no RTOS primitive appears anywhere in OMNIS-authored
code.* No `xTaskCreate`, no queues, no semaphores, no mutexes, no `vTaskDelay`.
One superloop, living entirely inside `app_main`, driven by a GPTimer ISR and
`volatile` flags. IDF driver internals are treated as part of the hardware.
Every source file gets a header comment stating this rule.

**Consequence that must be handled, not discovered later:** a `while(1)` that
never yields starves the idle task on its core and trips the task watchdog.
Plan: pin the superloop's core, disable the idle-task watchdog for that core in
`sdkconfig.defaults`, and have the loop spin on the tick flag. This is a real
tradeoff of the superloop choice and I'll document it in `BUILD-LOG.md` rather
than hide it.

**Please confirm this line is what you meant.**

### 1b. RMT rate-change strategy — the single biggest design decision here

Verified against the installed IDF: ESP32-S3 has **exactly 4 RMT TX channels**
(`RMT_LL_TX_CANDIDATES_PER_INST = 4`) — precisely enough for four steppers, zero
spare. It supports infinite-loop TX with hardware auto-stop.

Two ways to make a variable-rate pulse train:

- **Option A — infinite loop.** One 2-symbol payload per channel,
  `loop_count = -1`. Perfectly uniform output. But changing the rate requires
  `rmt_disable()` → `rmt_enable()` → `rmt_transmit()`, because the header
  explicitly notes a looping channel can only be terminated by `rmt_disable()`.
  Doing that on four channels at up to 500 Hz is a lot of register churn and a
  visible glitch on every rate change.

- **Option B — burst re-arm (recommended).** Each tick, queue a short block of
  pulses at the current period via `rmt_transmit()` with a copy encoder and
  `queue_nonblocking`. With `trans_queue_depth = 4` the driver's own ISR chains
  the next transaction back-to-back, so the pulse train is continuous. Rate
  updates land naturally on tick boundaries, no disable/enable, and it gives an
  obvious place to apply the acceleration limit §13c asks for.

I recommend **B**. Say if you want A.

*Flagging once, then building what you asked for:* LEDC would do variable-rate
step pulses in about a tenth of the code — set a frequency on four channels and
walk away. RMT is the right call if you want the step-count precision and the
per-pulse control that a future homing/positioning mode needs, and you asked for
RMT explicitly, so RMT is what I'll build. Just noting the cheaper path exists.

### 1c. How much of the balance controller is in scope?

Scope item 1 says "500 Hz balance/EKF control loop" but the parenthetical only
describes fusion and the fault check. `omnis-info.md` §13 describes three
separable pieces:

1. Attitude EKF + fusion + fault — **unambiguously in scope**
2. Inner 500 Hz lean-angle PID driving the two grounded wheels — *probably* in scope
3. Outer ~20 Hz velocity-bias loop (§13b) + "which side is down" detection (§11c)

**Recommendation:** build 1 and 2. Leave 3 out of the MVP — the outer loop is
the piece that most needs empirical tuning on real hardware, and side-detection
adds a mode state machine this build doesn't otherwise need. Fixed assumption:
the FL/FR pair is the grounded pair in balance mode, named as one constant so
swapping it is a one-line change. Both get logged in `BUILD-LOG.md` as known
limitations.

---

## 2. Directory layout

Standalone IDF project inside the repo (own `CMakeLists.txt`, own `main/`), so
it doesn't collide with the root `OMNIS` project. Built with
`idf.py -C omnis-superloop-mvp build`.

```
omnis-superloop-mvp/
  PLAN.md                  <- this file
  BUILD-LOG.md             <- created Stage 1, appended every stage
  omnis-info.md            <- written last (process step 3)
  CMakeLists.txt
  sdkconfig.defaults
  main/
    CMakeLists.txt
    main.c                 <- app_main + the superloop, Stage 7
    omnis_pins.h           <- GPIO map, transcribed from omnis-info §3a
    omnis_params.h/.c      <- geometry + tunables; params.json shape, in RAM
    tick.h/.c              <- GPTimer ISR, volatile tick flags, overrun count
    mecanum_kinematics.h/.c
    mpu6050.h/.c
    attitude_ekf.h/.c
    crsf.h/.c
    step_gen.h/.c
    fault.h/.c             <- fault state machine + buzzer patterns
  test/
    test_kinematics.c      <- host-side, Cases A-D from the reference
    run_host_tests.sh
```

---

## 3. Stages

One stage per step. **After each, I stop and wait for explicit approval.**

### Stage 1 — Skeleton, pin map, params, 500 Hz tick

- `CMakeLists.txt`, `main/CMakeLists.txt`, `sdkconfig.defaults`
  (target esp32s3, 16 MB flash, idle-task WDT off on the superloop's core).
- `omnis_pins.h` — every GPIO from `omnis-info.md` §3a that this build uses:
  `COM_ENA`=1, `FL_STEP`=4, `FL_DIR`=5, `SDA`=8, `SCL`=9, `BUZZ`=14,
  `RL_STEP`=15, `RL_DIR`=16, UART TX=17, UART RX=18 (**note the §3c crossover —
  net names are from the receiver's perspective, so they're reversed**),
  `RR_STEP`=21, `FR_DIR`=40, `FR_STEP`=41, `RR_DIR`=42. Excluded-feature pins
  (10-13 SD, 2/3/39/45/47/48 buttons, 6 batt sense) are listed as comments
  marked reserved, so nothing later grabs them by accident.
- `omnis_params.h/.c` — one struct mirroring the `params.json` schema from §9f,
  populated from compile-time defaults (no SD card in this build). Geometry as
  named fields, never inline in a formula:
  `wheel_radius_mm = 30`, `wheelbase_mm = 223`, `track_width_mm = 230`.
- `tick.c` — GPTimer, 1 MHz resolution, auto-reload alarm at 2000 counts
  (500 Hz). ISR does exactly two things: sets `volatile bool g_tick_500hz`, and
  increments `g_tick_overruns` if the flag was already set. No I2C, no logging,
  nothing blocking, ever.
- `main.c` — GPIO init (COM_ENA driven to disabled at the very first
  opportunity, before anything else can spin a motor), then a bare superloop
  that consumes the tick and does nothing. Proves the timing spine.

**Done when:** it builds, and a scope on any free pin (or a counter printed once
a second) shows 500 Hz ±0.1% with zero overruns.

### Stage 2 — Kinematics

Port `assets/kinematics/mecanum_kinematics.{c,h}` in **unchanged**. It already
implements exactly the verified equations, with `mecanum_clamp` (common-factor),
`mecanum_deadband` and `mecanum_null_space`. No re-derivation, no edits beyond
the doc-comment path references.

Add `test/test_kinematics.c` — a plain host build (`cc`, no IDF), asserting the
four verified cases from §7 of the reference to 1e-3:

| Case | Input | Expected |
|---|---|---|
| A | `vx=200` | all four `f = +3395.305` |
| B | `vy=200` | `-3395.305, +3395.305, +3395.305, -3395.305` |
| C | `w=1.0` | `-3845.183, +3845.183, -3845.183, +3845.183` |
| D | `vx=200, vy=100, w=0.5` | `-224.939, +7015.550, +3170.366, +3620.244` |

Plus the IK→FK round-trip on Case D returning `(200.0, 100.0, 0.5)` and
`mecanum_null_space` returning 0 on all four.

Also add the RC→body mapping from reference §5 (`rc_to_body()`), with proposed
defaults to confirm on the bench — derived from Case D's note that ~7000
µsteps/s is this build's practical ceiling:
`VX_MAX_MMPS = 300`, `VX_SECONDARY_MMPS = 150`, `VY_MAX_MMPS = 300`,
`W_MAX_RADPS = 1.5`, `MAX_STEP_RATE = 7000`, `DEADBAND = 20` steps/s.

**Done when:** `run_host_tests.sh` passes all four cases plus round-trip on the
Mac, before any of it runs on hardware.

### Stage 3 — MPU6050 driver + attitude EKF

> **Largely pre-built.** `assets/control/` now contains the EKF, dual-IMU
> fusion, fault check and mounting layer, host-tested with 76 assertions
> (`./run_host_tests.sh`). This stage becomes: port those files in unchanged,
> write the MPU6050 I2C driver beneath them, wire the boot-time gyro
> calibration, and run `imu_mount_resolve()` once per IMU to confirm the mount
> constants. The filter maths is done and verified.

- `mpu6050.c` — register-level driver on the new `i2c_master` API (not the
  deprecated `i2c.h`). Both devices on the shared bus, 0x68 and 0x69, Fast-mode
  400 kHz. Wake from sleep, ±4g / ±500°/s, DLPF on. One burst read of the 14
  accel+temp+gyro registers per device per tick. **All I2C from the superloop,
  never from the ISR** — this is the rule that shapes the whole design.
  Boot-time gyro-bias averaging per §12 step 1.
- `attitude_ekf.c` — the 4-state filter from §11b, run twice independently:
  state `[roll, pitch, bias_roll, bias_pitch]`, gyro-driven prediction, accel
  `atan2` measurement update, and the adaptive-R inflation when
  `| |a| − 1g | > 0.2g`. Lever-arm compensation deliberately skipped per §11b's
  own "skip for v1".
- Fusion — inverse-covariance-weighted average of the two estimates, per §11c.
- Fault — if the two IMUs' estimates differ by more than **15°**, raise
  `FAULT_IMU_DISAGREE`. (Note: you wrote "disagree >15° from vertical"; §11c
  specifies disagreement *between the two IMUs*, which is what I'm
  implementing. Say if you meant the other thing.)

**Timing budget I'll measure and report:** two 14-byte burst reads at 400 kHz is
roughly 290 µs of pure bus time before overhead — about 15% of the 2 ms tick.
Feasible, but it is the tightest thing in this build, so Stage 3 reports the
real measured number, not this estimate.

### Stage 4 — CRSF parser

- UART1 on GPIO17 = ESP32 **TX**, GPIO18 = ESP32 **RX** (the §3c reversal),
  420000 baud 8N1.
- Byte-at-a-time frame state machine: sync `0xC8`, length, type `0x16`, payload,
  **CRC8 poly 0xD5 (DVB-S2)** over type+payload. Bad CRC drops the frame and
  counts it; it does not resync mid-frame.
- Unpack 22 payload bytes into 16 × 11-bit channels; map CRSF 172–1811 to
  −1.0…+1.0.
- Channel map from §9f's `rc.channel_map`: throttle 1, pitch 2, roll 3, yaw 4.
  Kill switch on ch 6. Named constants, not literals.
- **Link loss:** track microseconds since last valid frame. Past a timeout
  (proposed 250 ms — well over the slowest ExpressLRS packet interval), raise
  `FAULT_RC_TIMEOUT`. §7f is explicit that hold-last-value is the wrong
  behaviour here, so the parser reports the timeout and the failsafe path acts.
- Draining the UART ring buffer happens in the superloop with
  `uart_read_bytes(..., 0)` — non-blocking, zero timeout, no task, no queue of
  ours.

**Done when:** a real transmitter's stick movements print as four normalized
values on the monitor, and switching the TX off raises the timeout fault within
250 ms.

### Stage 5 — RMT step generation

- Four TX channels (all four the S3 has), 1 MHz resolution → 1 µs symbol
  granularity, `mem_block_symbols = 64`, `trans_queue_depth = 4`,
  `flags.queue_nonblocking = 1` so the superloop can never block on a full queue.
- One copy encoder shared by all four channels.
- Per tick, per wheel: convert the signed rate to a period, build a small symbol
  burst covering the next tick, `rmt_transmit()`. (Option B from §1b — pending
  your confirmation.)
- **DIR handling:** the A4988 needs DIR stable ≥200 ns before a STEP edge. On a
  sign change, the wheel is stopped for one full tick, DIR is written, and
  pulses resume the tick after. 2 ms of margin against a 200 ns requirement —
  vastly more than needed, and free, since the deadband already holds
  near-zero wheels stopped.
- Per-wheel `dir_invert` flags at this layer only. Reference §6 is explicit:
  never fix a backwards motor by flipping a matrix sign.
- Acceleration limit on the commanded rate (§13c) — the only defence against
  step loss on an encoder-less build.
- `step_gen_stop_all()` — an immediate, synchronous all-channels-off used by
  the failsafe path.

**Done when:** commanding `vx=200` spins all four the same way at a scope-verified
3395 Hz, and `vx=vy` leaves FL and RR completely stationary — the bench test the
reference §4 calls for.

### Stage 6 — Fault state machine + buzzer

- `fault.c` — a bitmask of `FAULT_IMU_DISAGREE`, `FAULT_RC_TIMEOUT`,
  `FAULT_RC_KILL`, plus latching rules (IMU disagreement latches until reboot;
  RC timeout clears when the link returns).
- **The failsafe action is one thing: drive COM_ENA to disabled and stop all
  RMT channels.** Per §7g point 5 this is written and testable before the
  superloop that calls it, not bolted on afterwards.
- Buzzer on GPIO14 — it's a driver-IC module, so the pin is a logic-level
  enable, not a PWM tone input. Distinct non-blocking patterns clocked off the
  tick counter: fast double-chirp = RC loss, continuous = IMU disagreement,
  single short = boot OK. No delays, no blocking.
- Documented in the log: in balance mode, cutting COM_ENA drops the robot.
  §7f says that's the accepted behaviour for this class of machine; a
  controlled sit-down is out of MVP scope.

### Stage 7 — Superloop integration

`app_main`: init in dependency order (COM_ENA disabled first → params → tick →
I2C/IMUs → gyro-bias calibration → UART/CRSF → RMT), then:

```
while (1) {
    if (!g_tick_500hz) continue;
    g_tick_500hz = false;

    crsf_poll();                  /* drain UART ring buffer, non-blocking */
    mpu6050_read_both();          /* I2C — superloop only, never the ISR */
    ekf_update_both();
    attitude_fuse_and_check();    /* -> FAULT_IMU_DISAGREE */

    fault_evaluate();
    if (fault_active()) { failsafe(); buzzer_service(); continue; }

    rc_to_body(...);              /* -> vx, vy, w  (or balance PID in 2-wheel) */
    mecanum_inverse(...);
    mecanum_clamp(...); mecanum_deadband(...);
    accel_limit(...);
    step_gen_update(...);         /* RMT re-arm */
    buzzer_service();
}
```

Fault check sits **before** any motor command, every single tick.

**Done when:** the whole thing runs on hardware, sticks drive wheels, killing the
TX stops the wheels and sounds the buzzer within 250 ms, and the tick-overrun
counter stays at zero over a multi-minute run.

---

## 4. Process steps 3 and 4

- `omnis-superloop-mvp/omnis-info.md` — full copy of the original, edited to
  describe *this* build: superloop-not-RTOS and why, the excluded features
  marked deferred-not-abandoned with a pointer to the parent doc's section for
  each, §10 filled in with the verified kinematics instead of the
  "derive externally" placeholder, and geometry constants stated.
- `omnis-superloop-mvp/BUILD-LOG.md` — created at Stage 1, appended after every
  stage: date, what was built, decisions and why, open questions, known
  limitations, what's left. A running history, not a final-state summary.

---

## 5. Open questions carried into the build

| # | Question | Where it bites | Proposed default |
|---|---|---|---|
| 1 | Is the §1a "no RTOS in our code" line what you meant? | Everything | as stated |
| 2 | RMT Option A or B? | Stage 5 | **B** |
| 3 | Is the inner balance PID in scope? Outer loop and side-detection out? | Stage 7 | inner in, outer out |
| ~~4~~ | ~~IMU fault definition~~ — **RESOLVED**: the metric is the angle between the two IMUs' *estimates of vertical*, which is both readings of your wording at once. Implemented and tested. | — | — |
| 5 | Kill switch on CRSF channel 6, 2-position (§7d read "skill" as "kill") | Stage 4 | ch 6, high = killed |
| 6 | `VX_MAX_MMPS` etc. — proposed from Case D's ceiling, unverified on hardware | Stage 2 | 300 / 150 / 300 / 1.5 |
| ~~7~~ | ~~Which wheel pair is grounded in balance mode~~ — **RESOLVED**: front is the OLED end, a short edge, so the chassis tips about body **Y**. Lean angle is **pitch**; the grounded pair is FL+FR or RL+RR. `omnis_balance_pair_from_pitch()` in `omnis_imu_mounting.h`. | — | — |
| ~~8~~ | ~~IMU mount arrow interpretation~~ — **RESOLVED**: no longer an assumption. `imu_mount_resolve()` derives the descriptor from two poses (level, then nose-down) with no interpretation, verified against all 24 right-handed mountings. Run once per IMU at Stage 3 bring-up. | — | — |

Only question 1 remains open, and it blocks nothing before Stage 1. Questions 4
and 7 were resolved by the board photo and the confirmed front edge; question 8
was closed by deriving the mounting in firmware instead of assuming it.

---

**Awaiting approval before writing any code.**
