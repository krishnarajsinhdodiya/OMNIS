# OMNIS Superloop MVP — Build Log

A running history of this build: what was made, why it was made that way, what
was learned, and what is still open. Appended after every stage — read
top-to-bottom it should explain how the thing came to be, not just what it
currently is.

Newest entries at the bottom.

---

## 2026-09-05 — Planning

**Built:** `PLAN.md`. No code.

**Decisions:**

**1a — What "no FreeRTOS" actually means here.** ESP-IDF *is* FreeRTOS; `app_main`
is a task and every IDF driver uses RTOS primitives internally. There is no way
to reach zero FreeRTOS short of writing bare-metal register drivers for every
peripheral, which the deadline does not allow. The rule adopted instead: **no RTOS
primitive appears in OMNIS-authored code** — no `xTaskCreate`, queues,
semaphores, mutexes, or `vTaskDelay`. IDF driver internals are treated as part of
the hardware.

Arduino IDE was offered as an alternative and rejected **on evidence**:

- It does not remove FreeRTOS. `loop()` *is* a FreeRTOS task — the installed core
  3.3.10's own `cores/esp32/main.cpp:113` calls
  `xTaskCreateUniversal(loopTask, ...)`. Arduino moves the `xTaskCreate` out of
  your code and into the core where you cannot see or control it. That is
  strictly *less* control over the exact thing the rule is about.
- It breaks decision 1b. Arduino's RMT wrapper has no transaction queue; its own
  `esp32-hal-rmt.h` states a second `rmtWriteAsync()` issued mid-transfer returns
  `false`. Only `rmtWriteLooping()` (Option A) is reachable there.
- The IDF toolchain was already installed and documented, so Arduino's usual
  setup-speed advantage did not apply.

Both toolchains give an identical "no RTOS in my code" guarantee, so nothing was
traded away by staying with ESP-IDF v6.0.2.

**1b — RMT strategy: Option B, burst re-arm.** Each tick, queue a short pulse
block at the current period with `queue_nonblocking`. Rejected Option A (infinite
loop) because changing rate requires `rmt_disable()`/`enable()`/`transmit()` on
four channels at up to 500 Hz, with a glitch on every rate change.

**1c — Balance scope:** inner 500 Hz lean-angle PID in; §13b's outer
velocity-bias loop and live side-detection out of the MVP.

**Learned:** ESP32-S3 has **exactly 4 RMT TX channels**
(`RMT_LL_TX_CANDIDATES_PER_INST = 4`) — precisely enough for four steppers, zero
spare. Verified against the installed IDF, not assumed.

**Open:** questions 5 and 6 (CRSF kill-switch channel; RC scaling constants)
carry stated defaults to be confirmed on the bench at the stage that uses them.

---

## 2026-09-05 — Stage 1: skeleton, pin map, params, 500 Hz tick

**Built:**

| File | What it does |
|---|---|
| `CMakeLists.txt`, `main/CMakeLists.txt` | Standalone IDF project, `MINIMAL_BUILD ON` |
| `sdkconfig.defaults` | The watchdog/affinity settings that make a superloop legal |
| `main/omnis_pins.h` | Every GPIO from omnis-info.md §3a, plus reserved pins |
| `main/omnis_params.h/.c` | `params.json` §9f schema as a struct, compile-time defaults |
| `main/tick.h/.c` | 500 Hz GPTimer, ISR sets a flag, overrun counter |
| `main/main.c` | Safe GPIO state, then the superloop with self-measurement |

**Result:** builds clean, zero warnings. 154 KB binary, 85% of the app partition
free.

### Design decisions and why

**The superloop needs two sdkconfig settings to be legal.** A `while(1)` that
never yields starves its core's idle task and the task watchdog trips within
seconds. Fix: pin `app_main` to CPU1 (`ESP_MAIN_TASK_AFFINITY_CPU1`) and stop
watchdogging CPU1's idle task (`ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n`). CPU0 keeps
its idle watchdog and runs the system's housekeeping. The **interrupt** watchdog
is deliberately left on — it catches a hung ISR, which is a bug worth hearing
about.

**The flag is a plain `volatile bool`, and that is sufficient rather than
merely expedient.** `app_main` is pinned to CPU1 and the GPTimer interrupt is
allocated on whichever core registers it — CPU1, since `tick_init()` runs from
`app_main`. Producer and consumer are the same core, so there is no cross-core
visibility problem and no barrier is needed. **If `app_main` is ever un-pinned,
this reasoning breaks and the flag must become atomic.** Recorded here because it
is exactly the kind of assumption that silently rots.

**Overruns drop the tick rather than catching up.** When the ISR fires with the
previous tick still pending, it counts an overrun and moves on. Trying to catch
up would make `dt` wrong in the EKF, which is worse than a skipped cycle.

**Hardware auto-reload, not ISR-rewritten alarms.** The period is maintained by
the timer peripheral, so a late ISR delivers a late tick rather than accumulating
drift.

**`gpio_set_level()` is called *before* `gpio_config()` for COM_ENA.** Looks
backwards, is deliberate: it primes the output latch so the pin drives the
disabled level the instant it becomes an output, instead of briefly driving
whatever was in the register. §7g point 5 wants the kill line to be one of the
earliest things that works, and this is the earliest it can possibly be.

**The status report sits outside its own measurement window.** A log line takes
far longer than one 2 ms tick, so counting it would guarantee an overrun every
second and drown the real signal. Stats reset after printing. This report is
scaffolding and goes away once the loop has real work.

**Headroom is reported as well as overruns.** `tick_wait()` returns the
microseconds it spent waiting. Overruns are a *lagging* indicator — by the time
one appears the loop is already too slow. Headroom trending toward zero is the
*leading* indicator.

### Learned

- **`CONFIG_ESP_CONSOLE_UART_BAUDRATE` is silently ignored unless
  `CONFIG_ESP_CONSOLE_UART_CUSTOM=y`.** The first attempt set the baud and it
  stayed at 115200 with no warning. Caught by checking the generated `sdkconfig`
  rather than trusting `sdkconfig.defaults`. **Worth doing for every option
  added** — a wrong or unprompted Kconfig name fails silently.
- Settled on **460800**, not 921600, because esptool already flashes this board
  at 460800 — evidence the bridge handles that rate. At 115200 a ~130-character
  log line occupies ~11 ms, five whole ticks.
- The v6.0.2 environment will not activate via `export.sh` on this machine: it
  looks for a py3.11 venv while the install is py3.14. Use
  `. ~/.espressif/tools/activate_idf_v6.0.2.sh` instead. This matches the warning
  already in `phase0-toolchain-environment.md` §7.

### Known limitations

- **The 500 Hz figure is not yet hardware-verified.** It builds and the logic is
  right, but nothing has run on the board. The Stage 1 test is what confirms it.
- `max_accel_steps_s2 = 20000` is a placeholder. It is the §13c lost-step
  defence, and the correct value depends on the A4988 Vref current limit that has
  not been set yet (§3f).
- RC scaling constants (`vx_max_mmps` etc.) are derived from Case D's practical
  ceiling but unverified on hardware — PLAN.md question 6.
- `PIN_TICK_HEARTBEAT` (GPIO35) is bring-up scaffolding and should be removed
  once the tick is trusted.

### Still to do

Stages 2–7: kinematics port + host tests, MPU6050 driver + EKF, CRSF parser, RMT
step generation, fault/buzzer, superloop integration.

---

## 2026-09-07 — Stage 1 bench test: PASSED

First hardware run. Flashed to the EdgeHax S3 Pro over `/dev/cu.usbmodem101`,
USB cable only, battery disconnected.

### Result

```
I (0)     main_task: Started on CPU1
I (10)    omnis: COM_ENA driven DISABLED (GPIO1 = 1) - motors are off
I (10)    omnis:   yaw lever arm k = 226.5 mm, k/r = 7.550  (expect 226.5, 7.550)
I (10)    tick:  500 Hz tick running (period 2000 us)
I (2010)  omnis: 500.16 Hz | period 2000-2000 us | headroom min 1998 us | overruns 0 | uptime 2 s
I (10010) omnis: 500.13 Hz | period 2000-2000 us | headroom min 1998 us | overruns 0 | uptime 10 s
```

| Criterion | Target | Measured | |
|---|---|---|---|
| Overruns | 0, sustained | **0** across all 10 windows | pass |
| Period jitter | ±5 us | **2000–2000 us**, no measurable jitter | pass |
| Geometry | k=226.5, k/r=7.550 | exact | pass |
| COM_ENA at boot | disabled | `GPIO1 = 1`, before anything else | pass |

Two settings confirmed working that could only be confirmed on hardware:

- `main_task: Started on CPU1` — the `ESP_MAIN_TASK_AFFINITY_CPU1` pinning took
  effect. This is the premise the plain-`volatile` flag argument rests on.
- No task-watchdog trip over 10 s of a never-yielding loop — the CPU1 idle-WDT
  setting took effect.

**Headroom: 1998 us of 2000.** The Stage 1 loop body costs ~2 us, so 99.9% of the
tick budget is still free for Stages 2–7. The console prints at GPIO43/44 as
expected (`cpu_start: GPIO 44 and 43 are used as console UART I/O pins`).

### Bug found and fixed: the reported rate was biased high

The rate read a consistent **500.13 Hz** rather than 500.00, while `period`
measured an exact `2000-2000 us`. The period is measured independently and is
ground truth, so the discrepancy had to be in the reporting arithmetic — and it
was. Two compounding off-by-ones in `main.c`:

1. `window_start` was set immediately *after* the status print, which is
   mid-period, not on a tick edge.
2. The window counted N ticks but divided by the elapsed time of **N-1**
   intervals plus a partial one.

Arithmetic confirms it exactly: a first-tick offset of 1740 us gives
`500 x 1e6 / (1740 + 499x2000) = 500.13 Hz`, the precise figure observed.

Fixed by anchoring `window_start` on the first tick edge of each window and
dividing by `window_ticks - 1`, since N tick edges span N-1 intervals.

**Why fix a cosmetic diagnostic:** this same reporting block is how loop health
gets judged in every later stage. A rate that reads systematically high hides
exactly the degradation it exists to reveal — and when a real slowdown appears in
Stage 3 or 5, nobody wants to be re-deriving whether the number is trustworthy.

**The hardware was never wrong.** No timing behaviour changed; only the number
printed about it.

### Still to do

Stages 2–7, unchanged. Stage 1's `PIN_TICK_HEARTBEAT` scaffolding on GPIO35
remains and should be removed once the tick is no longer under suspicion.

---

## 2026-09-07 — Stage 2: kinematics port, RC mapping, drive pipeline

**Built:**

| File | What it does |
|---|---|
| `main/mecanum_kinematics.{c,h}` | **Byte-identical copy** of `assets/kinematics/`. Not re-derived, not edited. |
| `main/drive.{c,h}` | RC sticks -> body velocity, and the full pipeline in the required order |
| `test/test_kinematics.c` | 76 host assertions against the reference's verified numbers |
| `test/run_host_tests.sh` | `cc` + run. No ESP-IDF, no hardware. |

Also added stick-polarity inverts to `omnis_rc_scale_t`, because the reference is
emphatic that a reversed transmitter axis is corrected at the mapping layer and
nowhere else.

**Result:** 76 host assertions pass, 0 fail. Firmware builds clean, zero
warnings, 161 KB (85% of the partition still free).

### Design decisions and why

**The port is byte-identical, verified by `diff`.** The whole value of that
module is that its numbers are checked; "porting" it by retyping would throw that
away. `diff -q` against the asset is part of the stage's verification.

**The pipeline order is encoded once, in `drive_solve()`, not trusted to call
sites.** Reference §6 has two ordering rules that are easy to get wrong and
silent when you do:

- *Clamp before computing FK.* Feeding pre-clamp rates to forward kinematics is
  the classic open-loop drift bug — the estimator believes a velocity the
  hardware never produced.
- *Clamp with a common scale factor, never per-wheel.* Per-wheel clipping warps
  the motion vector; a commanded strafe becomes an arc.

**The null-space health metric is taken post-clamp but PRE-deadband.** This was
the one genuinely non-obvious ordering call. `mecanum_clamp()` scales all four
wheels equally so it preserves the null space, but `mecanum_deadband()` is
deliberately a *per-wheel* operation and therefore legitimately injects
null-space energy. Measuring after it would make the metric fire on healthy
commands — and an alarm that cries wolf gets ignored, which is worse than not
having it.

**Forward kinematics is taken AFTER the deadband**, for the mirror-image reason:
a deadbanded wheel genuinely will not turn, so the commanded-velocity estimate
must reflect that.

**`vx` is deliberately NOT clamped in `drive_rc_to_body()`.** Throttle and pitch
are additive into `vx`, so full deflection on both gives 450 mm/s against a
300 mm/s nominal. Reference §5 says handle this *either* by clamping combined
`vx` *or* by relying on the wheel clamp — "but not both, or the response becomes
non-linear near full stick." Chose the wheel clamp: clamping `vx` alone while
leaving `vy` and `w` untouched would rotate the commanded motion vector, so a
full-throttle diagonal would quietly become a *different* diagonal. The common
factor scales all three components together, so direction survives and only
magnitude drops.

**A NaN stick reads as centred.** `clamp_unit()` fails both comparisons on NaN,
so it is caught explicitly. Without that, one NaN channel propagates through the
IK into a step rate. A dead stick reading as centred is the safe interpretation.

**Boot-time self-check on the target.** The same four cases run on the Xtensa FPU
at startup, and the firmware refuses to enter the superloop if they fail.
IEEE-754 says float32 must agree bit-for-bit with the Mac, but "must" and "does"
are different claims and this costs microseconds to settle. A silent divergence
would otherwise surface as a robot that drives subtly wrong.

### Verified numbers

All four reference cases reproduce exactly:

| Case | Input | FL | FR | RL | RR |
|---|---|---|---|---|---|
| A | `vx=200` | +3395.305 | +3395.305 | +3395.305 | +3395.305 |
| B | `vy=200` | −3395.305 | +3395.305 | +3395.305 | −3395.305 |
| C | `w=1.0` | −3845.183 | +3845.183 | −3845.183 | +3845.183 |
| D | `vx=200 vy=100 w=0.5` | −224.939 | +7015.550 | +3170.366 | +3620.244 |

Case D round-trips through FK to `(200.0, 100.0, 0.5)`; null space `-0.0002`
(float32 noise). Reference §4's bench test holds: a `vx = vy` diagonal leaves FL
and RR **exactly** `0.0f`, not merely small.

### Known limitations

- The per-tick stick input is **synthetic** — a slow circle in (vx, vy) with some
  yaw, so all four wheels vary and both the clamp and deadband get exercised.
  The CRSF parser replaces it in Stage 4.
- Nothing drives a motor yet. `sol.rates` is computed and measured but discarded;
  Stage 5's RMT generator is what consumes it.
- Stick polarity inverts all default to `false` — unverified until a radio is
  bound in Stage 4 (PLAN.md question 6 territory).
- `vx_max_mmps` and friends remain derived-but-unverified.

### Still to do

Stages 3–7: MPU6050 + EKF, CRSF parser, RMT step generation, fault/buzzer,
integration.

---

## 2026-09-15 — Board Rev 2.0 pinout migration

**Context:** the builder supplied the Rev 2.0 schematic and asked for every
remaining stage to be finished in one pass (Claude access lapses for ~2 months
from 2026-09-16; the board is not yet assembled). Per-stage approval is replaced
by per-stage commits, target builds with zero warnings, host tests, and
`TESTING.md` for the bench. Recorded in PLAN.md.

**Built:** `main/omnis_pins.h` rewritten; `main/main.c` loses COM_ENA and the
GPIO35 heartbeat; `sdkconfig.defaults` flash header 16 MB -> 8 MB; master
`assets/documentation/omnis-info.md` §2/§3/§7/§8/§9 updated.

### How the pinout was verified

Three sources existed and they did not all agree:

1. The rendered schematic image the builder attached.
2. `assets/pcb/OMNIS.kicad_sch`.
3. `assets/pcb/rev2-pin-assignment.md`, a planning document.

Rather than read pin numbers off a picture, every ESP32 pin was traced through
the KiCad file's wire segments to its global label (symbol at 231.14,125.73; lib
pin offsets applied; BFS over wire endpoints). **The image and the schematic
agree on all 36 GPIOs.** The planning doc disagrees on seven nets — FR_STEP,
FR_DIR, BL_DIR and four buttons. The schematic is authoritative and is what
`omnis_pins.h` encodes; the full table is in that header and in master
omnis-info.md §3i.

The same trace also settled a scare from the rendered image: the `MPU2_INT`
junction appears to touch the `I2C_SDA` label. It does not. R3, `MPU1_INT` and
`MPU2_INT` join onto GPIO7 only; `I2C_SDA`/`I2C_SCL` sit directly on the GPIO8/9
pin ends with no wire to the interrupt net.

### What Rev 2.0 changes in behaviour

**COM_ENA is gone.** All four A4988 EN# pins are hardwired to GND. Consequences,
all now reflected in code and docs:

- The drivers are energised whenever 12 V is present — including before firmware
  runs. `gpio_safe_state()` now pins the STEP lines low first, because a floating
  STEP input on a live driver can move a wheel.
- The failsafe is no longer "disable the drivers". It is **"stop generating STEP
  pulses"**. The motors hold position; they do not go limp. Only unplugging the
  battery de-energises them.
- `PIN_STEPPER_EN` survives as `GPIO_NUM_NC`, so restoring the enable later (the
  plan's cuttable-link hedge onto GPIO0/3/45) is a one-line change.

**Shared IMU interrupt.** Both MPU6050 INT pins sit on GPIO7 with a 10 k
pull-up. That is only safe with both chips open-drain (`INT_PIN_CFG` = 0xC0),
which Stage 3 configures at boot.

**Hazards encoded as comments in `omnis_pins.h`:** FR_STEP on GPIO40 (an EdgeHax
LED pin — the planning doc advised against a STEP line there), RL_DIR on
strapping pin GPIO46, and GPIO38–42 depending on GPIO3 staying NC.

### Decisions

**The Stage 1 heartbeat on GPIO35 was a bug, and is removed rather than moved.**
GPIO35 is an octal-PSRAM line on the N16R8. It happened to work because PSRAM is
not enabled in this build, but toggling a PSRAM data line at 250 Hz is not
something to keep. The tick is verified, so the heartbeat has no remaining job.

**Flash header 8 MB.** The Rev 2.0 schematic symbol is
`ESP32-S3-DEVKITC-1U-N8R2` (8 MB); the board docs say N16R8 (16 MB). An 8 MB
header boots on both; a 16 MB header on a real 8 MB part is the one combination
that can misplace partitions. The app is ~0.2 MB, so nothing is lost.

### Found while reading ahead — fixed in the next commit

Two latent bugs in `assets/control/`, found before porting it, both of which
would have bitten in Stage 3 or 7:

1. **`pid_t` collides with POSIX.** `typedef struct {...} pid_t;` in
   `assets/control/pid.h` conflicts with `sys/types.h`, which IDF headers pull in.
   Confirmed on both xtensa-esp32s3-elf-gcc and host clang. The host test only
   passed because nothing it included happened to reach `sys/types.h`.
2. **The flat-frame pitch folds at the balance point.** Balance mode sits at
   pitch ≈ ±90°, where `atan2(-ax, hypot(ay,az))` reads 85° for both 85° and 95°
   of nose-down. The lean direction is lost exactly where it matters.
   `attitude-ekf-derivation.md` §7 called this "survivable" — that was wrong.

---

## 2026-09-15 — Two latent bugs fixed in `assets/control/` before porting

Both found by reading ahead to Stages 3 and 7, before any of the code reached
the target. Fixed at the source so the asset folder and the firmware stay
byte-identical.

### 1. `pid_t` collides with POSIX

`assets/control/pid.h` declared `typedef struct {...} pid_t;`. `pid_t` is the
POSIX process-id type in `sys/types.h`, which ESP-IDF headers include throughout.
Reproduced on both compilers the moment `sys/types.h` is in scope:

```
xtensa-esp32s3-elf-gcc: error: conflicting types for 'pid_t'; have 'struct <anonymous>'
host clang:             error: typedef redefinition with different types
```

The host test had passed only because nothing it included happened to reach
`sys/types.h`. Renamed to **`pid_ctrl_t`** everywhere (header, source, test, PID
diagram). Verified: `pid.c`, `attitude_ekf.c` and `imu_fusion.c` all compile on
xtensa with `-include sys/types.h -include unistd.h -Werror`.

(First rename attempt silently did nothing: macOS `sed -E` does not support `\b`.
Caught by the zero match count; redone with `perl`.)

### 2. The flat-frame pitch folds at the balance point

This one is a design error in my own earlier documentation, not a typo.

`pitch = atan2(-ax, hypot(ay, az))` is confined to ±90° because the hypot is
never negative. Balance mode sits at ±90° — the edge of the range — where the
formula **reflects**:

| Nose-down | `pitch` reads |
|---|---|
| 85° (leaning 5° back from upright on the front pair) | **84.999992°** |
| 95° (leaning 5° forward) | **84.999985°** |

Which way the robot is falling — the only thing a balancer needs — is destroyed
at the operating point. `attitude-ekf-derivation.md` §7 was titled "why it is
survivable" and argued pitch passes cleanly through 90°. That was wrong, and a
balance controller built on it would have pushed the wrong way on one side of
upright. The EKF would also have fought itself: past the fold, the gyro
prediction and the accelerometer update disagree in sign.

**Fix: change frames, not formulas.** While balancing, the body-frame accel and
gyro are rotated a further 90° about Y before reaching the EKF, so the pose looks
level to the filter. Both frames put `+X'` on the chassis top face:

| Pose | Map | Verified: 95°/85° read |
|---|---|---|
| Front pair down | `{+3,+2,−1}` | **+5.000° / −5.000°** |
| Rear pair down | `{+3,−2,+1}` | **+5.000° / −5.000°** |

So positive balance-frame pitch always means "falling forward", in either pose,
and `gyro_y'` is always its rate (checked: `+0.1` rad/s for a forward fall on
both pairs). An EKF seeded at +3° followed the lean through zero to
**−3.004°**. The flat-frame pitch keeps its *sign* through the fold, so it still
picks the grounded pair at arming — which is all it is used for now.

Constants `OMNIS_IMU_BALANCE_FRAME_FRONT_DOWN` / `_REAR_DOWN` added to
`omnis_imu_mounting.h`; the mounting self-check now covers all four descriptors.
Corrected: derivation §7 (rewritten, with a correction notice), reference §7
items 2–3, sensor-fusion reference (the disagreement metric is
rotation-invariant, so needs no change), README ("four things that will bite"),
the walkthrough's integration example, and master omnis-info.md §11c.

**Host tests:** `assets/control` 76 → **97** assertions (Case 13 added), 0 fail.

---

## 2026-09-15 — Stage 3: MPU6050 driver, IMU orchestration, EKF + fusion ported

**Built:**

| File | What it does |
|---|---|
| `main/mpu6050_regs.h` | Register map, config values, pure 14-byte burst decoder (host-tested) |
| `main/mpu6050.{c,h}` | I2C transport on the v6 `i2c_master` API; health counters |
| `main/imu.{c,h}` | Both IMUs: bus bring-up, boot calibration, per-tick fusion, frame switching, mount wizard |
| `main/omnis_time.h` | Busy-wait delay for boot only — no `vTaskDelay` exists in this build |
| `main/omnis_config.h` | Build-time switches: tick rate, I2C speed, alternate reads, mount wizard |
| `main/{attitude_ekf,imu_fusion,pid}.{c,h}`, `omnis_imu_mounting.h` | **Byte-identical** to `assets/control/` (checked with `cmp`) |
| `test/test_mpu6050.c` | 28 assertions: decoding, sign extension, full scale, plausibility, identity |
| `test/test_control.c` | The asset suite, 97 assertions, now run against the firmware copies |
| `test/run_host_tests.sh` | Rewritten to run every suite and fail if any one fails |

**Result:** target build clean, zero warnings, 189 KB (82% of the partition
free). Host tests: **3 suites, 201 assertions, 0 failures.** No hardware yet — the
board is not assembled.

### Design decisions and why

**`INT_PIN_CFG` is the first register written after reset.** Board Rev 2.0 ties
both INT pins to GPIO7. The part powers up push-pull; making it open-drain before
anything else shortens the window in which two push-pull outputs share a wire. The
window is benign anyway (interrupt disabled, both idle at the same level), but
there is no reason to hold it open. The interrupt stays disabled; the tick polls.

**Every configuration write is read back.** Catches a failed write, a wrong
address, and a second device answering in place of the intended one — all of which
otherwise produce an IMU that "works" with the wrong full-scale range, i.e. angles
that are wrong by a factor.

**MPU6500-family parts are accepted.** GY-521 boards very often carry an MPU6500
(WHO_AM_I 0x70) instead of a genuine MPU6050 (0x68). Every register used here has
the same address and meaning, so rejecting them would reject most parts actually
sold. Logged as a warning. Only the temperature formula differs, and temperature is
diagnostic.

**A dead IMU is detected from I2C behaviour, not the interrupt.** With a shared
INT line the pin cannot say which IMU fired. So `mpu6050_read()` counts
consecutive errors *and* consecutive byte-identical bursts: a live sensor's noise
floor changes the 14 bytes every sample, so 50 identical bursts (100 ms) is a
latched part. Implausible bursts (all 0x00 or all 0xFF) are rejected outright —
gravity makes an all-zero accelerometer impossible.

**Data path order is fixed:** decode → sensor mount → subtract bias → balance
frame (only while balancing) → EKF. The mount must precede the EKF or the
antiparallel IMUs read 20° apart. Bias is subtracted in the body frame, so a later
frame change never has to rotate a bias state — `imu_set_frame()` just re-seeds
the filters from the last accelerometer sample.

**Measured dt, not the nominal period.** Correct after an overrun, correct when a
read fails for a few ticks (the next good read integrates across the gap), and
correct when `OMNIS_IMU_READ_ALTERNATE` halves each IMU's rate. Clamped to 50 ms so
a long stall cannot integrate a wild angle in one step.

**Calibration gates on stillness, and fails rather than guessing.** 2 s of
samples per attempt, up to 5 attempts. Rejected if gyro std-dev exceeds 0.02 rad/s
(~20× the noise floor), if |a| std-dev exceeds 0.02 g, or if mean |a| is not within
0.1 g of 1 g — that last one is not a motion problem but a wrong full-scale setting
or failing part. A bias measured while someone was holding the robot would bake a
drift into every later angle.

**Mount wizard instead of arrow-reading.** `OMNIS_RUN_MOUNT_WIZARD 1` boots into a
timed two-pose procedure that runs `imu_mount_resolve()` on raw sensor data and
prints `#define` lines to paste, flagging any that differ from the header. It
matters more on Rev 2.0: the IMU placement is not confirmed on the new board.

**Escape hatch for I2C cost.** Two 1+14-byte transactions at 400 kHz are
estimated at a few hundred µs plus driver overhead, but the v6 `i2c_master` driver's
real per-transaction cost is unmeasured. `OMNIS_IMU_READ_ALTERNATE` reads one IMU
per tick, halving it, with no other change needed because every consumer uses
measured dt.

### Known limitations

- **I2C time on the real bus is unmeasured.** The status line reports the worst
  tick (`i2c NNN us`) so the first bench run settles it.
- **Stage 3 halts on IMU init or calibration failure.** Stage 6 turns both into
  latched faults that block arming while the radio and buzzer keep working.
- The drive pipeline still runs on synthetic sticks; nothing drives a motor yet.
- The IMU mounting constants are still those confirmed on the Rev 1 board photo.
  Rev 2.0 placement must be confirmed with the wizard.

---

## 2026-09-15 — Stage 4: CRSF parser, RC input decoding, UART glue

**Built:**

| File | What it does |
|---|---|
| `main/crsf_parser.{c,h}` | Pure byte-stream parser: framing, CRC-8/DVB-S2, 11-bit unpack, link stats |
| `main/rc_input.{c,h}` | Pure: channels -> sticks, arm request, drive mode, speed, pot |
| `main/crsf.{c,h}` | UART1 RX-only, zero-timeout polling, RX-pin auto-detect, link-loss logic |
| `test/test_crsf.c` + `test/crsf_golden.h` | 55 assertions; golden frames from an independent implementation |
| `main/omnis_params.*` | Channel map gains drive mode / speed / pot / arm invert; `rc` and `control` blocks |
| `main/main.c` | Radio started before IMU calibration and polled during it; synthetic sticks replaced |

**Result:** target build clean, zero warnings, 213 KB (80% free). Host tests:
**4 suites, 256 assertions, 0 failures.**

### Design decisions and why

**Golden frames come from a separate implementation.** `crsf_golden.h` is
generated by a Python implementation of the packing and CRC written independently
of the C, and the C must reproduce its bytes exactly. A test that compares a
function with itself proves nothing; two independent implementations agreeing on
a bug is far less likely. The all-centre frame it produced,
`c8 18 16 e0 03 1f … 7c ad`, is the canonical CRSF centre frame widely published
for ExpressLRS — a third, external confirmation. The CRC also passes the published
CRC-8/DVB-S2 check value (`"123456789"` → `0xBC`).

**RX-only UART, and that is what makes pin auto-detect safe.** The MVP sends no
telemetry, so the UART TX signal is never routed to a pin. If no valid frame
arrives on GPIO18 within 1.5 s, the receive signal moves to GPIO17 and back until
frames appear. Both pins are inputs the whole time, so a crossed harness can never
put the ESP32's output against the receiver's. Rev 1.0's receiver-perspective net
names are exactly the mistake this rescues.

**Resync drops one byte, never a buffer.** A truncated frame's header claims bytes
that are actually the start of the next, perfectly good frame. Discarding the
buffer on the CRC failure would throw that frame away and cost a whole frame
interval every time the line glitches. Tested explicitly: a 10-byte truncated
frame followed by a good one recovers the good one.

**Link loss produces a NEUTRAL command, never a held one.** §7f: holding the last
value on link loss is wrong for a robot. The link is OK only while RC frames arrive
within 250 ms; otherwise `rc_command_neutral()` — centred sticks, no arm request.
An optional uplink-LQ floor exists but defaults off: the frame timeout is the
primary detector, and a floor set too high disarms on a fringe link for no benefit.

**The arm switch needs a decisive position.** HIGH (> +0.5) arms, LOW or MID
never does, whichever way `arm_switch_invert` is set. A 3-position switch mapped to
the arm channel by mistake therefore cannot arm the robot from its centre detent.

**Rescaled deadzone.** Inside ±4% the stick reads exactly 0; outside it ramps
from 0, so there is no step when the stick leaves the band — a plain "zero it if
small" deadzone makes the robot lurch at the edge.

**Arming will require every stick within 10% of centre** (enforced in Stage 6).
The Pocket's throttle stick does not spring-centre, and this mapping puts
throttle-centre at zero speed: a throttle left at the bottom is full reverse. The
tolerance is what stops the robot arming straight into it.

**Bounded, non-blocking polling.** At most 4 × 128 bytes are parsed per tick with
`uart_read_bytes(..., 0)`; the 1 KB ring buffer holds ~40 frames, so even a long
stall cannot overflow it. `uart_set_rx_timeout(3)` hands bytes over after three
idle symbols, so a frame reaches the parser well inside one tick.

**The radio starts before IMU calibration**, and is polled from the calibration
loop's per-tick hook, so it has found its pin and locked by the time the robot is
ready.

### Known limitations and things to check on the bench

- **Channel order.** EdgeTX defaults a new model to AETR (ch1 roll, ch2 pitch, ch3
  throttle, ch4 yaw); §9f's map is throttle-first. Either reorder the mixer on the
  radio or change the four numbers in `omnis_params.c`. The status line prints
  throttle/roll/yaw so the check takes a minute.
- **Stick polarity** is still unverified; `invert_*` flags are all false.
- No telemetry is sent back to the radio (battery sensing is out of scope anyway).
- Until the radio is powered, the RX pin probe keeps swapping every 1.5 s. Harmless —
  both pins are inputs — and it stops at the first valid frame.
- Nothing drives a motor yet: `sol.rates` is computed from real sticks and discarded
  until Stage 5.

---

## 2026-09-15 — Stage 5: RMT step generation, acceleration limit, bench pattern

**Built:**

| File | What it does |
|---|---|
| `main/step_wave.{c,h}` | Pure STEP waveform generator: rate in, RMT-style symbols out |
| `main/step_gen.{c,h}` | Four RMT TX channels, burst re-arm, exact queue regulation, DIR state machine |
| `main/drive.{c,h}` | + `drive_slew_rates()`: acceleration limit by a common factor |
| `main/omnis_params.*` | `motor` block: per-wheel DIR invert, pulse width, minimum low time |
| `main/omnis_config.h` | `OMNIS_BENCH_STEP_TEST` — fixed motion pattern, no radio, wheels off the ground |
| `main/main.c` | Step generation initialised straight after parameters; bench-gated drive path |
| `sdkconfig.defaults` | RMT done-callback and copy encoder in IRAM |
| `test/test_step_wave.c` | 87 assertions on the reconstructed waveform |
| `test/test_kinematics.c` | + 13 slew-limiter assertions (89 total) |

**Result:** normal build clean, zero warnings, 236 KB; bench variant
(`OMNIS_BENCH_STEP_TEST 1`) also clean, 240 KB. Every sdkconfig option confirmed
in the regenerated file. Host tests: **5 suites, 356 assertions, 0 failures.**
In a normal build the motors still never step — the supervisor that arms them is
Stage 6/7.

### Design decisions and why

**Chunks, queued with EXACT regulation.** Each tick, each wheel keeps three
~2 ms bursts queued. "Pending" is not estimated from timing: it is `submitted`
(superloop) minus `completed` (incremented in the RMT done-callback, the module's
only ISR). So the generator can never over-fill the driver's queue, and a late
tick is absorbed by the ~6 ms lead and topped up on the next one.

**Payload buffers outlive their transactions.** `rmt_transmit()` does not copy:
the copy encoder reads the buffer from the ISR while it plays. Each wheel owns a
ring of 8 buffers against a queue depth of 6, so a slot is never rewritten while
queued. A failed submit does not advance the ring or commit the waveform phase —
that chunk is simply regenerated next tick. One encoder per channel, because an
encoder holds per-transaction state.

**Stopping overwrites queued payloads instead of calling `rmt_disable()`.** Read
from the installed IDF's `rmt_tx.c`: `rmt_tx_disable()` recycles only the
transaction in progress, without a done callback, and leaves everything else
queued to play after the next enable — steps after a stop, plus a pending count
that never drains. Instead `step_gen_stop()` clears the level bits of every queued
burst in place. The ISR only ever reads those words, so the rewrite is race-free;
queued bursts play as silence and the counters stay consistent. At most the burst
already copied into RMT memory — one tick — still plays.

**Direction reversal drains before touching DIR.** Queued bursts were generated
for the old direction; flipping DIR while they play sends those steps the wrong
way. A reversal stops queueing, waits for the queue to empty, writes DIR, holds a
full tick (2 ms against the A4988's 200 ns setup), then resumes.

**Waveform guarantees, each tested on a reconstructed timeline across chunk
boundaries:** a pulse is never split and every chunk ends LOW (the output idles
low between transactions, so a split pulse becomes two steps and a high-ended
chunk can merge two into one); every pulse is exactly 10 µs and every gap at least
10 µs, including through 20 s of random rate changes; no zero-duration halves
(RMT reads zero as end-of-transmission).

**The rounding residue is carried.** Edges land on whole microseconds, but the
error is fed back into the phase. Verified over 10 s at 3395.305 steps/s (Case A):
within 1.5 pulses of exact. Rounding each edge independently would have lost ~55
steps.

**Phase is a fraction of the period.** Speeding up from a crawl does not wait out
the old period: after one chunk at 20 steps/s, the first pulse at 7000 steps/s
came at **137 µs**, not ~48 ms later.

**Acceleration is limited by a common factor.** Slewing each wheel separately
lets small changes arrive before large ones, warping the motion vector mid-change
exactly as per-wheel clipping does. With one factor, every intermediate command
lies on the straight line between old and new motion. Verified: mid-transition
`vy/vx` and `w/vx` hold the target ratios to 1e-4 and the null space stays empty.
A non-positive limit freezes the wheels rather than meaning "unlimited".

### Bug found by the tests

**At high rates the symbol buffer, not the clock, ended each chunk.** The first
waveform run failed 2 of 77 checks: at 50 000 steps/s a 64-symbol chunk lasted
~1.26 ms instead of 2 ms. Every pulse guarantee still held and the average rate was
still exact — but short chunks quietly erode the queue lead the generator relies
on, and nothing stopped the firmware being configured into that regime. Added
`step_wave_max_full_chunk_rate()` (30 000 steps/s for 64 symbols), made
`step_gen_init()` refuse a `max_step_rate` above it, and corrected the test to
demand full-length chunks only below that limit while still checking every other
guarantee above it. The configured 7000 steps/s is far inside.

### A mistake during verification, recorded so it is not repeated

Checking that the bench variant compiles meant flipping `OMNIS_BENCH_STEP_TEST` to
1, building into a scratch directory, and restoring the header. In zsh, `status` is
a read-only special variable: `status=$?` aborted the rest of the command line, so
**the restore never ran and the header was left enabling the bench pattern.** It
was caught on the next step, before any commit or flash, and restored and verified.
Two lessons: never name a shell variable `status` here, and a restore must not sit
behind commands that can fail.

### Known limitations

- **RMT inter-transaction gap is unmeasured.** Between bursts the output idles low
  for the ISR's hand-over latency, probably tens of µs per 2 ms burst — roughly a
  1% lower effective rate. Harmless to pulse integrity; measure with a logic
  analyser.
- **Reversal dead time is up to ~8 ms** (drain + settle). Irrelevant for driving;
  it matters for balancing, where wheel speed crosses zero often.
- **`dir_invert` defaults (right side inverted) are unverified.** The bench
  pattern's FORWARD segment settles them wheel by wheel.
- `max_accel_steps_s2 = 20000` remains a placeholder until the A4988 current limits
  are set.

---

## 2026-09-15 — Stage 6: fault latch, buzzer, arming supervisor, integration

**Built:**

| File | What it does |
|---|---|
| `main/fault.{c,h}` | Pure fault latch with three clearing classes |
| `main/buzzer_pattern.{c,h}` | Pure patterns + event-over-fault priority player |
| `main/buzzer.{c,h}` | GPIO16 glue; writes the pin only on a level change |
| `main/supervisor.{c,h}` | Pure arming state machine — the one place that decides whether STEP pulses may exist |
| `main/main.c` | Rewritten cleanly: sense → judge → act → report, faults instead of halts |
| `main/omnis_params.*` | `safety` block: settle time, upright/flat thresholds, overrun limits |
| `test/test_supervisor.c` | 79 assertions |

**Result:** normal build clean, zero warnings, 242 KB; bench variant also clean.
Host tests for this commit: **6 suites, 435 assertions, 0 failures** (the Stage 7
balance suite, 28 more, passes too and ships next).

**No-RTOS compliance, checked mechanically rather than asserted.** Two scans,
both empty: (1) source grep of `main/` for any FreeRTOS call, handle type or
`freertos/` include; (2) the undefined symbols of the compiled `libmain.a` — which
catches anything reaching FreeRTOS through a macro or header — contain no
`xTask*`, `vTask*`, `xQueue*`, `xSemaphore*`, `xEventGroup*`, `xTimer*` or port
symbols. What the firmware does import is ESP-IDF driver API (`rmt_*`, `uart_*`,
`i2c_*`, `gptimer_*`, `gpio_*`, `esp_timer_*`), exactly the line drawn in PLAN.md §1a.

### Design decisions and why

**On board Rev 2.0 the supervisor is the entire safety story.** EN# is hardwired,
so nothing else can keep a live driver from moving a wheel. Motors step only in
`ARMED_FLAT` or `ARMED_BALANCE`; every other state calls `step_gen_stop()` every
tick.

**Three fault clearing classes, chosen by what the fault implies.** FOREVER
(parameters, IMU init/calibration/comm/disagreement, step-generator init): the
robot's own sensing or setup cannot be trusted and nothing done from the radio
fixes that — reboot. UNTIL-DISARM (tilt, overrun): the hardware is fine, the
situation was not; cleared once disarmed with the switch LOW. LIVE (radio link):
present while down. Any active fault of any class blocks arming and forces a
disarm.

**Arming needs a LOW→HIGH edge, and the attempt consumes it, pass or fail.** So a
switch left HIGH at power-on can never arm, a rejected arm needs a deliberate
re-try, and — tested explicitly — the robot does not re-arm itself when a fault
clears or the link returns with the switch still up.

**Every arm check has its own rejection reason**, printed and buzzed: fault, no
link, IMU not ready, sticks not centred, pose does not match the mode, balance gains
zero. The first failing check is the one reported, ordered from "hardware is
broken" to "you asked for the wrong thing".

**Mode is latched at arming.** Moving the mode switch while armed does nothing —
changing between flat and balance mid-motion is a crash. AUTO picks balance if the
robot is tipped past 60°, flat otherwise.

**Balance is refused explicitly until Stage 7 wires the controller**
(`balance_gains_set = false`), rather than arming into a mode that does nothing.

**Boot never halts on a peripheral.** A missing IMU, a failed calibration or an
RMT failure latches a fault; the loop, buzzer and telemetry keep running so the
problem is reported. Only invalid parameters or a tick that will not start drop to
`fault_only_loop()`, because everything downstream divides by the parameters.

**The kinematics self-check now uses the fixed REFERENCE geometry**, not the
parameter set. It verifies the code and the FPU; editing geometry cannot fake a
parameter fault.

**Buzzer patterns are tested to be pairwise distinguishable** — the test compares
every pattern's timeline against every other's — and a one-shot event always plays
to completion over a fault pattern, which then restarts from its beginning so it is
heard whole.

**A dead UART must not become a log flood.** If `crsf_init()` fails, the loop never
calls `crsf_poll()`: `uart_read_bytes()` on an uninstalled driver logs an error per
call, which at 500 Hz would itself wreck the loop timing. The robot simply never
has a link, and cannot arm.

**Overrun fault, windowed, with the telemetry line excluded.** 25 missed deadlines
in a 500-tick window faults an armed robot: past 5% the loop's dt, and so the EKF
and PID, can no longer be trusted. The ticks after a log line are not counted — a
telemetry line blocks for a few ms and must not be able to trip a fault by itself.

**Link loss is a fault only once there has been a link.** Before the radio is ever
seen, the robot just cannot arm (NO_LINK); after, losing it disarms and double-chirps.

### Known limitations

- The overrun window is a fixed block, so 24 overruns either side of a boundary go
  uncounted. Adequate for catching a loop that is genuinely too slow.
- Disarming stops the wheels immediately rather than ramping down. Correct for a kill
  switch; mechanically abrupt at speed.
- The motors hold current whenever 12 V is present — hardware, not firmware (EN#).
- No battery cutoff: battery sensing is out of MVP scope.

---

## 2026-09-16 — Stage 7: balance controller and full integration

**Built:**

| File | What it does |
|---|---|
| `main/balance.{c,h}` | Lean PID in the balance frame -> grounded-pair wheel rates |
| `main/main.c` | Frame switching on arm/disarm, balance path, balance tilt fault, disarm summary |
| `main/omnis_params.*` | `balance` block extended: accel clamp, stick mapping, tilt limit, invert, tuning range |
| `main/omnis_config.h` | `OMNIS_TELEMETRY_IN_BALANCE`, `OMNIS_TUNE_KP_FROM_POT` |
| `test/test_balance.c` | 28 assertions, including a closed-loop inverted-pendulum simulation |

**Result:** normal build clean, zero warnings, 244 KB (77% free). A build with all
three optional switches on (bench pattern, live kp tuning, telemetry in balance)
also compiles clean. Host tests: **7 suites, 463 assertions, 0 failures.**
No-RTOS compliance re-checked: 44 source files clean, and none of the 185 undefined
symbols in `libmain.a` is a FreeRTOS symbol.

### The wheel sign — and an error caught while deriving it

The kinematics' `+omega` is "the sense that drives the robot forward when flat".
What that does when the robot is standing on two wheels depends on where the ground
contact sits relative to the axle, and it is **not the same for the two pairs**.

For rolling without slip the centre moves opposite to the contact point's tangential
velocity, `omega x r`:

| Pose | ground at | `omega x r` | centre moves |
|---|---|---|---|
| flat | body −Z | `(−wR, 0, 0)` | +X — forward, by definition |
| front pair down | body **+X** | `(0, 0, −wR)` | +Z = **+X′**, toward the top face |
| rear pair down | body −X | `(0, 0, +wR)` | −Z = **−X′**, away from it |

So `sigma = +1` on the front pair and `−1` on the rear.

**My first derivation put the front-pair contact at body −X and concluded `+omega`
drove the robot backward on both pairs.** Nose-down 90° puts +X *down*, so the
contact is at +X. Caught by re-deriving from the rotation matrix before any of it
reached a file. The corrected sign is one factor, applied in one place, with the
whole derivation written out in `balance.h` — because if it is wrong on the real
robot, the symptom (wheels driving away from the fall) looks identical to a
controller sign error, and the two have completely different fixes.

Left/right flips too: with `+X′` on the top face and `+Z′` up, the robot's left is
`+Y′`, which is `+Y` on the front pair but `−Y` on the rear. Balancing on the rear
pair, the "left" wheel is **RR**.

### Evidence

`test_balance.c` runs a real closed loop: an inverted pendulum whose cart
acceleration comes from the commanded wheel rates through the rolling relation
above, written out a second time in the test independently of
`balance_pair_sign()`.

| Case | Result |
|---|---|
| Released at 3°, front pair | recovers to 0.00°, peak lean 3.00°, peak wheel 1522 steps/s |
| Released at 3°, rear pair | recovers to 0.00° — the sign flip is exercised |
| Released at −3° | recovers |
| `output_invert` on a correct robot | **falls** (>30°) — the sign genuinely matters |
| Zero gains | **falls** — which is why the supervisor refuses to arm them |

What this cannot prove is that the derivation matches the physical robot. That is
the bench sign check in TESTING.md §7.3, and it is why `output_invert` exists —
with an explicit warning that if only *one* pair is wrong, the flag is the wrong
fix.

### Design decisions and why

**Arming balance switches the estimation frame, then waits.** On the arming tick
the filters are re-seeded in the balance frame (their flat-frame angles mean nothing
there) and `ARMED-BAL-SETTLE` holds the wheels still for 400 ms — two EKF time
constants — before the controller drives.

**The balance tilt fault is gated on the frame having actually switched.** On the
arming tick the estimate is still flat-frame, where standing upright reads ~85°;
without that gate every balance arm would instantly fault with TILT.

**No slew limiter in the balance path.** The controller already limits acceleration
through its PID output clamp, and a second rate limit is just added lag, which
destabilises a balancer. The flat path keeps its slew limiter, where smoothness
matters and phase margin does not.

**The deadband still applies.** A wheel dithering across zero chatters its DIR line
(kinematics reference §6), and a balancing robot sits near zero speed constantly.

**Telemetry goes silent while balancing.** A blocking log line every second is a
periodic disturbance to the loop. One summary prints on disarm — time balanced,
peak lean, final speed — and `OMNIS_TELEMETRY_IN_BALANCE` restores the stream for
tuning.

**Live kp tuning uses `balance_set_kp()`, not `balance_init()`**, because re-initialising
mid-balance clears the integrators and the forward speed, which is a fall.

**The outer velocity-bias loop is wired but off** (`vel_bias_gain = 0`), matching
PLAN.md decision 1c. Enabling it is a parameter change, not a code change.

### A test assertion that was wrong

The first version asserted that full forward stick "commands forward acceleration",
and passed only because it tested for non-zero. The truth is the opposite and more
interesting: from upright, the base must first move **backward** for the robot to
tip forward — a controlled fall is non-minimum-phase. The assertion now checks
`last_accel < 0` and says why.

### Known limitations

- **Reversal dead time (~8 ms)** while queued pulses drain. Balancing in place
  reverses often; if the robot hunts around upright, suspect this first.
- **Mecanum crab:** two grounded mecanum wheels driven differentially produce yaw
  *and* some sideways force, so turning while balancing drifts laterally.
- **No encoders**, so the robot will drift across the room (§13a). The outer loop is
  the mitigation, and it is off by default.
- **Every balance gain is unknown** until the robot is on a tether.

---

## 2026-09-16 — Build complete: summary, and where to pick it up

All seven stages are built, host-tested and committed. **Stage 1 is the only
stage verified on hardware** (500 Hz, zero overruns, 2026-09-07, on the Rev 1
board); everything since was written against an unassembled Rev 2.0 board, so the
bench procedure in `TESTING.md` is the remaining work.

### Totals

| | |
|---|---|
| Firmware | 44 files in `main/`, ~244 KB binary, 77% of the app partition free |
| Host tests | 7 suites, **463 assertions**, plus 97 in `assets/control/` |
| Builds verified | normal, bench-pattern, and all-optional-switches-on — each with **zero warnings** |
| No-RTOS rule | verified mechanically: no FreeRTOS call, handle or include in 44 sources, and none of `libmain.a`'s 185 undefined symbols |

### Bugs found, and what found them

Worth recording because the pattern is informative: **the tests and the
derivations caught more than the compiler did.**

| # | Bug | Found by | Would have looked like |
|---|---|---|---|
| 1 | `pid_t` collides with POSIX `sys/types.h` | compiling the asset on the target toolchain before porting | a hard compile error at Stage 7, blamed on the port |
| 2 | Flat-frame pitch **folds** at the balance point: 85° and 95° both read 85° | re-deriving §7 while designing the balance frame | a balancer that pushes the wrong way on one side of upright, blamed on gains |
| 3 | Chunks ended early above ~30 000 steps/s (symbol buffer, not the clock) | `test_step_wave.c` on the first run | queue lead silently eroding at high speed |
| 4 | Telemetry rate biased high (two off-by-ones) | the Stage 1 **hardware** run reading 500.13 Hz | a "healthy" loop rate that hides degradation |
| 5 | Balance wheel sign: front-pair contact is at body **+X**, not −X | re-deriving the rolling relation before writing it down | wheels driving away from the fall on one pair only |
| 6 | Test asserted forward stick commands forward acceleration | reading the assertion's own output | a tautological test passing forever |

Two process mishaps, both caught before any commit: zsh's read-only `$status`
aborted a command line and left the bench flag enabled in a working file; and two
parallel shell calls shared one working directory, so a build ran in the wrong
place and a "clean" compliance scan had actually scanned nothing.

### Where to pick this up

1. **Set the A4988 current limits (Vref) before connecting 12 V.** `EN#` is
   hardwired on Rev 2.0 — the motors energise the instant power is applied.
2. Work through `TESTING.md` in order. Stages 3–6 need only USB and a radio;
   Stage 5 onward needs the wheels off the ground.
3. The four things most likely to need changing, all parameters, none code:
   IMU mount constants (Stage 3.3 wizard), motor `dir_invert` flags (Stage 5.1),
   radio channel map and stick polarity (Stage 4.1), balance gains (Stage 7.4).
4. `BUILD-LOG.md` (this file) explains why each decision was made; `PLAN.md` has
   the scope and the settled decisions; `omnis-info.md` here is the project info
   file annotated for this build.

### What this build deliberately leaves for the FreeRTOS version

OLED and buttons, microSD and `params.json`, logging, OTA and Wi-Fi, battery
sensing — and the custom RTOS layer itself. Every pure module here (kinematics,
EKF, fusion, PID, balance, CRSF parser, fault/supervisor logic, waveform
generator) has no ESP-IDF dependency and carries over unchanged; only the glue
(`main.c`, `imu.c`, `crsf.c`, `step_gen.c`, `buzzer.c`) is superloop-shaped.

---

## 2026-09-16 — Wheel layout change: kinematics re-derived for parallel rollers

The wheels turned out **not** to be in the X-drive arrangement every document in
this repository had assumed. Mounting-shaft and motor constraints put the same
roller tilt on **both left wheels** and the mirrored tilt on **both right
wheels**:

```
        FL  /                 \  FR          left  pair  FL, RL  ->  δ = -1
        RL  /                 \  RR          right pair  FR, RR  ->  δ = +1
```

### Re-derived, not sign-flipped

The temptation with a change like this is to flip signs until the bench test
passes. That was rejected: the `vy` signs and the yaw lever arms both descend
from the same `δ_i`, so patching one column by hand produces a matrix that is
not the kinematics of any physical robot, and the error only shows up later as
drift. The derivation was redone from the rigid-body contact-patch velocity
onward.

The general per-wheel constraint survived untouched:

```
r*ω_i  =  vx  +  δ_i*vy  +  w*(δ_i*x_i − y_i)
```

Everything specific to the robot is now **which four signs go into δ**, and the
firmware mirrors that: `MECANUM_DELTA_FL` … `MECANUM_DELTA_RR` are the only
statement of the layout, and both lever arms are computed from them by
`mecanum_yaw_levers()`. A mirrored build is a four-line change; a half-applied
mirror is not expressible.

### What actually moved

| | Was (X-drive) | Now |
|---|---|---|
| Shared tilt | FL+RR, FR+RL | **FL+RL, FR+RR** |
| Yaw lever | `±k` on all four | **`±k` front, `±m` rear**, `k=(L+W)/2`, `m=(L−W)/2` |
| Rear IK rows | `vx + vy ∓ k*w` | **`vx − vy ± m*w`** |
| Strafe pattern | diagonals | **sides**, equal magnitude |
| Bench idle test | `vx=vy` idles FL and RR | **idles the whole LEFT SIDE** |
| `MᵀM` | diagonal | **not diagonal** — `c_vy · c_w = 2W` |
| FK yaw scale | `r/(2(L+W))` | **`r/(2L)`** — track width drops out |
| FK structure | yaw ← sides, strafe ← diagonals | **yaw ← Δ_F − Δ_R, strafe ← weighted Δ_F + Δ_R** |

**Only the rear pair changed.** Both front-wheel equations are identical to the
previous edition, which was the fastest way to check the port had been applied
consistently rather than half-applied.

### Three results worth recording

**The layout is holonomic for every real chassis.** Working the pseudoinverse
through gives `det(MᵀM) = 4L²` — the track width cancels completely. There is no
aspect ratio, square included, at which this drivetrain loses a degree of
freedom. That matters because the rear yaw lever `(L−W)/2` is only −3.5 mm on
this frame and *looks* singular; it is not. The rear pair stops contributing to
yaw at `L = W` while still contributing to strafe, and rank is preserved.

**It costs nothing in performance and something in precision.** The peak wheel
rate per unit of commanded motion is identical to the X-drive on all three axes,
so the same step-rate ceiling buys the same top speed, strafe speed and yaw
rate — this was checked rather than assumed, and is now a test. What it costs is
odometry: the strafe and yaw columns meet at 44.1° instead of 90°, amplifying
per-wheel error by **1.44× on `vy` and 2.03× on `w`**.

**The null space did not move.** `(+1, +1, −1, −1)` still annihilates all three
motion columns — the solve uses only `k ≠ m`, i.e. `W ≠ 0`, so it holds for
either handedness and any geometry. `mecanum_null_space()` needed no edit at all
and the health metric keeps its meaning. Of everything here that was the one
pleasant surprise.

### Verification

- The closed-form pseudoinverse was checked against `(MᵀM)⁻¹Mᵀ` computed in
  **exact rational arithmetic**, for both handedness signs and four geometries
  (223×230, 300×200, 250×250 square, 100×400). Every coefficient matched
  exactly, with no floating point in the comparison. Doing this before writing
  the closed form into the code and three documents was the right order — the
  hand algebra had already produced one wrong intermediate (`2(k+m)` where the
  dot product is `2(k−m)`), which the check would have caught had the prose
  survived that far.
- `test_kinematics.c` gained a layout-properties section: per-axis round trips
  (a rank-deficient layout fails only one axis, so a combined test can mask it),
  the lever identities `κ_F + κ_R = −W` and `κ_F − κ_R = −L`, the 64.7:1 ratio,
  the square-frame zero and the sign flip at `L > W`, the non-orthogonality
  `c_vy · c_w = 2W = 460`, and the unchanged null space asserted against the
  columns themselves rather than against commands the IK happened to produce.
- The boot self-check now asserts the rear pair at ±59.418 steps/s for a 1 rad/s
  spin. **An X-drive build would show ±3845 on all four**, so that one line
  catches a stale port on the hardware.
- Both diagonals are now bench-tested. `vx = vy` idles the left side and
  `vx = −vy` idles the right; running *both* is what separates "handedness
  correct" from "handedness mirrored", since a mirrored build passes one and
  fails the other.

### The convention problem, stated honestly

A roller glyph `/` or `\` can mean the roller's **axis** or its **rolling
direction**, and the two readings are mirror images. Picking one consistently is
not the same as being right about the hardware, and the difference is not
academic — it flips which way the robot strafes.

The derivation now fixes the convention explicitly in §2 (the glyph is the
**axis**, seen from above, robot facing up the page, body +y to the page-left)
and §5.3 carries the bench test that settles it on the physical robot, along
with the exact four-constant fix. This is deliberately *not* left as a judgement
call in prose: the code has one place to change and the test has a
pass/fail answer.

### Documents rewritten

All three kinematics documents were rewritten rather than patched —
`derivation.md` (second edition, re-derived from §4 onward, with a new §9.3 on
conditioning and a §13 reviewer's checklist of what moved), `reference.md`, and
`code-explained.md` (whose verification transcript is now real output from the
shipped source). `TESTING.md` Stages 5.1 and 7.1, both copies of `omnis-info.md`,
`omnis_imu_mounting.h` (the IMUs sit on the FL/RR diagonal, which *was* a
handedness pair and is not one now), the root `README.md` and `PLAN.md`'s
corner-labelling note were all brought into line.

`OPERATIONS_GUIDE.md` was added in the same pass: build and flash, the superloop
walkthrough, a file-by-file index of all 44 sources, and a tuning cheat sheet
whose §4.2 is the "forward is right but strafe is backwards" procedure.
