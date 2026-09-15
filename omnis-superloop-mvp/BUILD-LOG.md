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
