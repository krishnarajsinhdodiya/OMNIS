# OMNIS Superloop MVP — Bench Bring-Up and Testing

A step-by-step procedure for taking the firmware from a bare board to a
balancing robot, written to be followed without help. Each stage has what to
set up, what to do, what a pass looks like, and what the common failures mean.

**Do the stages in order.** Each one proves something the next one depends on.
Stage 1 was verified on hardware on 2026-09-07 (Rev 1 board); everything from
Stage 3 on has been built and host-tested but not yet run on a board.

---

## 0. Read this first — safety

**Board Rev 2.0 has no motor enable line.** All four A4988 `EN#` pins are
hardwired to GND. The consequences are not optional reading:

- **The motors are energised the moment 12 V is connected**, before the firmware
  even boots. They hold position and get warm. The only way to de-energise them is
  to **unplug the battery**. Keep the connector within reach.
- The firmware's "stop" means **no more STEP pulses**. Motors hold; they do not go
  limp. Nothing in software can make a stalled, energised motor safe — only
  unplugging can.
- **Set the A4988 current limits (Vref) before ever connecting 12 V.** With
  `EN#` hardwired, a wrong current limit starts cooking a motor or driver the
  instant power is applied. Set `Vref = Imax × 8 × Rsense` for the KV4239-T3B004's
  rated current, with margin (omnis-info.md §3f, §13c).
- **Wheels off the ground** for Stages 5 and 6. Chassis on a box, wheels free.
- **First driving tests on speed LOW** (speed switch down), in an open space.
- **Balance tests on a tether** or over something soft (Stage 7).

---

## 1. Toolchain, build, flash

Activate ESP-IDF v6.0.2. On this machine `export.sh` fails (it looks for a
py3.11 venv, the install is py3.14), so use the vendor script:

```bash
. ~/.espressif/tools/activate_idf_v6.0.2.sh
```

Build, flash and open the monitor (find the port with `ls /dev/cu.* | grep -v Bluetooth`):

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp -p /dev/cu.usbmodem101 flash monitor
```

The console runs at **460800** baud; `idf.py monitor` picks that up automatically.
Exit the monitor with **Ctrl-]**. If the monitor shows garbage, set
`CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200` in `sdkconfig.defaults`, delete
`omnis-superloop-mvp/sdkconfig`, and rebuild.

**After editing `sdkconfig.defaults`, always delete `omnis-superloop-mvp/sdkconfig`**
or the change is silently ignored.

Build-time switches live in `main/omnis_config.h`; tunable numbers live in
`main/omnis_params.c`. Every edit to either needs a rebuild and reflash.

---

## 2. Host tests — no hardware at all

Run these after any code change, before flashing anything:

```bash
cd /Volumes/Projects/OMNIS/omnis-superloop-mvp && ./test/run_host_tests.sh
```

It builds and runs every hardware-independent suite with plain `cc` and ends with
`ALL N HOST TEST SUITES PASSED`. Any failure exits non-zero. The expected values in
those suites came from independent references (the kinematics reference, a separate
Python CRSF implementation, published CRC check values) — if a test fails after a
change, the test is almost certainly right.

The control modules also have their own suite in `assets/control/`
(`./run_host_tests.sh` there).

---

## Stage 1 — Timing spine

**Setup:** USB only. **Battery disconnected.**

**Do:** flash and monitor. Leave it running 2–3 minutes.

**Pass:**

```
I (xx) omnis: kinematics self-check (cases A-D, round trip, null space, side idle): PASS
I (xx) tick: 500 Hz tick running (period 2000 us)
```

and a status line every second beginning like:

```
500.0Hz body NNNus ovr 0 | ...
```

| Field | Must be |
|---|---|
| `500.0Hz` | 500.0 ± 0.1 |
| `ovr` | **0**, sustained — the single most important number in the build |
| `body` | loop body time in µs; must stay well under 2000 |

Also confirm `main_task: Started on CPU1` near the top of the boot log, and no
`Task watchdog got triggered` message, ever.

| Failure | Meaning |
|---|---|
| `Task watchdog got triggered ... IDLE1` | CPU1 idle-watchdog setting did not take — delete `sdkconfig`, rebuild |
| `ovr` steadily non-zero | the loop body is too slow — see Stage 3's I²C note |
| `HALTED: parameter set failed validation` | an edit to `omnis_params.c` broke a range check; the message is repeated every 2 s |

---

## Stage 2 — Kinematics self-check

Nothing to set up — this is the `kinematics self-check ... PASS` line in the Stage 1
boot log. It re-runs the four verified kinematics cases on the ESP32's FPU. A `FAIL`
drops the firmware into a fault loop with the config buzzer pattern.

---

## Stage 3 — IMUs

**Setup:** USB only, battery disconnected. Both GY-521 modules fitted. **Robot
flat and completely still on the bench at power-on** — calibration runs in the
first ~2 s.

### 3.1 Boot

**Pass:**

```
I (xx) mpu6050: 0x68: configured (WHO_AM_I 0x68, +-500 dps, +-4 g, DLPF 94 Hz, 500 Hz, INT open-drain)
I (xx) mpu6050: 0x69: configured (...)
I (xx) imu: IMU A (0x68) ready, mount {+1,+2,+3}
I (xx) imu: IMU B (0x69) ready, mount {-1,-2,+3}
I (xx) imu: gyro calibration 1/5: keep the robot STILL for 2.0 s
I (xx) imu:   IMU A: bias (...) rad/s | gyro sd 0.00NN | |a| 1.00N g sd 0.00NN | NN.N C
I (xx) imu: calibration done
```

A WHO_AM_I warning like `0x70 - not a genuine MPU6050` is **normal** for GY-521
clones (MPU6500 family) and harmless.

During calibration the buzzer ticks softly once a second.

| Failure | Meaning / fix |
|---|---|
| `0x68: no ACK` / `0x69: no ACK` | wiring, power, or AD0 level (A must be GND, B must be 3V3) |
| `read back 0xNN, wrote 0xNN` | a second device is answering, or the bus is marginal — try `OMNIS_I2C_SCL_HZ 100000` |
| `gyro not still` repeated | the robot moved, or vibration (fans, a nearby motor). Calibration retries 5 times, then latches `IMU_CAL` |
| `|a| = 0.NNN g at rest, expected 1.000` | wrong accelerometer range or a failing part |

### 3.2 Attitude signs

Watch the `imu r... p...` fields of the status line:

| Action | Expect |
|---|---|
| Flat and still | `r` and `p` within ±2°, `dis` under 2° |
| Tip **nose down** (OLED end lower) | `p` goes **positive** |
| Tip **left side down** | `r` goes **negative** |
| Tip **right side down** | `r` goes **positive** |

`dis` is the angle between the two IMUs' estimates of "down". It must stay small in
every pose. **Positive pitch = nose down** is intended (it is how the `atan2` model
works) — do not "fix" it.

### 3.3 IMU mounting — if `dis` is large, or before trusting Stage 7

The mounting constants still come from the Rev 1 board photo. On Rev 2.0, confirm
them with the wizard:

1. In `main/omnis_config.h` set `OMNIS_RUN_MOUNT_WIZARD 1`, rebuild, flash, monitor.
2. Follow the prompts: **pose 1** chassis flat and level; **pose 2** front (OLED)
   edge lowered 30–60° and held. Each pose has an 8 s countdown.
3. It prints one `#define OMNIS_IMU_x_MOUNT` line per IMU, marked
   `matches the current header` or `DIFFERS: update the header`.
4. Paste any differing line into `main/omnis_imu_mounting.h` **and**
   `assets/control/omnis_imu_mounting.h` (keep them identical), set the wizard back
   to 0, rebuild, and re-check 3.2.

A large `dis` with correct mounts is a loose module, a bad cable, or a failing
sensor — that is exactly what the `IMU_DISAGREE` fault exists to catch.

### 3.4 I²C timing

The status line's `i2c NNNus` is the worst tick's I²C time. If `ovr` climbs or
`body` approaches 1600 µs, set `OMNIS_IMU_READ_ALTERNATE 1` (one IMU per tick). As a
last resort drop `OMNIS_TICK_RATE_HZ` to 250. Everything uses measured `dt`, so
nothing else needs changing.

---

## Stage 4 — Radio

**Setup:** USB only, battery disconnected. ExpressLRS receiver powered and bound to
the RadioMaster Pocket, radio on.

**Pass:**

```
I (xx) crsf: listening at 420000 baud on GPIO18 (RX-only); will also probe GPIO17
I (xx) crsf: CRSF locked on GPIO18
```

and the status line shows `rc OK LQ100` (link quality under 100 is fine; it varies).

`CRSF locked on GPIO17` means the receiver's Tx is wired to GPIO17. It works — both
pins are inputs — but the harness does not match the schematic.

### 4.1 Channel order and stick polarity

Move one stick at a time and watch `thr pit rol yaw`:

| Stick | Expect |
|---|---|
| Throttle **up** | `thr` → +1 |
| Right stick **forward** | `pit` → +1 |
| Right stick **left** | `rol` → **+1** |
| Left stick (rudder) **left** | `yaw` → **+1** |

- **The wrong field moves** (e.g. throttle moves `rol`): EdgeTX defaults to AETR
  order (ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw). Either reorder the mixer on
  the radio, or change `p->channel_map.*` in `omnis_params.c` (1-based).
- **Right field, wrong sign:** set that stick's `p->rc_scale.invert_*` flag in
  `omnis_params.c`. Never flip a sign anywhere else.
- Centred sticks must read `+0.00` (a ±4% deadzone absorbs small centring error).

### 4.2 Switches

| Switch | Field | Expect |
|---|---|---|
| Arm switch (ch 6) | `arm` | `arm1` only in its **up** position. If up reads `arm0`, set `arm_switch_invert = true` |
| Drive-mode 3-pos (ch 8) | `mode` | `LOW` / `MID` / `HIGH` |
| Speed 3-pos (ch 9) | `spd` | `LOW` / `MID` / `HIGH` |

**The Pocket's throttle stick does not spring-centre.** Arming requires every stick
within 10% of centre, so the throttle must be held at its middle to arm.

### 4.3 Link loss

Switch the radio off. Within 250 ms the line shows `rc LOST` and `faults RC_LINK`,
and the buzzer double-chirps. Radio back on: `rc OK`, the fault clears, chirping
stops. (Before the radio is first seen, the robot is simply silent and cannot arm.)

---

## Stage 5 — Steppers, on the bench pattern

**Setup:** **wheels off the ground.** A4988 current limits set (§0). 12 V connected.
USB connected.

1. In `main/omnis_config.h` set `OMNIS_BENCH_STEP_TEST 1`. Rebuild, flash.
2. The boot log warns `WHEELS WILL TURN ~3 s AFTER BOOT`. The radio is ignored.
3. The pattern cycles, logging each segment:

| Segment | Every wheel should… |
|---|---|
| FORWARD 200 mm/s | **all four roll forward** — the top of each wheel moves toward the OLED end |
| BACKWARD 200 mm/s | all four roll backward |
| STRAFE LEFT | **both left wheels backward; both right wheels forward**, all four at the same speed |
| ROTATE CCW | both left wheels backward, both right forward — but **the front pair spins fast (~3845 Hz) and the rear pair only crawls (~59 Hz)** |
| DIAGONAL vx=vy | FR and RR forward; **FL and RL completely still** |
| ANTI-DIAGONAL vx=−vy | FL and RL forward; **FR and RR completely still** |

> **The rear pair crawling during ROTATE is correct**, not a stuck motor. Under
> the lateral parallel roller layout the rear wheels' yaw lever is 1.5% of the
> front's, so a spin is absorbed almost entirely by the rear rollers. If all
> four wheels spin equally fast, the build is running **stale X-drive
> kinematics** — check that `main/mecanum_kinematics.h` has
> `MECANUM_DELTA_RL == MECANUM_DELTA_FL`.

### 5.1 Fixing directions

Work in this order. Forward motion does not involve roller handedness at all,
so it must be right before any of the other rows mean anything.

- **A wheel runs backward in FORWARD:** flip that wheel's `p->motor.dir_invert[i]`
  in `omnis_params.c` (order `[FL, FR, RL, RR]`). Never fix it in the kinematics.
- **The WRONG SIDE stands still in the two diagonal segments** — right side still
  on `vx=vy`, left side still on `vx=−vy` — and FORWARD was correct: the roller
  handedness is mirrored from what the code assumes. Negate **all four**
  `MECANUM_DELTA_*` constants in `main/mecanum_kinematics.h` together and
  rebuild. Do not touch a `dir_invert` flag for this; see OPERATIONS_GUIDE §4.2.
- **A DIAGONAL pair stands still** (FL+RR or FR+RL) rather than a side: the
  wheels are not in the parallel layout at all, and the derivation does not
  describe this robot.
- **One wheel creeps during a DIAGONAL segment** while its partner is still: a
  DIR flag is wrong — fix FORWARD first.
- The status lines add `step: pulses N underruns 0 submit-fail 0`. **Both counters
  must stay 0.** Non-zero underruns mean the superloop is starving the step queue
  (check `ovr`).
- Optional, with a logic analyser: each STEP pin shows **~3395 Hz** during FORWARD,
  with 10 µs high pulses.

**Set `OMNIS_BENCH_STEP_TEST` back to 0 before going on.** The robot will never arm
while it is 1 — the bench pattern overrides the radio.

Strafe and rotate are only meaningful on the ground (Stage 7.1): on the bench
you can confirm wheel directions, not the resulting motion.

---

## Stage 6 — Arming, disarming, faults

**Setup:** **wheels off the ground**, 12 V, radio on, `OMNIS_BENCH_STEP_TEST 0`.
Mode switch `LOW` (flat), speed `LOW`.

### 6.1 Arming

1. Boot with the arm switch **up**. The robot must **not** arm — a switch left up at
   power-on never arms.
2. Arm switch **down**, then **up**, sticks centred. Buzzer: two short beeps;
   console `ARMED -> ARMED-FLAT`; status `ARMED-FLAT`.
3. Move the sticks: wheels respond, scaled by the speed switch.
4. Arm switch **down**: one long beep, `DISARMED`, wheels stop at once.

### 6.2 Each rejection (three fast beeps + a reason on the console)

| Try | Expected reason |
|---|---|
| Arm with a stick off-centre | `sticks not centred` — then centre it: still not armed until the switch is cycled |
| Arm with the radio off | `no radio link` |
| Mode switch `MID` (balance) while flat | `balance requested but robot not tipped up` |
| Mode `MID`, robot tipped onto a pair, balance gains still 0 | `balance gains are zero` |
| Mode `LOW`, robot tipped 40° | `flat requested but robot is tipped` |

### 6.3 Disarm conditions

| While armed, do | Expect |
|---|---|
| Switch the radio off | wheels stop; `DISARMED`; double chirp. Radio back on with the switch still up: **stays disarmed** until the switch is cycled |
| Lift and tip the robot past 45° | `FAULT raised - active: TILT`; slow half-second beeps; disarmed. Switch down to clear |

### 6.4 Buzzer reference

| Sound | Meaning |
|---|---|
| one short blip | boot OK |
| soft tick every second | calibrating — keep still |
| two short | armed |
| one long | disarmed |
| three fast | arm rejected — reason on the console |
| short then long | balance engaged |
| **continuous** | IMU fault (disagree / comm / init / calibration) — reboot needed |
| double chirp every 0.5 s | radio link lost |
| slow half-second beeps | tilt: fell over or tipped while armed |
| long-short-short | configuration fault (parameters or step generator) |
| triple chirp every second | control loop overrunning |

### 6.5 Fault reference

| Fault | Cleared by |
|---|---|
| `PARAMS`, `IMU_INIT`, `IMU_CAL`, `IMU_COMM`, `IMU_DISAGREE`, `STEP_INIT` | **reboot** — the robot's own sensing or setup cannot be trusted |
| `TILT`, `LOOP_OVERRUN` | disarming with the arm switch down |
| `RC_LINK` | the link returning (re-arming still needs a switch cycle) |

---

## Stage 7 — On the ground, then balancing

### 7.1 Flat driving

**Setup:** open floor, speed `LOW`, mode `LOW`.

Arm and check, gently:

| Input | Robot should |
|---|---|
| throttle forward | drive toward the OLED end |
| right stick left | **strafe** left, without turning |
| rudder left | **rotate** anticlockwise (seen from above) in place |
| right stick diagonal forward-left | move diagonally; **FL and RL** stay still |

- **Strafe goes right when it should go left** (and forward was correct): the
  roller handedness is mirrored from what the code assumes. Negate all four
  `MECANUM_DELTA_*` constants in `main/mecanum_kinematics.h` together and
  rebuild — a four-line change (reference §4, OPERATIONS_GUIDE §4.2). Swapping
  the physical wheels left-to-right is the other valid fix; do not do both.
- **Rotation feels weak or the robot crabs while turning:** expected to a
  degree. Only the front pair contributes meaningfully to yaw under this
  layout, so yaw torque is roughly half an X-drive's. It is a force effect, not
  a kinematics error — the commanded yaw *rate* is still exact.
- **Rotates the wrong way:** a stick polarity (`invert_yaw`) — Stage 4.1.
- Raise to speed `MID`, then `HIGH`, only once `LOW` behaves.

### 7.2 Balance — before anything else

Prerequisites, all of them: Stage 3.3 mounting confirmed, Stage 5 directions
correct, Stage 7.1 driving correct. A tether (a strap to an overhead point, slack
enough to balance, short enough to stop a fall) or a soft surface.

**Balance gains ship at zero, and the robot refuses to arm balance with zero gains.**
No numeric gains are provided — omnis-info.md §13d is right that they depend on mass,
CG height and motor torque, which the firmware does not know.

### 7.3 The sign check — do not skip

1. In `omnis_params.c` set a small `p->balance.kp`, e.g. `100000.0f`
   (units: steps/s² per radian). Leave `ki` and `kd` at 0. Rebuild, flash.
2. Tip the robot up onto its **front pair** (OLED end down) and hold it upright.
3. Mode switch `MID` (or `HIGH` for auto). Arm. Console:
   `ARMED -> ARMED-BAL-SETTLE on the FRONT pair: hold it upright for 400 ms`.
4. After the short-long beep (balance engaged), **gently tip the top of the robot
   toward its top face** (the OLED side).
5. **The wheels must roll the robot toward its top face — under the fall, catching
   it.** If they drive it away from the fall, the controller sign is wrong.
6. Repeat on the **rear pair**.

| Result | Action |
|---|---|
| Both pairs catch the fall | correct — go on to tuning |
| **Both** pairs push the wrong way | set `p->balance.output_invert = true` |
| **Only one** pair is wrong | **stop.** The wheel-sign derivation in `main/balance.h` does not match the hardware (or one pair's DIR flags are wrong). Do not use `output_invert` to hide it |

### 7.4 Tuning

Follow omnis-info.md §13d, on the tether:

1. **kp alone.** Raise until the robot oscillates steadily around upright, then back
   off ~30%.
2. **Add kd** to damp the oscillation. Too much sounds like a buzz from the motors.
3. **Add ki last, small**, only to remove a steady lean.
4. **Trim** (`trim_rad`) for a robot that balances but always drifts one way — a
   CG offset.
5. The outer velocity loop (`vel_bias_gain`) is **off** by design for the MVP. Enable
   it only once the inner loop is solid, starting very small.

**Order of magnitude, not a recommendation:** in the host simulation (a point mass
15 cm above 30 mm wheels) `kp ≈ 5×10⁵` and `kd ≈ 5×10⁴` balanced it. The real robot
will differ.

**Faster kp search with the S1 slider:** set `OMNIS_TUNE_KP_FROM_POT 1` and
`p->balance.tune_kp_max` (e.g. `1000000.0f`). The slider then sets kp live from 0 to
that maximum; while disarmed the status shows `tune: slider 0.NN -> kp NNN`. **Start
with the slider at zero.** Arming balance is allowed whenever `tune_kp_max > 0`
in this mode.

While balancing the per-second status line is **silent** (a blocking log line is a
disturbance). On disarm one summary prints:
`DISARMED after 12.4 s balancing | peak lean 7.3 deg | final speed 120 st/s`.
Set `OMNIS_TELEMETRY_IN_BALANCE 1` to keep the status line while tuning.

### 7.5 Known balance limitations

- **No encoders.** The robot will slowly drift across the room; that is what the
  outer loop is for (omnis-info.md §13a).
- **Reversal dead time.** A wheel reversing waits up to ~8 ms for its queued pulses
  to drain before DIR changes. Balancing in place reverses often; if the robot
  "hunts" near upright, this is a suspect.
- **Mecanum crab.** Turning while balancing (roll stick) drives two mecanum wheels
  differentially, which also produces some sideways force.
- A lean past 35° is treated as a fall (`TILT`).

---

## Status line field guide

```
500.0Hz body 812us ovr 0 | DISARMED | faults none | imu r+0.3 p-1.2 dis 0.4 i2c 640us bad 0 | rc OK LQ100 thr+0.00 pit+0.00 rol+0.00 yaw+0.00 arm0 mode LOW spd LOW | whl +0 +0 +0 +0
```

| Field | Meaning |
|---|---|
| `500.0Hz` | measured loop rate |
| `body` | worst loop-body time this second [µs] |
| `ovr` | missed ticks this second (the line's own cost is excluded) |
| state | `DISARMED`, `ARMED-FLAT`, `ARMED-BAL-SETTLE`, `ARMED-BALANCE` |
| `faults` | active faults, most severe first |
| `imu r p` | fused roll / pitch [°] (flat frame while disarmed) |
| `dis` | worst IMU disagreement this second [°] — fault at 15 |
| `i2c` | worst I²C time in one tick [µs] |
| `bad` | ticks this second without a valid IMU estimate |
| `rc` | `NONE` (never seen), `OK`, `LOST`; `LQ` = uplink link quality % |
| `thr pit rol yaw` | decoded sticks, after the deadzone |
| `arm mode spd` | arm request, drive-mode and speed switches |
| `whl` | commanded wheel rates FL FR RL RR [steps/s] |

## Build-time switches (`main/omnis_config.h`)

| Switch | Default | Purpose |
|---|---|---|
| `OMNIS_TICK_RATE_HZ` | 500 | loop rate |
| `OMNIS_I2C_SCL_HZ` | 400000 | I²C speed |
| `OMNIS_IMU_READ_ALTERNATE` | 0 | one IMU per tick, if I²C is too slow |
| `OMNIS_RUN_MOUNT_WIZARD` | 0 | boot into the IMU mount wizard (Stage 3.3) |
| `OMNIS_BENCH_STEP_TEST` | 0 | bench motion pattern, radio ignored, **wheels off the ground** |
| `OMNIS_TELEMETRY_ENABLED` | 1 | per-second status line |
| `OMNIS_TELEMETRY_IN_BALANCE` | 0 | keep the status line while balancing |
| `OMNIS_TUNE_KP_FROM_POT` | 0 | S1 slider sets balance kp live |
