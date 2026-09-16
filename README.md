# OMNIS

**O**mnidirectional **M**obility and **N**on-holonomic **I**nertial **S**tabilization

A four-wheel robot platform where every wheel is driven by its own stepper motor,
built from scratch on the ESP32-S3 as a personal embedded-systems portfolio
project.

Two headline capabilities:

1. **Holonomic drive** — mecanum angled-roller wheels give full directional
   control: strafe, rotate in place, and move diagonally without turning first.
2. **Self-balancing mode** — the platform can tip up and balance/drive on two
   wheels, Segway-style, in addition to normal four-wheel driving.

All four steppers stay powered during two-wheel balance mode. That is
deliberate: the platform must be able to balance on *either* side, so if it is
flipped mid-run the new pair of wheels is immediately drivable. The balance
controller detects which side is down and re-maps which two wheels provide
balance thrust.

---

## Where the working firmware is

> ### 👉 [`omnis-superloop-mvp/`](omnis-superloop-mvp/) — the complete, buildable firmware
>
> A **plain superloop with no FreeRTOS tasks, queues, semaphores or mutexes**.
> All seven stages are built, host-tested and committed: 500 Hz dual-IMU EKF,
> mecanum kinematics, CRSF radio, RMT step generation, fault handling, arming
> supervisor and the two-wheel balance controller.
>
> **Start at [`omnis-superloop-mvp/TESTING.md`](omnis-superloop-mvp/TESTING.md)
> §0 before applying power** — board Rev 2.0 hardwires the A4988 enable, so the
> motors are live whenever 12 V is present.

The repository holds **two** firmware trees, and it is worth being clear about
which is which:

| Tree | What it is | State |
|---|---|---|
| [`omnis-superloop-mvp/`](omnis-superloop-mvp/) | The **deadline build**. Bare superloop, no RTOS primitives, everything needed to drive and balance | **Complete**, host-tested, awaiting bench bring-up on the assembled Rev 2.0 board |
| `main/` (repository root) | The **future FreeRTOS build** described in [`assets/documentation/omnis-info.md`](assets/documentation/omnis-info.md) | Still at the ESP-IDF hello-world stage — a placeholder, not abandoned work |

The superloop MVP exists because the full RTOS architecture is a learning
project with a longer runway than the delivery deadline allowed. Features left
out of it — OLED and buttons, microSD/FATFS and `params.json`, data logging,
OTA/Wi-Fi, battery sensing — are **deferred, not abandoned**, and the design for
each is already written down in the info file.

---

## Repository layout

```
OMNIS/
├── omnis-superloop-mvp/            ★ the working firmware — start here
│   ├── main/                         44 sources; see OPERATIONS_GUIDE.md §3
│   │   ├── main.c                    boot sequence + the superloop itself
│   │   ├── tick.{c,h}                500 Hz GPTimer, one flag, no RTOS
│   │   ├── imu.{c,h} mpu6050.*       dual MPU6050 over I²C
│   │   ├── attitude_ekf.* imu_fusion.*   per-IMU EKF + inverse-covariance fusion
│   │   ├── mecanum_kinematics.*      IK/FK for the parallel-roller layout
│   │   ├── drive.{c,h}               sticks → body velocity → wheel rates
│   │   ├── crsf*.{c,h} rc_input.*    ExpressLRS radio
│   │   ├── step_wave.* step_gen.*    STEP pulses via the RMT peripheral
│   │   ├── balance.{c,h} pid.*       two-wheel balance controller
│   │   ├── fault.* buzzer*.* supervisor.*   safety and arming
│   │   └── omnis_{pins,params,config}.*     board map, tunables, switches
│   ├── test/                         7 host suites, no hardware needed
│   ├── OPERATIONS_GUIDE.md           build, architecture, file index, tuning
│   ├── TESTING.md                    bench bring-up, stage by stage
│   ├── README.md                     overview and stage status
│   └── BUILD-LOG.md                  scope, decisions, construction history
│
├── assets/
│   ├── kinematics/                   drivetrain maths, derived from scratch
│   │   ├── mecanum-kinematics-derivation.md     the full derivation
│   │   ├── mecanum-kinematics-reference.md      equations and verified numbers
│   │   ├── mecanum-kinematics-code-explained.md line-by-line walkthrough
│   │   └── mecanum_kinematics.{c,h}             the module itself
│   ├── control/                      EKF, fusion and PID: derivations + modules
│   ├── documentation/
│   │   ├── omnis-info.md             full design spec + what the MVP does — ONE copy
│   │   ├── s3-pro-docs/              board pinout, datasheet, CAD model
│   │   └── setup-guides/             toolchain, VS Code, ESP32 command reference
│   └── pcb/                          KiCad project and schematics
│
├── main/                             future FreeRTOS build (hello-world stub)
├── CMakeLists.txt                    root ESP-IDF project
├── .devcontainer/                    QEMU/Linux container (no board access)
└── .vscode/
```

The modules under `assets/kinematics/` and `assets/control/` are **byte-identical**
to the copies in `omnis-superloop-mvp/main/`. They live in `assets/` with their
derivations so the maths and the code that implements it stay next to each other.

---

## Drivetrain — read this before touching the kinematics

The wheels are **not** in the textbook X-drive arrangement. Mounting-shaft and
motor constraints put the **same roller tilt on both left wheels** and the
mirrored tilt on **both right wheels**:

```
        FL  /                 \  FR          left  pair  FL, RL  ->  δ = -1
        RL  /                 \  RR          right pair  FR, RR  ->  δ = +1
```

The standard mecanum mixing matrix does not describe this robot. The inverse and
forward kinematics were **re-derived from first principles** for this layout —
see [`assets/kinematics/mecanum-kinematics-derivation.md`](assets/kinematics/mecanum-kinematics-derivation.md).

Four consequences worth knowing up front:

- **Strafe splits along the sides**, not the diagonals. The `vx = vy` bench test
  idles the whole **left side**, not the FL/RR diagonal.
- **The two axles have different yaw levers** — `(L+W)/2` at the front and
  `(L−W)/2` at the rear, a **64.7:1** ratio on this frame. During a pure spin
  the rear wheels barely turn. That is correct, not a fault.
- **Still fully holonomic and no slower.** `det(MᵀM) = 4L²`, non-zero for any
  real chassis, and the peak wheel rate per unit of commanded motion is
  unchanged — same top speed, strafe speed and yaw rate as an X-drive.
- **Odometry is noisier.** The strafe and yaw columns meet at 44° instead of
  90°, amplifying per-wheel error by 1.44× on `vy` and 2.03× on `w`.

The layout is stated in exactly one place — the four `MECANUM_DELTA_*` constants
in `mecanum_kinematics.h` — and everything else is computed from them. If the
robot strafes the wrong way, negate all four together; never fix it with a `DIR`
invert flag. [`OPERATIONS_GUIDE.md §4.2`](omnis-superloop-mvp/OPERATIONS_GUIDE.md)
has the two-direction bench test that tells the two failures apart.

---

## Hardware

| Module | Part | Notes |
|---|---|---|
| MCU | **ESP32-S3-WROOM-1-N16R8** on an EdgeHax S3 Pro dev board | 16 MB flash / 8 MB Octal PSRAM |
| Wheel motors (×4) | Nidec **KV4239-T3B004** | 2-phase hybrid stepper, NEMA17-class (42 mm) |
| Stepper drivers (×4) | **A4988** | Fixed 1/16 microstepping → 3200 microsteps/rev |
| Wheels (×4) | Mecanum omnidirectional, 60 mm | **Lateral parallel roller layout** — see above |
| IMUs (×2) | **MPU6050** at `0x68` and `0x69` | Diagonally opposite corners, mounted **antiparallel**; fused by inverse covariance |
| Display | **SSD1306** 128×64 I²C OLED | Deferred in the MVP |
| RC link | **RadioMaster Pocket** TX + **ExpressLRS Nano** Rx | CRSF over UART, 420000 baud |
| Storage | Onboard microSD (SPI mode, FAT32) | Deferred in the MVP |
| Buzzer | 3-pin buzzer module | Fault and failsafe cues |
| Buttons | 6× tactile | **Active-HIGH**, needs pull-downs. Deferred in the MVP |
| Power | 3S LiPo → **L7805CV** 5 V logic rail | Battery divider present; sensing deferred in the MVP |

Schematic: [`assets/pcb/`](assets/pcb/) (KiCad project, Rev 2.0).

### GPIO map — board Rev 2.0

Traced wire-by-wire from the KiCad schematic, not read off a drawing. The
authoritative copy is
[`omnis-superloop-mvp/main/omnis_pins.h`](omnis-superloop-mvp/main/omnis_pins.h).

| GPIO | Function |
|---|---|
| 4, 5 | Front-Left STEP / DIR |
| 40, 38 | Front-Right STEP / DIR |
| 15, 46 | Rear-Left STEP / DIR (46 is a strapping pin) |
| 21, 42 | Rear-Right STEP / DIR |
| 8, 9 | I²C SDA / SCL — OLED and both MPU6050s |
| 7 | IMU interrupt |
| 17, 18 | UART to the ExpressLRS receiver — ESP **TX** / **RX** |
| 16 | Buzzer |

**Three gotchas worth knowing before you wire or flash anything:**

- **`COM_ENA` is gone on Rev 2.0.** The A4988 enable is hardwired, so the motors
  are energised whenever 12 V is present — before the firmware runs, and while
  it is halted in a fault. Nothing in software can de-energise them. The
  failsafe is therefore "stop STEP pulses, motors hold", and setting the four
  `Vref` current limits is a **prerequisite to first power-up**, not a
  calibration step.
- The schematic's `RX` / `TX` net names are from the *receiver's* perspective.
  GPIO17 is the ESP32's **TX** and GPIO18 is its **RX** — the reverse of the
  labels. The firmware auto-detects the RX pin anyway.
- GPIO46 is a strapping pin and GPIO38–42 sit on the default JTAG pins.
  Everything behaves normally as long as nothing holds them through a power
  cycle.

---

## Firmware architecture (superloop MVP)

```
            GPTimer ISR (500 Hz) ── sets a volatile flag, nothing else
                     │
   app_main ─ while(1) ─ tick_wait() ─────────────────────────────────────────┐
     │                                                                        │
     │  1. SENSE   imu_update()      2× MPU6050 on I²C → mount → bias → EKF → fuse
     │             crsf_poll()       UART1 RX-only → CRSF parser → channels
     │                                                                        │
     │  2. JUDGE   faults            IMU comm/disagree · link loss · tilt · overrun
     │             supervisor        DISARMED / ARMED-FLAT / BAL-SETTLE / BALANCE
     │                                                                        │
     │  3. ACT     flat:     sticks → IK → clamp → deadband → slew ─┐         │
     │             balance:  lean PID (balance frame) → wheel pair ─┼→ step_gen_update()
     │             else:     step_gen_stop()                         │    RMT ISR chains
     │                                                               │    ~2 ms bursts
     │  4. REPORT  buzzer (every tick) · telemetry (once a second)            │
     └────────────────────────────────────────────────────────────────────────┘
```

**The no-RTOS rule.** No FreeRTOS primitive appears in OMNIS-authored code —
ESP-IDF driver internals are treated as hardware. It was checked mechanically,
not merely followed: neither the `main/` sources nor the undefined symbols of the
compiled `libmain.a` contain any FreeRTOS task, queue, semaphore, event-group,
timer or port symbol.

The superloop is pinned to **CPU1** with that core's idle-task watchdog disabled
— a superloop never yields, so the idle task never runs.

Full explanation, file index and tuning cheat sheet:
[`omnis-superloop-mvp/OPERATIONS_GUIDE.md`](omnis-superloop-mvp/OPERATIONS_GUIDE.md).

### Control and estimation

Each MPU6050 runs its own 4-state EKF — `[roll, pitch, gyro_bias_roll,
gyro_bias_pitch]` — with adaptive measurement noise that distrusts the
accelerometer whenever `|‖a‖ − 1g|` exceeds ~0.2 g, i.e. exactly when the
platform is accelerating rather than merely tilted. Because roll couples only to
`bias_roll` and pitch only to `bias_pitch`, the 4×4 filter decomposes **exactly**
into two independent 2-state filters — no matrix inverse anywhere.

A fusion layer takes the inverse-covariance-weighted average of the two
estimates and raises a fault if they disagree by more than 15°, failing safe
instead of averaging through a bad sensor. Disagreement is measured on the
**gravity vectors**, not on Euler angles, so it stays meaningful at any attitude.

Balance runs in a dedicated **balance frame** rather than the flat body frame,
because flat-frame pitch folds at exactly the balance point — 85° and 95° both
read 85°, losing the lean direction where a balancer most needs it. The
controller is an inner 500 Hz PID on lean angle (rate taken straight from the
gyro for latency) plus a slow 20 Hz outer loop that biases the lean setpoint to
bleed off sustained forward effort. There are no wheel encoders, so this is the
honest ceiling of the current BOM — an encoder pair is the single
highest-leverage hardware upgrade available.

### Safety

- **Arming supervisor** with LOW→HIGH edge consumption: a fault that clears
  while the switch is still up does not re-arm the robot on its own.
- **CRSF link-loss, IMU comm failure, IMU disagreement, tilt and loop overrun**
  all latch faults and stop STEP pulses within the same tick.
- **Failsafe policy:** "hold last value" is explicitly the wrong default here — a
  dropped link mid-deflection would leave the robot executing that command
  unattended.
- Balance mode refuses to arm with zero gains, because a zero-gain balancer
  simply falls over.

---

## Building

Requires **ESP-IDF v6.0.2** natively for hardware work. Activate it with the
tools script — `export.sh` looks for a Python 3.11 virtualenv while this install
runs on 3.14:

```bash
. ~/.espressif/tools/activate_idf_v6.0.2.sh
```

Build and flash the working firmware:

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp build
```

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp -p /dev/cu.usbmodem101 flash monitor
```

Run the host test suites — no hardware, no ESP-IDF:

```bash
cd /Volumes/Projects/OMNIS/omnis-superloop-mvp && ./test/run_host_tests.sh
```

Full setup, including the native-vs-devcontainer split, is in
[`assets/documentation/setup-guides/phase0-toolchain-environment.md`](assets/documentation/setup-guides/phase0-toolchain-environment.md).
The devcontainer has **no USB passthrough** — it is for QEMU experiments only.
All real board work happens natively.

---

## Status

| Stage (superloop MVP) | Built + host-tested | Verified on hardware |
|---|---|---|
| 1 Timing spine | ✅ | ✅ 2026-09-07 (Rev 1 board): 500 Hz, zero overruns |
| 2 Kinematics | ✅ | boot self-check passed on the FPU |
| 3 IMUs + EKF | ✅ | ⬜ |
| 4 Radio | ✅ | ⬜ |
| 5 Step generation | ✅ | ⬜ |
| 6 Faults + supervisor | ✅ | ⬜ |
| 7 Balance | ✅ | ⬜ |

Not known until the assembled Rev 2.0 board runs: real I²C time per tick, IMU
mounting descriptors, motor `DIR` polarities, radio channel order, and every
balance gain. All five are **parameters**, not code changes —
[`OPERATIONS_GUIDE.md §4`](omnis-superloop-mvp/OPERATIONS_GUIDE.md) is the cheat
sheet.

### Roadmap beyond the MVP

Everything deferred from the superloop build, in the order it is worth adding:

1. **FATFS mount + `params.json` loader** with its fallback chain — so tuning
   stops requiring a rebuild
2. **Data logging** to append-only JSON Lines under `/sdcard/logs/`
3. **On-board wheel encoders** — the single highest-leverage hardware upgrade,
   and what the velocity-bias outer loop is really waiting for
4. **OLED and the button/menu state machine**
5. **OTA over Wi-Fi** with a factory slot and rollback verified against a
   deliberately broken image
6. **The full FreeRTOS architecture** at the repository root, once it has been
   learned properly rather than adopted in a hurry

### Open questions

- Internal vs. external button pull-downs
- Heading-hold: hold-to-engage or press-to-toggle
- What the transmitter's trim potentiometer trims — static balance-setpoint bias
  or live control gain
- A4988 `Vref` current limits, unset until the motors are on the bench

---

## License

Not yet specified.
