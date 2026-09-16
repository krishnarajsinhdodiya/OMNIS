# OMNIS Superloop MVP

Firmware for the OMNIS robot — four-wheel mecanum drive that can also stand up
and balance on either wheel pair — written as a **plain superloop with no FreeRTOS
tasks, queues, semaphores or mutexes**. It is the deadline build that gets the
robot driving and balancing before the full FreeRTOS architecture in
[`../assets/documentation/omnis-info.md`](../assets/documentation/omnis-info.md) is
learned and implemented. Deferred features are deferred, not abandoned.

**Target:** ESP32-S3 (EdgeHax S3 Pro / DevKitC-1 class), board Rev 2.0 · ESP-IDF v6.0.2

| Document | Read it for |
|---|---|
| **[TESTING.md](TESTING.md)** | Bench bring-up, stage by stage. **Start here before powering anything.** |
| [PLAN.md](PLAN.md) | Scope, decisions and why, stage definitions |
| [BUILD-LOG.md](BUILD-LOG.md) | The full history: what was built, what was learned, every bug found |
| [omnis-info.md](omnis-info.md) | The project info file, updated for this build |

---

## What it does

| In scope | Where |
|---|---|
| 500 Hz loop: dual-MPU6050 attitude EKF, inverse-covariance fusion, 15° disagreement fault | `imu.c`, `attitude_ekf.c`, `imu_fusion.c` |
| Verified 4-wheel mecanum inverse / forward kinematics | `mecanum_kinematics.c`, `drive.c` |
| CRSF / ExpressLRS radio: throttle→vx, yaw→w, pitch+roll→vx/vy, arm, mode, speed | `crsf*.c`, `rc_input.c` |
| STEP pulses for four A4988s on the RMT peripheral, nothing in the hot path | `step_wave.c`, `step_gen.c` |
| Buzzer fault / failsafe signalling, arming supervisor | `fault.c`, `buzzer*.c`, `supervisor.c` |
| Two-wheel balance: lean PID in a dedicated balance frame | `balance.c`, `pid.c` |

**Excluded — deferred, not abandoned:** OLED and buttons, microSD / FATFS and
`params.json`, data logging, OTA and Wi-Fi, battery voltage sensing.

---

## Architecture

```
            GPTimer ISR (500 Hz) ── sets a volatile flag, nothing else
                     │
   app_main ─ while(1) ─ tick_wait() ─────────────────────────────────────────┐
     │                                                                        │
     │  1. SENSE   imu_update()      2x MPU6050 on I2C → mount → bias → EKF → fuse
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

**The no-RTOS rule** (PLAN.md §1a): no RTOS primitive in OMNIS-authored code.
ESP-IDF driver internals are treated as hardware. The rule was checked mechanically,
not just followed: neither `main/` source nor the undefined symbols of the compiled
`libmain.a` contain any FreeRTOS task, queue, semaphore, event-group, timer or port
symbol.

**On board Rev 2.0 the A4988 enable is hardwired**, so the motors are energised
whenever 12 V is present and the supervisor is the entire safety story: STEP pulses
exist only while armed. Read TESTING.md §0.

---

## Layout

```
omnis-superloop-mvp/
  main/
    main.c                   boot sequence + the superloop
    omnis_pins.h             board Rev 2.0 GPIO map (traced from the KiCad file)
    omnis_params.{c,h}       every tunable number (future params.json)
    omnis_config.h           build-time switches
    tick.{c,h}               500 Hz GPTimer
    mpu6050{.c,.h,_regs.h}   IMU driver
    imu.{c,h}                both IMUs: calibration, fusion, frames, mount wizard
    attitude_ekf.{c,h}  ┐
    imu_fusion.{c,h}    │    byte-identical to assets/control/
    pid.{c,h}           │
    omnis_imu_mounting.h┘
    mecanum_kinematics.{c,h} byte-identical to assets/kinematics/
    drive.{c,h}              sticks → wheel rates, slew limiter
    crsf_parser.{c,h}        pure CRSF parser
    crsf.{c,h}               UART glue, RX-pin auto-detect
    rc_input.{c,h}           channels → sticks, switches, arm request
    step_wave.{c,h}          pure STEP waveform generator
    step_gen.{c,h}           RMT glue for four wheels
    fault.{c,h}              fault latch
    buzzer_pattern.{c,h}     pure buzzer patterns
    buzzer.{c,h}             buzzer GPIO
    supervisor.{c,h}         arming state machine
    balance.{c,h}            balance controller
  test/
    run_host_tests.sh        runs every suite
    test_*.c                 host tests, no hardware
    crsf_golden.h            reference frames from an independent implementation
```

Every pure module (no ESP-IDF includes) is host-tested. The hardware glue
(`imu.c`, `crsf.c`, `step_gen.c`, `buzzer.c`, `main.c`) is exercised by the bench
procedure in TESTING.md.

---

## Build, flash, test

```bash
. ~/.espressif/tools/activate_idf_v6.0.2.sh
```

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp build
```

```bash
cd /Volumes/Projects/OMNIS && idf.py -C omnis-superloop-mvp -p /dev/cu.usbmodem101 flash monitor
```

```bash
cd /Volumes/Projects/OMNIS/omnis-superloop-mvp && ./test/run_host_tests.sh
```

---

## Status

| Stage | Built + host-tested | Verified on hardware |
|---|---|---|
| 1 Timing spine | ✅ | ✅ 2026-09-07 (Rev 1 board): 500 Hz, zero overruns, no jitter |
| 2 Kinematics | ✅ | boot self-check passed on the FPU (2026-09-07) |
| 3 IMUs + EKF | ✅ | ⬜ |
| 4 Radio | ✅ | ⬜ |
| 5 Step generation | ✅ | ⬜ |
| 6 Faults + supervisor | ✅ | ⬜ |
| 7 Balance | ✅ | ⬜ |

Not yet known until the board runs: real I²C time per tick, IMU mounting on Rev 2.0,
motor DIR polarities, radio channel order, and every balance gain.
