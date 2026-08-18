# OMNIS

**O**mnidirectional **M**obility and **N**on-holonomic **I**nertial **S**tabilization

A four-wheel robot platform where every wheel is driven by its own stepper motor, built from scratch on the ESP32-S3 as a personal embedded-systems portfolio project.

Two headline capabilities:

1. **Holonomic drive** — Mecanum-style angled-roller wheels give full directional control: strafe, rotate in place, and move diagonally without turning first.
2. **Self-balancing mode** — the platform can tip up and balance/drive on two wheels, Segway-style, in addition to normal four-wheel driving.

All four steppers stay powered during two-wheel balance mode. That is deliberate: the platform must be able to balance on *either* side, so if it is flipped mid-run the new pair of wheels is immediately drivable. The balance controller detects which side is down and re-maps which two wheels provide balance thrust.

> **Status: early development.** The hardware design and firmware architecture are specified; the application code is still at the ESP-IDF hello-world stage. See [Roadmap](#roadmap).

---

## Design philosophy

Written by hand, not vibe-coded. The stack is chosen for what it teaches:

- **Bare-metal-style ESP32-S3 programming** — direct peripheral work, not high-level Arduino libraries
- **A custom RTOS layer** built on top of the FreeRTOS that ships inside ESP-IDF
- **Custom peripheral drivers** rather than off-the-shelf libraries wherever reasonably feasible
- **An EKF** (Extended Kalman Filter) fusing two IMUs for attitude and balance estimation

Two deliberate exceptions where correctness beats from-scratch: `esp_https_ota` for firmware updates and `esp_vfs_fat` for the SD card. Reinventing chunked flash writes or a FAT implementation buys risk, not portfolio value.

---

## Hardware

| Module | Part | Notes |
|---|---|---|
| MCU | **ESP32-S3-WROOM-1-N16R8** on an EdgeHax S3 Pro dev board | 16 MB flash / 8 MB Octal PSRAM |
| Wheel motors (×4) | Nidec **KV4239-T3B004** | 2-phase hybrid stepper, NEMA17-class (42 mm) |
| Stepper drivers (×4) | **A4988** | Fixed 1/16 microstepping → 3200 microsteps/rev |
| Wheels (×4) | Magnum / Mecanum omnidirectional | Angled rim rollers enable sideways and diagonal travel |
| IMUs (×2) | **MPU6050** at `0x68` and `0x69` | Mounted at diagonally opposite corners for redundant, fused attitude sensing |
| Display | **SSD1306** 128×64 I²C OLED | Status plus an on-screen config menu |
| RC link | **RadioMaster Pocket** TX + **ExpressLRS Nano** Rx | CRSF over UART |
| Storage | Onboard microSD (SPI mode, FAT32) | Run logs and a hand-editable parameter file |
| Buzzer | 3-pin buzzer module | Fault, warning, and OTA-progress cues |
| Buttons | 6× tactile — Up / Down / Left / Right / Select / Special-Function | **Active-HIGH**; needs pull-downs |
| Power | 3S LiPo → **L7805CV** 5 V logic rail | Battery sensed through a 120 kΩ / 33 kΩ divider on GPIO6 |

Schematic: [`assets/pcb/Schematics/schematic_omnis.pdf`](assets/pcb/Schematics/schematic_omnis.pdf) (rev 1.0).

### GPIO map

| GPIO | Function |
|---|---|
| 1 | `COM_ENA` — shared enable for all four A4988 drivers |
| 2, 39, 47, 48 | Buttons: Right, Down, Left, Up |
| 3, 45 | Buttons: Special-Function, Select (both strapping pins) |
| 4, 5 | Front-Left STEP / DIR |
| 41, 40 | Front-Right STEP / DIR |
| 15, 16 | Rear-Left STEP / DIR |
| 21, 42 | Rear-Right STEP / DIR |
| 6 | Battery voltage sense (ADC1_5) |
| 8, 9 | I²C SDA / SCL — OLED and both MPU6050s |
| 10–13 | microSD CS / MOSI / CLK / MISO |
| 14 | Buzzer |
| 17, 18 | UART to ExpressLRS receiver — TX / RX |

Spare: GPIO 0, 7, 19, 20, 35–38, 43, 44, 46.

**Three gotchas worth knowing before you wire or flash anything:**

- The schematic's `RX` / `TX` net names are from the *receiver's* perspective. GPIO17 is the ESP32's **TX** and GPIO18 is its **RX** — the reverse of the labels.
- Buttons are wired to VCC, so they are **active-HIGH** and idle floating. Enable internal pull-downs (or fit external ones).
- GPIO3 and GPIO45 are strapping pins, and GPIO39–42 are the default JTAG pins whose mode is strapped by GPIO3. Everything behaves normally as long as no button is held down through a power cycle.

---

## Firmware architecture

### Control and estimation

Each MPU6050 runs its own 4-state EKF — `[roll, pitch, gyro_bias_roll, gyro_bias_pitch]` — with adaptive measurement noise that distrusts the accelerometer whenever `|‖a‖ − 1g|` exceeds ~0.2 g, i.e. exactly when the platform is accelerating rather than merely tilted. A fusion layer takes the inverse-covariance-weighted average of the two estimates and raises a fault if they disagree by more than ~15°, failing safe instead of averaging through a bad sensor. Target loop rate is **500 Hz**.

Balance is a cascaded controller: an inner 500 Hz PID on lean angle (rate taken straight from the gyro for latency), plus a slow 10–20 Hz outer loop that integrates commanded step-rate as a stand-in for velocity and biases the lean setpoint to pull it back toward zero. There are no wheel encoders, so this is the honest ceiling of the current BOM — an encoder pair is the single highest-leverage hardware upgrade available.

### Safety

- **Kill switch** and **CRSF link-loss timeout** both drive `COM_ENA` to disabled, wired in at the top of the control loop from the very first commit rather than bolted on last.
- **Battery cutoff** at 3.3 V/cell (9.9 V pack), with a buzzer warning at 3.5 V/cell.
- **Failsafe policy:** "hold last value" is explicitly the wrong default here — a dropped link mid-deflection would leave the robot executing that command unattended.
- **OTA is gated on the robot being parked** and disables the steppers throughout, since flash-write jitter on a balancing platform is a fall risk.

### Storage and configuration

The SD card holds append-only JSON Lines run logs under `/sdcard/logs/boot_NNNN.log` and a single hand-editable `/sdcard/params.json` carrying every tunable: PID gains, geometry, per-IMU calibration, CRSF channel map, Wi-Fi credentials, and battery thresholds. On boot the firmware parses `params.json`, falls back to `params.json.bak`, then to hardcoded defaults *with a buzzer fault* — silently booting a balancing robot with zeroed PID gains is an immediate-fall failure mode. All SD I/O lives on a low-priority task fed by a queue, never on the control loop.

### OTA

Custom 16 MB partition table with a **factory** slot kept alongside `ota_0` / `ota_1`, so a bad wireless push can never take out the known-good image. Rollback is enabled and `esp_ota_mark_app_valid_cancel_rollback()` is only called after a real self-test — IMUs responding, drivers enumerating, CRSF link established — not immediately on boot.

---

## Repository layout

```
OMNIS/
├── CMakeLists.txt
├── main/                              Firmware sources
├── sdkconfig                          Generated ESP-IDF configuration
├── assets/
│   ├── omnis-info.md                  Full design spec — the source of truth
│   ├── pcb/Schematics/                Schematic PDF
│   ├── s3-pro-docs/                   Board pinout, datasheet, CAD model
│   └── setup-guides/
│       ├── phase0-toolchain-environment.md
│       └── vscode-setup.md
├── .devcontainer/                     QEMU/Linux container (no board access)
└── .vscode/
```

---

## Building

Requires **ESP-IDF v5.3.1** natively for hardware work. Full setup, including the native-vs-devcontainer split, is in [`assets/setup-guides/phase0-toolchain-environment.md`](assets/setup-guides/phase0-toolchain-environment.md).

```bash
idf.py set-target esp32s3 && idf.py build
```

```bash
idf.py -p /dev/cu.usbmodem* flash monitor
```

The devcontainer runs a newer IDF (v6.1-dev) but has **no USB passthrough** — it is for QEMU experiments only. All real board work happens natively.

---

## Roadmap

1. **CRSF frame parser** and channel decode on GPIO17/18 — nothing else works without it
2. **Button debounce and mode state machine** — drive mode, speed limiter, kill, position-hold, heading-hold
3. **Kill switch and link-loss failsafe** — a single GPIO write, tested early
4. **Flat-mode mecanum mixing** — bench-testable on four wheels, no balance loop needed
5. **FATFS mount + `params.json` loader** with its fallback chain
6. **Dual-IMU EKF** and side detection
7. **Balance control loop**
8. **OTA over Wi-Fi** with rollback verified against a deliberately broken image
9. **On-screen config menu** — deliberately last, once the real settings surface is known

### Open questions

- Internal vs. external button pull-downs
- Heading-hold: hold-to-engage or press-to-toggle
- What the transmitter's trim potentiometer trims — static balance-setpoint bias or live control gain
- A4988 `Vref` current limits, unset until motor specs are dialed in
- Mecanum inverse-kinematics equations, still to be derived (spec §10)

---

## License

Not yet specified.
