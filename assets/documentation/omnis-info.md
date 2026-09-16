# OMNIS — Project Info File

> **What this file is.** The single source of truth for OMNIS, covering **both**
> firmware builds:
>
> - **The specification** — what OMNIS is designed to be, including everything
>   the superloop MVP defers. This is the brief for the full FreeRTOS build.
> - **What the superloop MVP actually does**, section by section, as indented
>   `> **Superloop MVP:**` notes under the section they qualify.
>
> **Everything unmarked applies to both builds.** Where a `Superloop MVP` note
> appears it describes the deadline build in `omnis-superloop-mvp/`, and
> overrides the surrounding text for that build only.
>
> There is deliberately no second copy of this file. It previously existed twice
> — the spec here and an annotated duplicate inside `omnis-superloop-mvp/` — and
> the two had to be edited in lockstep, which is exactly the drift risk a
> duplicate invites. Merged into one file 2026-09-16.
>
> **Companion documents:** `omnis-superloop-mvp/README.md` (overview and layout),
> `OPERATIONS_GUIDE.md` (build, architecture, file index, tuning cheat sheet),
> `TESTING.md` (bench bring-up), `BUILD-LOG.md` (what was built, what was
> learned, every decision and every bug found), and `assets/kinematics/` +
> `assets/control/` for the drivetrain and estimation maths.

## MVP at a glance

| | |
|---|---|
| **Architecture** | One `while(1)` in `app_main`, paced by a 500 Hz GPTimer ISR that only sets a flag. **No FreeRTOS primitive in OMNIS-authored code** — verified mechanically, see below. |
| **Built** | 500 Hz dual-IMU EKF + fusion + 15° disagreement fault · mecanum IK/FK · CRSF radio · RMT STEP generation for four A4988s · buzzer faults and failsafe · arming supervisor · two-wheel balance |
| **Excluded** | OLED and buttons (§14) · microSD, FATFS and `params.json` (§9) · OTA and Wi-Fi (§8) · battery sensing (§3g). **Deferred, not abandoned.** |
| **Verified on hardware** | Stage 1 only (500 Hz timing spine, 2026-09-07). Everything else is built, host-tested and awaiting the assembled Rev 2.0 board. |
| **Host tests** | 7 suites, 463 assertions, plus 97 more in `assets/control/` |

### Why a superloop, and what it costs

The full design (§1) commits to a custom RTOS layer on FreeRTOS. This build
deliberately sets that aside to get the robot moving on a deadline, and returns to
it afterwards. The bridge is not free:

- **Everything shares one thread of control.** A slow section delays everything
  else, so the loop measures itself every tick and faults if it misses 5% of its
  deadlines while armed.
- **No task priorities.** Work that a real RTOS build would put on a low-priority
  task (logging, telemetry) must instead be rationed by hand — which is why the
  status line is once a second and silent while balancing.
- **ESP-IDF is still FreeRTOS underneath.** `app_main` is itself a task and the
  drivers use RTOS primitives internally. The rule adopted is that *OMNIS-authored
  code* uses none, and IDF driver internals count as hardware. This is checked, not
  assumed: neither the firmware sources nor the undefined symbols of the compiled
  `libmain.a` reference any FreeRTOS task, queue, semaphore, event-group, timer or
  port symbol.

What carries over unchanged to the FreeRTOS build: the kinematics, the EKF and
fusion, the PID and balance controller, the CRSF parser, the fault and supervisor
logic, and the waveform generator — all of them pure, host-tested modules with no
IDF dependency.

---


**Name:** OMNIS — **O**mnidirectional **M**obility and **N**on-holonomic **I**nertial **S**tabilization

---

## 1. Project Goal

A four-wheel car, each wheel driven by its own stepper motor, built as a personal embedded-systems portfolio piece. Two headline capabilities:

1. **Holonomic drive** — Magnum (angled-roller/Mecanum-style) wheels give full directional control: strafe, rotate in place, move diagonally, without needing to turn first.
2. **Self-balancing mode** — the car can also tip up and balance/drive on two wheels, Segway-style, in addition to normal 4-wheel driving.

> **Superloop MVP:** implemented as stated — all four motors stay energised (board
> Rev 2.0 hardwires `EN#`, so they are energised whenever 12 V is present), and the
> grounded pair is chosen at arming from the sign of the flat-frame pitch. The
> airborne pair is commanded zero. Which pair is "active" is re-evaluated only at
> arming, never mid-balance.

**Balance-mode wheel behavior (confirmed):** all four steppers stay powered and active during 2-wheel balance mode — none are mechanically or electrically disengaged. This is deliberate: the platform needs to be able to balance on *either* side, so if it's flipped mid-run the "new" pair of wheels needs to be immediately drivable without a mode-switch delay. Firmware/control implication: the balance controller needs to know (or detect) which side is currently down and re-map which two wheels are "active" for balance thrust vs. which two are along for the ride, rather than assuming a fixed pair.

The name is deliberately left open-ended (not tied to "car" or "stepper" specifically) since this platform may extend into further omnidirectional-wheel work down the line.

**Software philosophy:** written from scratch by the builder, with light AI assistance only — not vibe-coded. Deliberately chosen stack:

- Bare-metal-style programming on the ESP32-S3 (not just calling high-level Arduino libraries)
- A custom RTOS layer, built on top of FreeRTOS (which ships inside ESP-IDF) — see `vscode-setup.md` §8 for the learning path
- Custom peripheral drivers rather than pulling in ready-made libraries wherever reasonably feasible
- An EKF (Extended Kalman Filter) fusing the two IMUs for attitude/balance estimation

> **Superloop MVP:** the custom RTOS layer is the one item deliberately postponed —
> this build is a plain superloop (see "Why a superloop" above). The other three hold:
> register-level drivers written here (MPU6050, CRSF, RMT step generation), no
> Arduino libraries, and the dual-IMU EKF is implemented and host-tested.

---

## 2. Components

| Module | Part | Notes |
|---|---|---|
| Wheel motors (×4) | Nidec Servo Corporation **KV4239-T3B004** (OEM cross-ref FK2-7586) | 2-phase hybrid stepper, NEMA17-class (42mm), KV42 series |
| Stepper drivers (×4) | **A4988** | One per motor, U2–U5. RST#/SLP# tied to VCC. **Rev 2.0: EN# hardwired to GND — no software enable (§3f)** |
| Wheels (×4) | **Magnum wheels** (omnidirectional, angled-roller/Mecanum-style) | Angled rollers around the rim enable true sideways/diagonal movement for holonomic drive |
| MCU module | **ESP32-S3-WROOM-1-N16R8** (16MB flash / 8MB Octal PSRAM) on EdgeHax S3 Pro dev board | N16R8 confirmed by builder. PSRAM conflict resolved — FR_STEP/FR_DIR/RR_DIR/B_DOWN moved off GPIO35–38 onto GPIO39–42 (§3a/§3b), so module PSRAM variant (Octal vs Quad) no longer matters for this design. **Rev 2.0 note:** the KiCad schematic uses the `ESP32-S3-DEVKITC-1U-N8R2` symbol (8 MB flash); the superloop MVP therefore builds an 8 MB image header, which boots on both parts. |
| Dev board | **EdgeHax S3 Pro** | Schematic symbol: `ESP32-S3PRO-DEVKIT-edgehax` |
| IMUs (×2) | **MPU6050** (U6, U7) | Mounted at diagonally opposite corners for redundant/fused attitude sensing. AD0 address collision resolved: U6 = 0x68, U7 = 0x69. **As-built (confirmed 2026-09-05):** U6/0x68 at the **front-left** corner facing forward, U7/0x69 at the **rear-right** corner facing rearward — i.e. mounted **antiparallel, 180° apart about Z**. This must be corrected in firmware before the EKF or two healthy sensors read ~20° apart and trip §11c's fault at every boot. Constants in `assets/control/omnis_imu_mounting.h`. **Rev 2.0:** both INT pins wired-OR onto GPIO7 with a 10 kΩ pull-up — must be configured open-drain (§3h) |
| Display | **SSD1306**-based 128×64 I2C OLED (schematic symbol `DISPLAY-OLED-128X64-I2C`, designator G$1) | Controller part confirmed by builder — schematic still uses the generic OLED symbol, which is fine since the symbol doesn't need to change; drive it as SSD1306 |
| RC link | **RadioMaster Pocket** TX + **ExpressLRS Nano Rx** receiver (U10) | CRSF over UART — see §3c for the TX/RX crossover |
| Buzzer | **Buzzer Module** (U9, 3-pin: VCC / IO / GND) | Has its own driver IC onboard (not a bare piezo) — IO pin is a logic-level control signal. **Rev 2.0: GPIO16** (moved off GPIO14, which belongs to the microSD) |
| Buttons | 6× tactile, 4-pin (K4-6×6_TH) — Up / Down / Left / Right / Select / Special-function (B_SF) | Special-function opens an on-screen config/setup menu on the display (EdgeTX-menu-style, §14). Wired **active-HIGH** |
| Battery | 3S LiPo (schematic power-in header labeled "12V", nominal for 3S) | Powers whole system via 2-pin header H5. **No dedicated power switch planned** — board is live whenever the battery is connected; power-off = unplug. Voltage sensed via a divider on GPIO6 — see §3g |
| Regulator | **L7805CV** (U8) — **being replaced in Rev 2.0** | INPUT ← battery rail, OUTPUT → 5V logic rail (net "VCC"). A linear 12→5 V drop at ~0.6 A dissipates ~3.5 W and will thermally shut down; switching-regulator options are in `assets/pcb/rev2-pin-assignment.md` §4 |
| Capacitors | 9× 150µF electrolytic (U11–U19) | One per: I2C/OLED rail, each MPU6050 (×2), each A4988 logic rail (×4), ESP32 5V input, main battery input. No voltage rating specified in the schematic — pick per rail (25V+ for the 12V/battery side, lower is fine for the 5V logic side) |


## 3. ESP32-S3 GPIO Pinout — board Rev 2.0 (2026-09-15)

> **Rev 2.0 supersedes Rev 1.0.** Source of truth is `assets/pcb/OMNIS.kicad_sch`. Every row below was traced pin-by-pin through the schematic's wire segments to its global label. The planning document `assets/pcb/rev2-pin-assignment.md` was written before the schematic was final and disagrees on seven nets — see §3i. Firmware follows the schematic: `omnis-superloop-mvp/main/omnis_pins.h`.

### 3a. Connected / used pins

| GPIO | Schematic net | Connects to | Notes |
|---|---|---|---|
| 1 | B_UP | Up button | Active-HIGH, external pull-down (§3d). Freed by removing COM_ENA |
| 2 | B_DOWN | Down button | Active-HIGH |
| 4 | FL_STEP | Front-Left A4988 STEP | |
| 5 | FL_DIR | Front-Left A4988 DIR | |
| 6 | BATT_SENSE | Battery divider | ADC1_CH5 — §3g |
| 7 | MPU1_INT + MPU2_INT | Both MPU6050 INT pins, wired-OR, 10 kΩ (R3) pull-up to 3V3 | Both IMUs **must** be open-drain, active-low — §3h |
| 8 | I2C_SDA | I²C bus — both MPU6050s (+ OLED) | |
| 9 | I2C_SCL | I²C bus | |
| 15 | BL_STEP | Rear-Left ("back-left") A4988 STEP | Firmware name `RL` |
| 16 | BUZZER | Buzzer module IO | Moved from GPIO14 (microSD) |
| 17 | TX | ExpressLRS receiver **Rx** input | ESP32 transmit pin — §3c |
| 18 | RX | ExpressLRS receiver **Tx** output | ESP32 receives CRSF here — §3c |
| 21 | BR_STEP | Rear-Right ("back-right") A4988 STEP | Firmware name `RR` |
| 38 | FR_DIR | Front-Right A4988 DIR | The one clean pin of 35–38 on N16R8 |
| 39 (MTCK) | B_SF | Special-function button | Plain GPIO because GPIO3 is NC (§3e) |
| 40 (MTDO) | FR_STEP | Front-Right A4988 STEP | **EdgeHax S3 Pro orange-LED pin** — §3e |
| 41 (MTDI) | B_LEFT | Left button | EdgeHax white-LED pin |
| 42 (MTMS) | BR_DIR | Rear-Right A4988 DIR | EdgeHax green-LED pin — LED shows wheel direction |
| 46 | BL_DIR | Rear-Left A4988 DIR | **Strapping pin** — §3e |
| 47 | B_RIGHT | Right button | |
| 48 | B_SEL | Select button | |

### 3b. Deliberately unconnected

| GPIO | Why |
|---|---|
| 0 | BOOT strap — a pulled-down button here would force download mode |
| 3 | JTAG-source strap — left NC so GPIO39–42 stay ordinary GPIO |
| 10–14 | Onboard microSD |
| 19, 20 | Native USB D−/D+ (USB-Serial-JTAG) |
| 35, 36, 37 | Octal PSRAM on N16R8 — unusable (the Stage 1 firmware heartbeat on GPIO35 was a bug, since removed) |
| 43, 44 | UART0 console |
| 45 | VDD_SPI strap — HIGH at boot selects 1.8 V flash; the board will not boot |

Free for future use: GPIO **0, 3, 45**, all strap-limited. The plan reserves them for restoring a software stepper enable (§3f).

### 3c. CRSF UART — nets now named from the ESP32's side

Rev 1.0 named the CRSF nets from the receiver's perspective. Rev 2.0 names them from the ESP32's: net `TX` (GPIO17) → receiver Rx, net `RX` (GPIO18) ← receiver Tx. The planning doc recommends renaming them `CRSF_ESP_TX` / `CRSF_ESP_RX` so the ambiguity leaves the board entirely.

The superloop MVP never transmits to the receiver, so it opens the UART **RX-only** on GPIO18 and, if no valid CRSF frame arrives, also tries GPIO17 as the receive pin — which rescues a crossed harness. Both pins stay inputs throughout, so a crossed wire can never put two outputs against each other.

### 3d. Buttons — active-HIGH with external pull-downs

Common rail is **3V3** (Rev 1.0 fed 5 V into GPIOs — fault E1). Each button has a 10 kΩ pull-down, 1 kΩ series resistor and 100 nF filter (`rev2-pin-assignment.md` §1); software finishes the debounce. Firmware must configure them as **floating inputs** — an internal pull-up would fight the external pull-down. Buttons are excluded from the superloop MVP and are left untouched.

### 3e. Strapping pins, JTAG pins and devkit LEDs

- **GPIO46 carries BL_DIR.** GPIO46 is sampled at reset. The A4988 DIR input has no pull of its own, so the chip's internal pull-down wins and boot is normal. Nothing may drive BL_DIR high during reset.
- **GPIO39–42 are the JTAG pins.** They behave as GPIO because the JTAG-source strap, GPIO3, is NC. The Rev 1.0 hazard (B_SF on GPIO3, so holding the button through power-on handed four driver pins to JTAG) is gone by construction.
- **GPIO40/41/42 drive the EdgeHax S3 Pro's orange/white/green LEDs.** The final schematic puts **FR_STEP on GPIO40**, which the planning doc specifically advised against. It works electrically — the LED adds ~1 mA of load and flickers with steps — but if the front-right wheel misbehaves at high step rates, suspect this first. An Espressif DevKitC-1 has no LEDs on these pins.

### 3f. A4988 driver wiring (all four identical)

- **EN# → GND, hardwired. COM_ENA no longer exists.** The drivers cannot be disabled in software: whenever 12 V is present all four motors are energised and holding, with continuous heat and battery drain at standstill. Every failsafe that used to "drive COM_ENA disabled" now **stops generating STEP pulses** — the motors hold position rather than going limp, and only removing 12 V de-energises them. Hedge from the plan: bring EN# to GND through a 0 Ω link or jumper, so the enable can later be restored onto GPIO0/3/45 without a respin.
- **MS1/MS2/MS3/RST#/SLP#** tied to VCC → **1/16 microstepping, 3200 microsteps/rev**, which is what `mecanum_kinematics.h` assumes.
- **STEP/DIR** → per-wheel GPIOs (§3a). VDD from 5 V is in spec for 3.3 V logic drive; 3.3 V VDD removes the level mismatch (plan §7).
- **VMOT** → 12 V with **100 µF bulk + 100 nF ceramic at every driver**. Rev 1.0 placed the bulk capacitor on VDD, which is the classic way to destroy an A4988. **Pin 9 (logic GND) must be connected** — it reads as unconnected on Rev 1.0.
- **Current limit (Vref trim pot) — still not set.** Set it for the KV4239-T3B004's rated current before ever applying 12 V (§13c). With EN# hardwired this is now more urgent: the motors draw holding current the instant the battery is connected.

### 3g. Battery voltage sense (GPIO6)

Rev 2.0 divider: **R1 = 33 kΩ** (battery side) and **R2 = 10 kΩ**, both 1 %, with **100 nF at the GPIO pin**. Divisor 4.30 → 2.93 V at a full 12.6 V pack. The 7.7 kΩ source impedance suits the ADC sample-and-hold. Use `ADC_UNIT_1` / `ADC_CHANNEL_5` / `ADC_ATTEN_DB_12` with curve-fitting calibration, multisample, and filter hard (~200 ms) before acting on the value — stepper acceleration sags the rail for real.

Firmware thresholds (§9f): warn at 10.5 V, cut off at 9.9 V. **With COM_ENA gone the cutoff can no longer disable the drivers** — it can only stop stepping and alarm; the motors keep drawing holding current until the battery is unplugged. Battery sensing is excluded from the superloop MVP.

### 3h. Shared IMU interrupt line (GPIO7)

R3 (10 kΩ to 3V3), `MPU1_INT` and `MPU2_INT` meet at two junctions and run to GPIO7. Verified in the KiCad file: `I2C_SDA` and `I2C_SCL` land directly on the GPIO8/9 pin ends and are not connected to that net, despite looking adjacent in the rendered schematic.

A wired-OR only works if **both MPU6050s are configured open-drain, active-low** — `INT_PIN_CFG` (0x37) = `0xC0`. Push-pull outputs tied together fight. The superloop MVP writes that at boot, keeps the interrupt itself disabled, and polls both IMUs on the 500 Hz tick. Cost of sharing the line: the pin alone cannot say which IMU fired, so a dead IMU has to be detected from I²C behaviour (errors, frozen data) — which is how the MVP does it.

### 3i. Final schematic vs `rev2-pin-assignment.md`

The planning doc predates the finished schematic. Where they disagree, **the schematic is authoritative** and the firmware follows it:

| Net | Final schematic | Planning doc |
|---|---|---|
| FR_STEP | **GPIO40** | GPIO38 |
| FR_DIR | **GPIO38** | GPIO40 |
| BL_DIR (RL_DIR) | **GPIO46** | GPIO41 |
| B_SF | **GPIO39** | GPIO48 |
| B_LEFT | **GPIO41** | GPIO46 |
| B_RIGHT | **GPIO47** | GPIO39 |
| B_SELECT | **GPIO48** | GPIO47 |

Every other net agrees. If the schematic is ever changed back to match the planning doc, only `omnis_pins.h` needs editing.

---

## 5. Open Items / Not Yet Finalized

> **Superloop MVP — how each open item was resolved for this build:**
>
> - **Emergency kill switch:** implemented as an **arm switch** on channel 6. HIGH
>   permits motors; LOW or MID never does. Arming needs a LOW→HIGH edge that the
>   attempt consumes, so a switch left HIGH at power-on cannot arm, and the robot
>   never re-arms itself when a fault clears.
> - **Heading hold:** not implemented (out of MVP scope).
> - **IMU trim pot:** implemented as `params.balance.trim_rad`, a static offset on
>   the *lean target* — §13b's recommendation — not a gain trim. The S1 slider is
>   instead available as a live **kp** knob for tuning (`OMNIS_TUNE_KP_FROM_POT`).
> - **Button pull-downs:** not relevant here; buttons are excluded.

- **RC channel/switch semantics (§7d)** — "emergency skill switch" read as **emergency kill switch** (assumed transcription); heading-hold momentary button's hold-vs-toggle behavior; and what the IMU trim potentiometer trims (static balance-setpoint bias vs. live gain) are all still undefined. Pick these before writing the input-handling state machine, not while debugging it.


## 6. Related Files

- `setup-guides/vscode-setup.md` — fresh-install VS Code + ESP-IDF + FreeRTOS environment setup
- `setup-guides/phase0-toolchain-environment.md` — toolchain bring-up
- `setup-guides/esp32-command-reference.md` — command lookup
- **`assets/kinematics/`** — mecanum IK/FK: reference, derivation, walkthrough, and `mecanum_kinematics.{c,h}`. Fills §10.
- **`omnis-superloop-mvp/`** — the **superloop MVP firmware**: a complete, deadline build of the core (500 Hz EKF, mecanum drive, CRSF radio, RMT step generation, faults/supervisor, two-wheel balance) as a plain `while(1)` with no FreeRTOS primitives in project code. See its `README.md` for layout, `OPERATIONS_GUIDE.md` for build/architecture/tuning, `TESTING.md` for bench bring-up, and `BUILD-LOG.md` for the full history and every decision. OLED, buttons, SD, OTA and battery sensing are excluded — deferred, not abandoned, and still specified in this file.
- **`assets/control/`** — attitude EKF, dual-IMU fusion and balance PID: reference, derivation, walkthrough, `attitude_ekf.{c,h}`, `imu_fusion.{c,h}`, `pid.{c,h}`, and the as-built `omnis_imu_mounting.h`. Fills §11 and §13's implementation.

---

## 7. RC Control Scheme — RadioMaster Pocket / ExpressLRS

### 7a. Physical control inventory

Confirmed against the stock RadioMaster Pocket's hardware — 5 switches + 1 slider, no add-on module needed, exactly enough for the six functions below:

| Physical control | Type | Assigned function (per builder) |
|---|---|---|
| Left gimbal — throttle axis | analog | Throttle (flat mode) / lean-rate reference into balance controller (balance mode, see 7c) |
| Left gimbal — rudder axis | analog | Yaw rotate-in-place (flat mode) / **disabled** (balance mode) |
| Right gimbal — elevator axis | analog | Omnidirectional pitch component (flat mode) / forward-back "car" input (balance mode) |
| Right gimbal — aileron axis | analog | Omnidirectional roll/strafe component (flat mode) / turn-rate "car" input (balance mode) |
| **SE** — momentary button (top-left) | digital, momentary | Heading-hold — behavior (hold-to-engage vs. toggle) not yet decided, see 7d |
| **SA or SD** — 2-position latching switch | digital | Emergency kill switch — reading "skill" as a transcription of "kill", see 7d |
| **SA or SD** (the other one) | digital | Position hold (disables joysticks; robot just balances in place — see 7e, resolved, no odometry needed) |
| **SB or SC** — 3-position switch | digital | Drive mode: low = flat (4-wheel), mid = balance (2-wheel), high = auto-detect |
| **SB or SC** (the other one) — 3-position switch | digital | Speed limiter: low / medium / high |
| **S1** — potentiometer/slider (rear, top-right) | analog | IMU trim — trimming *what*, exactly, is undecided (see 7d) |

Ten channels used; CRSF's standard RC frame carries 16 regardless of ExpressLRS air-rate, so there's headroom for more later (e.g. a dedicated arm/status channel). Exact SA/SB/SC/SD-to-function assignment is a firmware choice, not fixed by the radio — pick it in the CRSF channel-mapping code and mirror it on the on-screen config menu (§14) so the OLED confirms current mode without needing the transmitter's own screen.

### 7b. Link — cross-reference to §3c

GPIO17 = ESP32 UART TX (feeds the ExLRS Rx module's Rx pin), GPIO18 = ESP32 UART RX (receives CRSF from the ExLRS Rx module's Tx pin) — already established in §3c, repeated here because the CRSF parser is the first thing this section depends on. Parse actual CRSF frames (start byte 0xC8, frame-type 0x16 for packed 11-bit RC channels) rather than treating the UART as a simple value stream — write and bench-test this driver before anything else here, since every mode/mixing decision below reads from it.

### 7c. Mode-dependent joystick mixing (as specified)

> **Superloop MVP:** flat mode is implemented exactly as described (throttle and
> pitch both into `vx`, roll into `vy`, yaw into `w`, one mecanum mix), with the
> speed-limiter switch scaling the sticks *before* the kinematics so every axis is
> limited equally. Balance mode is implemented with yaw disabled, pitch as a lean
> offset into the balance setpoint and roll as a turn differential — i.e. routed
> through the control loop, not applied as a raw actuator command. Throttle is
> unused while balancing.

**Flat (4-wheel holonomic) mode:**
- Throttle → forward/back thrust
- Yaw → rotate in place
- Pitch + Roll → full omnidirectional translation (strafe/diagonal)
- All four combine in the standard mecanum inverse-kinematics mix (each wheel's speed = a combination of vx, vy, yaw rate) — the actual equations are scoped out to §10.

**Balance (2-wheel Segway) mode:**
- Throttle → unchanged per spec; verify in testing it works as a lean/speed reference into the balance controller rather than a direct thrust value, since the control loop — not the stick — ultimately sets motor torque (§13).
- Yaw → disabled.
- Roll & Pitch → repurposed as differential-drive "car" controls (pitch = forward/back, roll = turn). Under active balance control these become an *offset into the balance setpoint* (§13b), not a raw actuator command — flat mode is direct kinematic mixing, balance mode routes stick input through the balance/EKF loop instead.
- Ties back to §1's active-side remapping: whichever two wheels are "down" are the ones driven by this mixing; the other two stay powered but idle.

### 7d. Open interpretation questions — resolve before coding, tracked in §5

- "Emergency skill switch" → read as emergency kill switch (originally: drive COM_ENA / GPIO1 to the disabled state. **Rev 2.0 removed COM_ENA** — EN# is hardwired to GND — so the kill switch now immediately stops all STEP generation, independent of drive mode; the motors hold position rather than going limp). If "skill" was literal, the whole channel assignment above needs revisiting.
- Heading-hold: hold-for-duration (release = free yaw) vs. single-press toggle — different firmware (level-trigger vs. edge-trigger). Pick one.
- IMU trim pot: a static bias added to the balance-angle setpoint (compensates for an off-center CG, see §13b) vs. a live gain/sensitivity trim on the EKF's control response — these are different variables in the control loop. The former is the far more common use of a trim pot on a balancing platform.

### 7e. Position-hold — resolved (confirmed by builder)

"Position hold" means the robot balances in place and does not translate — it is not a station-keeping/anti-push feature and does not require any position fix. With joysticks disabled on this switch, the drive-mixing inputs are simply zeroed, and in balance mode the existing EKF attitude/balance loop keeps the platform upright exactly as it already does moment-to-moment. No odometry, encoders, or BOM change needed — this only affects the mode/input-mixing state machine (§7c), not sensing.

### 7f. Failsafe on link loss — not in the original spec, surfacing because it matters here

CRSF/ExpressLRS failsafe behavior (no-pulses / hold-last-value / defined failsafe value) is configurable in the transmitter/receiver pairing. "Hold last value" on the drive channels is the wrong default here — if the link drops mid-deflection, the robot keeps executing that command with no radio watching it. Handle this in firmware (on CRSF frame timeout, not just the receiver's own failsafe pulses): stop all STEP generation immediately (originally "drive COM_ENA HIGH"; Rev 2.0 has no COM_ENA — see §3f). In balance mode, cutting power makes the robot fall — decide whether that's acceptable (most Segway-style bots are designed to just sit/fall safely) or whether a controlled sit-down sequence is wanted first.

### 7g. Practical firmware build order this implies

1. CRSF frame parser + channel decode (GPIO17/18) — nothing else works without it.
2. Switch/button debounce + mode-state machine (drive mode, speed limiter, kill, position-hold, heading-hold) — settle momentary-vs-toggle behavior here (7d).
3. Flat-mode mecanum mixing — bench-testable with the robot on its wheels, no balance loop needed yet.
4. Balance-mode control loop (§13) — depends on the EKF work in §11.
5. Kill-switch and link-loss failsafe wired in at the top of the control loop from the start, not bolted on last — it's a single GPIO write (shared EN# line, §3f) — *Rev 2.0: EN# is hardwired, so this is now "stop STEP generation"* and should be one of the earliest-tested pieces of the whole firmware.

---

## 8. OTA (Over-the-Air) Firmware Updates over Wi-Fi & Partition Scheme

> **Superloop MVP: EXCLUDED — deferred, not abandoned.** No Wi-Fi, no OTA, and the
> default single-app partition table. Flashing is over USB. Nothing in this section
> is implemented; it stands as the specification for the full build. Note that
> §8g's "disable `COM_ENA` before an update" no longer has a mechanism — board
> Rev 2.0 removed that line (§3f).

### 8a. Scope

Everything needed to plan the partition layout and update mechanics before writing app code — this only covers the OTA/partition side, not the Wi-Fi provisioning UI.

### 8b. Flash size (resolves the module-variant gap)

Module confirmed by the builder as **ESP32-S3-WROOM-1-N16R8** (16MB flash / 8MB Octal PSRAM) — the partition table in §8c is sized for this.

### 8c. Recommended partition table (16MB flash)

```
# Name,     Type, SubType,  Offset,   Size,   Flags
nvs,        data, nvs,      0x9000,   0x6000,
otadata,    data, ota,      0xf000,   0x2000,
phy_init,   data, phy,      0x11000,  0x1000,
factory,    app,  factory,  0x20000,  2M,
ota_0,      app,  ota_0,    ,         2M,
ota_1,      app,  ota_1,    ,         2M,
storage,    data, spiffs,   ,         1M,
```

Uses ~6MB of 16MB, leaving ~10MB free. Keep the **factory** slot even though a bare two-OTA-slot scheme is valid — a known-good image flashed once over USB that a bad wireless push can't touch. 2MB per app slot is generous headroom (a FreeRTOS + Wi-Fi + EKF image is typically well under 1.5MB even unstripped) — shrink later once real image size is known via `idf.py size`. The `storage` partition is optional; skip it if the SD card (§9) covers all persistent data needs, folding that 1M back into larger app slots instead.

Required menuconfig changes: Partition Table → Custom partition table CSV; Serial flasher config → Flash size → 16MB (must match the physical part or partitions get mis-placed); Bootloader config → enable app rollback support (§8e).

### 8d. Boot sequence

The bootloader reads `otadata` (dual 0x2000 sectors, so a power failure mid-write can't corrupt both) to determine which app partition to boot; if blank/erased, it falls back to `factory`. `esp_ota_begin()`/`esp_ota_write()`/`esp_ota_end()` target whichever OTA slot isn't currently active, and `esp_ota_set_boot_partition()` updates `otadata` once the new image is validated.

### 8e. Rollback — treat as required, not optional, for this project

Enable `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`. A newly-flashed image boots into a pending-verify state; firmware must call `esp_ota_mark_app_valid_cancel_rollback()` after a real self-test (IMUs responding, stepper drivers enumerating, CRSF link established) — not immediately on boot — or the bootloader auto-reverts to the previous working image on the next reset. That's the difference between "bad OTA push, robot reboots to last-known-good on its own" and "bad OTA push, bricked until you find a USB cable." Anti-rollback (`CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK`) is unnecessary for a single-owner hobby project — skip it.

### 8f. Use `esp_https_ota`, don't write this one from scratch

§1 commits to custom drivers "wherever reasonably feasible." OTA is the exception: `esp_https_ota` (built on `esp_ota_ops`) is exactly the kind of security-and-correctness-sensitive code where reinventing chunked-flash-write-with-resume buys risk, not portfolio value. Use the IDF component; spend from-scratch effort on the stepper drivers, EKF, and RTOS layer instead.

### 8g. Wi-Fi-specific and hardware-specific considerations

- Wi-Fi is internal RF on the WROOM-1 module — no GPIO conflicts with anything in §3.
- **Wi-Fi credentials: store in `params.json` on the SD card** (§9f, `wifi` block) — not `nvs`, not a `wifi_provisioning` flow. This project already committed to SD-card-based configuration for everything else (§9); a second settings path (SoftAP/BLE provisioning) would reintroduce the exact "two stores that can disagree" problem §9f already warns against. Trade-off: changing networks means pulling the card, not tapping through a phone app — fine for a single-owner hobby robot. `nvs` can still cache the last-used credentials for faster reconnects if wanted, but `params.json` stays the source of truth.
- **Only perform OTA while stationary, not mid-balance.** Flash erase/write can introduce scheduling jitter on the update task's core; on a two-wheel balancer relying on a tight IMU-EKF-motor timing loop, that jitter is a fall risk. Gate OTA start on drive-mode == flat (or explicitly parked) in firmware.
- Show OTA progress on the OLED (percent written) and a buzzer cue on start/success/failure — a 1–2MB write can take tens of seconds with no other indication it hasn't hung.
- Disable COM_ENA before starting the OTA write and keep it disabled through the post-OTA reboot, on top of the "parked only" gate above. *(Rev 2.0: COM_ENA no longer exists — EN# is hardwired, so the drivers stay energised through an update. This becomes "stop all stepping, require parked".)*

### 8h. Practical firmware build order this adds

1. Bring up `esp_https_ota` against a local dev HTTP server (`python -m http.server`) before any real/remote update flow.
2. Enable and test rollback with a deliberately broken test image before trusting it in the field.
3. Wire version string + build/git-hash reporting (OLED + a status query) so a field failure can confirm which image is actually running post-rollback.

---

## 9. SD Card Storage — Data Logging & JSON Parameter File

> **Superloop MVP: EXCLUDED — deferred, not abandoned.** No microSD, no FATFS, no
> logging. `params.json` is replaced by `main/omnis_params.c`: the same schema
> (§9f) as one C struct filled from compile-time defaults, deliberately passed by
> pointer everywhere so a JSON loader can drop in later without touching a single
> call site. Changing a parameter today means editing that file and reflashing.
> GPIO 10–14 are left unconnected for the card socket.

### 9a. Two independent uses on one card

1. **Data logger** — append-only, one file per run: fused attitude, commanded vs. actual wheel speeds, RC channel values, mode/state transitions, fault/error events.
2. **Parameter store** — a single human-editable JSON file holding every tunable setting (control gains, trim values, per-position speed-limiter scale factors, mecanum geometry constants, CRSF channel map, per-IMU calibration offsets, Wi-Fi credentials, battery thresholds), readable/writable with the card pulled and the robot powered off, then reloaded at next boot.

### 9b. Physical/interface — confirmed clean, four pins, no conflicts

Pin assignment confirmed directly from EdgeHax's pinout diagram: the onboard microSD slot is wired in **SPI mode**, using exactly four GPIOs — GPIO10–13. GPIO9 and GPIO14 (`FSPIHD`/`FSPIWP`) are **not** part of the SD socket wiring; they only appear in the vendor's pin-label chain because those are the chip's generic Octal/Quad-SPI-flash alternate-function names for every pin in that GPIO9–14 group. A microSD card has no real HD or WP signal in the first place, so 4-wire SPI here is exactly what you'd expect.

```
GPIO 10 = FSPICS0 (SD CS)    — confirmed
GPIO 11 = FSPID   (SD MOSI)  — confirmed
GPIO 12 = FSPICLK (SD CLK)   — confirmed
GPIO 13 = FSPIQ   (SD MISO)  — confirmed
```

No conflict with I2C SCL (GPIO9) or the buzzer (GPIO14) — both stay exactly as originally assigned in §3a. SDMMC mode is off the table (only 4 lines, no DAT0–3/CMD), so `sdmmc_host` isn't needed, only `sdspi_host`. **Rev 2.0 correction:** the EdgeHax pinout card brackets **five** microSD lines, GPIO10–14, so the buzzer did collide with the socket; it moved to GPIO16 (§3a).

### 9c. Filesystem and library choice

Use ESP-IDF's built-in `esp_vfs_fat` (FATFS) mounted over `sdspi_host`, exposed as a normal POSIX path (e.g. `/sdcard/...`). Like OTA (§8f), use the IDF component rather than writing a FAT implementation from scratch. Card format: **FAT32**, confirmed by builder.

### 9d. File layout on the card

```
/sdcard/
  params.json        <- the one file a human hand-edits
  params.json.bak     <- firmware-maintained backup of the last file that parsed successfully
  /logs/
    boot_0001.log
    boot_0002.log
    ...
```

`params.json` stays flat at the card root — not nested — so the "pull the card, edit in a text editor" workflow doesn't require hunting through folders.

### 9e. Boot log numbering

Scan `/sdcard/logs/` at boot for the highest existing `boot_NNNN.log`, use N+1. Avoids a separate counter file that could itself drift out of sync with reality; a directory listing at mount time is a one-time, cheap cost.

### 9f. params.json — schema and load behavior

Versioned, flat-ish schema so hand-edits stay easy and firmware can detect a stale/incompatible file:

```json
{
  "schema_version": 1,
  "control": {
    "speed_limit_low": 0.3,
    "speed_limit_med": 0.6,
    "speed_limit_high": 1.0,
    "heading_hold_mode": "hold_while_pressed"
  },
  "balance": {
    "ekf_bias_trim_deg": 0.0,
    "lean_pid": { "kp": 0.0, "ki": 0.0, "kd": 0.0 }
  },
  "geometry": {
    "wheel_radius_mm": 0.0,
    "wheelbase_mm": 0.0,
    "track_width_mm": 0.0
  },
  "imu_calibration": {
    "mpu_0x68": { "accel_offset": [0, 0, 0], "gyro_offset": [0, 0, 0] },
    "mpu_0x69": { "accel_offset": [0, 0, 0], "gyro_offset": [0, 0, 0] }
  },
  "rc": {
    "channel_map": {
      "throttle": 1, "pitch": 2, "roll": 3, "yaw": 4,
      "heading_hold_btn": 5, "kill_switch": 6, "position_hold": 7,
      "drive_mode": 8, "speed_limiter": 9, "imu_trim_pot": 10
    }
  },
  "wifi": { "ssid": "", "password": "" },
  "battery": { "warn_voltage": 10.5, "cutoff_voltage": 9.9, "divider_ratio": 0.2157 }
}
```

`heading_hold_mode` is where §7d's open hold-vs-toggle question actually gets resolved in practice — as a field, not a compile-time choice. `wifi` and `battery` blocks correspond to the decisions in §8g and §3g respectively.

Load behavior:
- On boot, try to parse `params.json`. If it fails to parse or `schema_version` is missing/unrecognized, fall back to `params.json.bak`; if that also fails, fall back to hardcoded firmware defaults **and** signal a fault (buzzer) rather than silently booting with zeroed-out PID gains — on a self-balancing robot, that's an immediate-fall failure mode.
- On every successful boot-time parse, copy the just-loaded file over `params.json.bak`.
- Changes made through the on-screen config menu (§14) must write back to `params.json` on the card, not just live in RAM — otherwise the hand-edit path and the menu path become two stores that can silently disagree.

### 9g. Data logger format

Line-delimited JSON (JSON Lines) rather than CSV or binary — one less parser to maintain, tolerates a torn last line on power loss (every prior line stays independently valid), and is directly greppable/tail-able off the card.

Don't flush on every line — SD writes are slow enough to stall the control loop, the same hazard as flash writes (§8g). Batch lines in a RAM ring buffer, flush on a fixed low-rate timer (200–500ms) or when the buffer fills; accept losing the last fraction of a second of log on a hard power-loss crash as the tradeoff.

### 9h. Keep SD I/O off the control loop's task/priority

Per §1's custom-RTOS design: SD writes belong on a low-priority task, never called directly from the balance control loop or the stepper STEP-pulse task/ISR. FATFS calls aren't ISR-safe and can block for tens of milliseconds. Pattern: control loop pushes a small struct into a queue; a dedicated low-priority storage task drains it and does the actual file I/O.

### 9i. Practical firmware build order this adds

1. Bring up FATFS mount over `sdspi_host` on GPIO10–13 (§9b) + read/write a test file; confirm write speed supports the intended logging rate.
2. `params.json` loader with the fallback chain in §9f — needed early, since PID/EKF gains and geometry constants come from here.
3. Storage task + queue for log writes, off the control loop's critical path.
4. Wire the on-screen menu (§14) to the same in-RAM param struct loaded from JSON, with an explicit "save to card" action that writes `params.json` first and only overwrites `.bak` after that write succeeds.

---

## 10. Mecanum Inverse-Kinematics Equations — RESOLVED

Derived, verified and implemented. Full material lives in **`assets/kinematics/`**:

| File | Contents |
|---|---|
| `mecanum-kinematics-reference.md` | Mixing matrix, IK/FK, sign conventions, corner labelling, RC channel mapping, four verified test cases |
| `mecanum-kinematics-derivation.md` | Full derivation |
| `mecanum-kinematics-code-explained.md` | Line-by-line walkthrough |
| `mecanum_kinematics.{c,h}` | Implementation — clamp, deadband, null-space slip metric |

### Wheel layout — lateral parallel rollers, NOT an X-drive

Mounting-shaft and motor constraints put the **same roller tilt on both left
wheels** and the mirrored tilt on **both right wheels**:

```
        FL  /                 \  FR          left  pair  FL, RL  ->  δ = -1
        RL  /                 \  RR          right pair  FR, RR  ->  δ = +1
```

The textbook mecanum matrix (shared tilts on the diagonals) does **not** describe
this robot. The equations below were re-derived from first principles for this
layout, not sign-flipped from the standard ones.

### The equations

```
k = (wheelbase_mm + track_width_mm) / 2        FRONT pair yaw lever
m = (wheelbase_mm - track_width_mm) / 2        REAR  pair yaw lever

ω_FL = ( vx - vy - k*w ) / wheel_radius_mm
ω_FR = ( vx + vy + k*w ) / wheel_radius_mm
ω_RL = ( vx - vy + m*w ) / wheel_radius_mm
ω_RR = ( vx + vy - m*w ) / wheel_radius_mm

f_i  = ω_i * STEPS_PER_RAD,   STEPS_PER_RAD = 3200/(2π) = 509.295818
```

Strafe splits along **sides** (left pair vs right pair), all four at equal
magnitude. Rotation splits along sides too — but with the front pair **64.7×**
the rear, so the two motions are distinguished by that ratio rather than by
their sign pattern.

Forward kinematics, in the two axle differentials `Δ_F = ω_FL − ω_FR` and
`Δ_R = ω_RL − ω_RR`:

```
vx = (r/4) * ( ω_FL + ω_FR + ω_RL + ω_RR )
vy = (r / (4L)) * ( (W - L)*Δ_F - (W + L)*Δ_R )
w  = (r / (2L)) * ( -Δ_F + Δ_R )
```

**Yaw is the difference of the axle differentials; strafe is their weighted
sum.** The X-drive had it the other way round. The slip metric
`(+ω_FL + ω_FR − ω_RL − ω_RR)` is **unchanged** — it annihilates all three
motion columns for either handedness and any geometry.

### Geometry — OMNIS as-built

`wheel_radius_mm = 30`, `wheelbase_mm = 223`, `track_width_mm = 230`
→ `k = 226.5 mm`, `m = -3.5 mm`, `k/r = 7.55`, `|k/m| = 64.7`.

Kept as named parameters (§9f `geometry` block), never inline in the formulas.
The four roller-handedness signs live in `MECANUM_DELTA_*` and are the only
statement of the layout; both lever arms are computed from them.

**What the layout costs, and what it does not.** Still fully holonomic —
`det(MᵀM) = 4L²`, non-zero for any real chassis. Peak wheel rate per unit of
commanded motion is unchanged, so top speed, strafe speed and yaw rate are all
the same as an X-drive's. What it costs is odometry precision: the strafe and
yaw columns meet at 44.1° instead of 90°, amplifying per-wheel error by **1.44×
on `vy` and 2.03× on `w`**.

**Bench test — run both directions.** Command `vx = vy` (diagonal forward-left):
both **left** wheels must be completely stationary. Then `vx = -vy`: both
**right** wheels must be. Correct side both times → handedness is right; the
other side → negate all four `MECANUM_DELTA_*` together; a *diagonal* pair →
the wheels are not in the parallel layout at all.

**Expected, not a fault:** a pure spin turns the rear wheels at 1.5% of the
front pair's rate, and below ≈0.34 rad/s of commanded yaw the deadband holds
them completely still while the front pair drives.


---

## 11. EKF — Attitude & Balance State Estimation

### 11a. Scope and approach

> **Implemented.** Code and full documentation live in **`assets/control/`** — see §6. The notes below remain the specification; the reference docs there carry the verified numbers, tuning constants and known limitations that came out of implementing it.

Two independent MPU6050s (§2, diagonally opposite corners) each run their own lightweight attitude EKF; a fusion layer above combines the two estimates and cross-checks for sensor faults. Full 3D quaternion/AHRS isn't needed — yaw isn't used by the balance controller (§13) and is left to drift; only lean angle (roll or pitch, whichever axis is currently "down") and its rate matter.

### 11b. Per-IMU EKF (run independently for U6 and U7)

4-state filter: `[roll, pitch, gyro_bias_roll, gyro_bias_pitch]`.

**Implementation note (confirmed):** because roll couples only to `bias_roll`, pitch only to `bias_pitch`, and `R` is diagonal, the 4×4 filter is block-diagonal and decomposes **exactly** into two independent 2-state filters. That is an identity, not an approximation — so no 4×4 covariance and no matrix inversion is needed anywhere. Verified tuning at 500Hz: `q_angle = 5e-4`, `q_bias = 1e-7`, `R = 1e-2`, giving a 0.80Hz accel/gyro crossover (τ = 0.198s) and 0.57° steady-state angle sigma.

- **Process model**: `roll_dot = gyro_x − bias_roll`, `pitch_dot = gyro_y − bias_pitch`; biases modeled as a slow random walk.
- **Measurement model**: accelerometer-derived tilt via `roll = atan2(ay, az)`, `pitch = atan2(−ax, sqrt(ay² + az²))`.
- **Adaptive measurement trust**: inflate the accelerometer's measurement-noise covariance whenever `|accel magnitude − 1g|` exceeds a threshold (e.g. 0.2g) — that deviation means the platform is accelerating linearly, not just tilted, and the accel-derived angle is briefly untrustworthy. This adaptive-R trick is most of what separates a good tilt EKF from a bad one on a moving robot.
- **Known limitation**: both IMUs sit at corners, not at the CG, so yaw rotation or hard acceleration adds a lever-arm centripetal/tangential component the model above ignores. Full compensation needs `a_corrected = a_measured − ω̇×r − ω×(ω×r)` per IMU — worth adding if balance quality in aggressive turns needs it, but skip for v1: the adaptive-R gating above already distrusts the accelerometer exactly when this error is largest.

### 11c. Dual-IMU fusion and side detection

- **Fused estimate**: inverse-covariance-weighted average of the two independent (roll, pitch) estimates.
- **Fault detection**: if the two IMUs disagree beyond ~15°, that's a bad mount, sensor, or cable, not something to average through — flag a fault (buzzer) and fail toward the link-loss failsafe (§7f, disable steppers) rather than balance on an untrustworthy number.

  **Compare gravity vectors, NOT Euler angles.** Balance mode sits at pitch ≈ ±90°, where roll is gimbal-locked and undefined. Verified: two IMUs at (roll 30°, pitch 88°) and (roll −30°, pitch 92°) differ by 60° of roll but only **3.46°** of actual disagreement about where "down" is — a naive `|roll_A − roll_B| > 15°` test false-faults continuously on a healthy robot. Reconstruct each IMU's gravity unit vector and take the angle between them; that metric is singularity-free at every attitude.

  **Correct the mounting rotation first (§2).** The two modules are mounted antiparallel. Uncorrected, two healthy sensors read ~20° apart and this fault fires at every boot. This is the most likely reason a correct dual-IMU implementation refuses to arm — check it before suspecting the filter.
- **"Which side is down" detection** (needed for §1's active-wheel-pair remapping): at rest (low gyro on both IMUs, accel ≈ 1g), read which body axis each IMU reports as aligned with gravity — that gives current "up," which maps to which wheel pair is grounded. Re-evaluate only at mode-entry or after a detected flip, not continuously, or it'll fight the controller mid-balance.
- **Balance-mode estimation frame (added 2026-09-15).** The flat-frame pitch cannot be the balance lean angle: `atan2(-ax, hypot(ay,az))` is confined to ±90° and **folds** at the balance point — nose-down 85° and 95° both read 85°, so which way the robot is falling is lost. Its sign survives, so it still picks the grounded pair at arming. While balancing, the accel and gyro are rotated a further 90° about Y into a frame where the pose is level, with `+X'` toward the top face in both poses, so positive lean always means "falling forward". Constants `OMNIS_IMU_BALANCE_FRAME_FRONT_DOWN` / `_REAR_DOWN` in `assets/control/omnis_imu_mounting.h`; full reasoning in `attitude-ekf-derivation.md` §7.

### 11d. Update rate

> **Superloop MVP:** 500 Hz achieved and measured on hardware — 500.0 Hz with zero
> overruns and no measurable jitter (Stage 1, 2026-09-07). The shared I²C bus is
> indeed the suspected bottleneck; the loop reports the worst per-tick I²C time
> every second, and `OMNIS_IMU_READ_ALTERNATE` halves it by reading one IMU per tick
> if needed. Each filter uses its own *measured* dt, so nothing breaks at any rate.

Target 500Hz for the EKF/balance loop — a well-established range for small, aggressive balancing robots, achievable on the ESP32-S3's dual 240MHz cores. The shared I2C bus (§3a: OLED + both MPU6050s) is the likely bottleneck, not CPU — read both IMUs every cycle and keep the OLED update off the balance loop's critical path, same task-isolation principle as §9h.

---

## 12. IMU / Sensor Calibration Procedure

> **Superloop MVP:** step 1 (gyro bias at every boot) is implemented, gated on the
> robot actually being still — gyro and accelerometer standard deviation plus a
> 1 g magnitude check, retried up to five times, and a latched fault if it never
> settles. Step 0 (mounting) is implemented as an interactive wizard. Steps 2–3
> (six-position accelerometer calibration) are **not** implemented: they need the
> persistent storage this build excludes.

Populates the `imu_calibration` block already scaffolded in `params.json` (§9f).

0. **IMU mounting orientation** (do this first — everything else assumes it). The two modules are mounted antiparallel (§2), and a wrong *global* orientation is invisible to every downstream check: the two IMUs still agree with each other, so §11c's disagreement fault stays silent while the balance controller drives the wrong body axis. Derive it rather than reading a silkscreen arrow — `imu_mount_resolve()` in `assets/control/` recovers the descriptor from two poses: chassis flat and level (pins body +Z), then tipped nose-down 15–75° (pins body +X); body +Y follows from right-handedness. One pose is not enough — a reading of `(0.5, 0, 0.866)` fits both "body X is sensor X at 30° tilt" and "body X is sensor Z at 60° tilt". Verified against all 24 right-handed mountings. Store per IMU alongside the calibration below.

   **Sign note:** with §11b's `pitch = atan2(-ax, hypot(ay,az))`, **positive pitch means nose DOWN** — the opposite of the aerospace convention. Verified: 30° nose-down reads accel `(-0.500, 0, +0.866)`, pitch `+30.0°`.

1. **Gyro bias**: with the robot stationary and level, sample each MPU6050's gyro for ~5 seconds at the target loop rate (§11d), average, store as `gyro_offset`. Worth redoing at every boot rather than trusting a stored value long-term — gyro bias drifts with temperature, and the robot has to sit still for the EKF to initialize anyway (§11c), so it costs nothing.
2. **Accelerometer offset and scale**: six-position test — lay the robot on each of its six faces in turn, recording each axis's rest reading in each orientation. Each axis should read ±1g in two positions and ≈0g in the other four; the deviation from that ideal gives bias (average of the +1g/−1g readings) and scale error (half their difference) per axis.
3. **Store both IMUs' results independently** in `mpu_0x68` / `mpu_0x69` — different physical chips, no reason to assume matching bias.
4. **Verification**: after loading calibration, confirm both IMUs agree at rest (§11c's fault threshold) and that accelerometer magnitude reads ~1g in a few arbitrary static poses, not just the six calibration positions.
5. **Re-calibration trigger**: if §11c's disagreement fault starts firing on a previously-fine robot, treat it as a calibration-drift (or mounting) signal and re-run this procedure before assuming a wiring fault.

---

## 13. Balance Control Loop

### 13a. Honest starting point: the ceiling this hardware sets

Best-performing self-balancing robots close an inner angle loop *and* an outer wheel-velocity loop using encoder feedback. This design has no wheel encoders — the A4988s are open-loop (§7e) — so true closed-loop velocity control isn't available with the current BOM. What follows is the best achievable architecture *without* encoders; it's what most hobbyist stepper-balance-bot projects actually run and it does work, but it has a real ceiling encoders would remove. If "absolute best" later means better than this architecture can deliver, a wheel encoder pair (even one axis) is the single highest-leverage hardware addition — flagging that now rather than pretending open-loop stepping is equivalent.

### 13b. Architecture: cascaded angle control with a velocity-bias outer loop

> **Implemented.** `assets/control/pid.{c,h}` — inner PID with derivative-on-measurement taken from the raw gyro and conditional-integration anti-windup, plus the outer `vel_bias_t` loop. Gains default to **zero** per §13d rather than guessing them; `pid-reference.md` §5 carries the tuning procedure. Note the outer loop's sign: forward drive effort must command a *backward* lean.

- **Inner loop (fast, 500Hz, matching §11d)**: PID on lean angle. `angle_error = target_lean_angle − fused_lean_angle` (§11c); `angle_rate` taken directly from gyro rather than differentiated from the EKF output, for lower latency. Output = commanded wheel acceleration, applied equally to both active wheels (§11c side-detection) for the fore-aft component.
- **Turn component**: differential step-rate bias between the two active wheels, from the roll-stick "turn" input (§7c), added after the fore-aft PID output.
- **Outer loop (slow, ~10–20Hz) — velocity-bias via commanded-step integration**: with no encoders there's no true velocity feedback, so use the *integral of commanded step-rate* as a proxy for sustained drive effort, and slowly bias `target_lean_angle` to pull that integral back toward zero. This is standard on encoder-less balance bots — it's what stops the robot settling into "balanced but slowly drifting across the room," and it's where CG-offset trim naturally lives: exactly what §7d's IMU-trim pot should feed (a static offset on this outer loop's target, not the raw EKF angle).
- **RC steering** enters as a bias on `target_lean_angle` (fore-aft) and the turn differential (left-right) — the standard "controlled fall" method: moving forward means commanding a small forward lean, and the inner loop's job is to prevent falling while achieving it.

### 13c. What protects against the open-loop-stepping risk

No step-loss detection is possible without encoders, so the mitigation is preventive: generous A4988 current-limit margin (§3f's Vref setting — tune for headroom, not just nominal torque) and acceleration limits in the step-rate command path, so the outer loop never asks for more than the motor can deliver without slipping. Both are cheap to get right and expensive to get wrong here, since a lost step is invisible to this architecture until the robot is visibly wrong.

### 13d. Gain tuning

> **Superloop MVP:** no gains are shipped — they default to zero and the supervisor
> **refuses to arm balance mode with zero gains**, so the robot cannot be armed into
> a mode that does nothing. `TESTING.md` §7.3–7.4 gives the bench sign check and the
> tuning order from this section, plus an optional live-kp slider so a search does
> not need a reflash per attempt.

No numeric PID gains are given here — they depend on physical parameters (mass, CG height, wheel radius) this file doesn't have, and a number offered without them would be a guess dressed up as a spec. Tune empirically: start conservative (small Kp, no Ki/Kd), test on a soft surface or a tether rig, increase Kp until oscillation appears then back off ~30%, add Kd to damp what's left, add a small Ki last (only to correct steady-state lean — too aggressive and it fights the outer velocity-bias loop in §13b).

---

## 14. On-Screen Config Menu (B_SF)

> **Superloop MVP: EXCLUDED — deferred, not abandoned.** No OLED, no buttons, no
> menu. Their GPIOs are listed as reserved in `main/omnis_pins.h` so nothing in this
> build claims them. Status comes from the once-a-second telemetry line and the
> buzzer patterns instead (`TESTING.md` §6.4).

Deferred by design — the menu tree, navigation model, and OLED layout get built once the rest of the firmware (RC, drive modes, EKF, balance, OTA, SD) is working and its actual settings surface is known, rather than designing a UI around guesses now. Every other section that references "the on-screen menu" (§2, §7a, §8g, §9f) describes what it will eventually need to expose — treat those as the requirements list when this section gets filled in.
