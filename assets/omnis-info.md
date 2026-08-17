# OMNIS — Project Info File

**Name:** OMNIS — **O**mnidirectional **M**obility and **N**on-holonomic **I**nertial **S**tabilization

---

## 1. Project Goal

A four-wheel car, each wheel driven by its own stepper motor, built as a personal embedded-systems portfolio piece. Two headline capabilities:

1. **Holonomic drive** — Magnum (angled-roller/Mecanum-style) wheels give full directional control: strafe, rotate in place, move diagonally, without needing to turn first.
2. **Self-balancing mode** — the car can also tip up and balance/drive on two wheels, Segway-style, in addition to normal 4-wheel driving.

**Balance-mode wheel behavior (confirmed):** all four steppers stay powered and active during 2-wheel balance mode — none are mechanically or electrically disengaged. This is deliberate: the platform needs to be able to balance on *either* side, so if it's flipped mid-run the "new" pair of wheels needs to be immediately drivable without a mode-switch delay. Firmware/control implication: the balance controller needs to know (or detect) which side is currently down and re-map which two wheels are "active" for balance thrust vs. which two are along for the ride, rather than assuming a fixed pair.

The name is deliberately left open-ended (not tied to "car" or "stepper" specifically) since this platform may extend into further omnidirectional-wheel work down the line.

**Software philosophy:** written from scratch by the builder, with light AI assistance only — not vibe-coded. Deliberately chosen stack:

- Bare-metal-style programming on the ESP32-S3 (not just calling high-level Arduino libraries)
- A custom RTOS layer, built on top of FreeRTOS (which ships inside ESP-IDF) — see `vscode-setup.md` §8 for the learning path
- Custom peripheral drivers rather than pulling in ready-made libraries wherever reasonably feasible
- An EKF (Extended Kalman Filter) fusing the two IMUs for attitude/balance estimation

---

## 2. Components

| Module | Part | Notes |
|---|---|---|
| Wheel motors (×4) | Nidec Servo Corporation **KV4239-T3B004** (OEM cross-ref FK2-7586) | 2-phase hybrid stepper, NEMA17-class (42mm), KV42 series |
| Stepper drivers (×4) | **A4988** | One per motor, U2–U5. RST#/SLP# tied to VCC — resolved, no longer floating |
| Wheels (×4) | **Magnum wheels** (omnidirectional, angled-roller/Mecanum-style) | Angled rollers around the rim enable true sideways/diagonal movement for holonomic drive |
| MCU module | **ESP32-S3-WROOM-1-N16R8** (16MB flash / 8MB Octal PSRAM) on EdgeHax S3 Pro dev board | N16R8 confirmed by builder. PSRAM conflict resolved — FR_STEP/FR_DIR/RR_DIR/B_DOWN moved off GPIO35–38 onto GPIO39–42 (§3a/§3b), so module PSRAM variant (Octal vs Quad) no longer matters for this design. |
| Dev board | **EdgeHax S3 Pro** | Schematic symbol: `ESP32-S3PRO-DEVKIT-edgehax` |
| IMUs (×2) | **MPU6050** (U6, U7) | Mounted at diagonally opposite corners for redundant/fused attitude sensing. AD0 address collision resolved: U6 = 0x68, U7 = 0x69 |
| Display | **SSD1306**-based 128×64 I2C OLED (schematic symbol `DISPLAY-OLED-128X64-I2C`, designator G$1) | Controller part confirmed by builder — schematic still uses the generic OLED symbol, which is fine since the symbol doesn't need to change; drive it as SSD1306 |
| RC link | **RadioMaster Pocket** TX + **ExpressLRS Nano Rx** receiver (U10) | CRSF over UART — see §3c for the TX/RX crossover |
| Buzzer | **Buzzer Module** (U9, 3-pin: VCC / IO / GND) | Has its own driver IC onboard (not a bare piezo) — IO pin is a logic-level control signal |
| Buttons | 6× tactile, 4-pin (K4-6×6_TH) — Up / Down / Left / Right / Select / Special-function (B_SF) | Special-function opens an on-screen config/setup menu on the display (EdgeTX-menu-style, §14). Wired **active-HIGH** |
| Battery | 3S LiPo (schematic power-in header labeled "12V", nominal for 3S) | Powers whole system via 2-pin header H5. **No dedicated power switch planned** — board is live whenever the battery is connected; power-off = unplug. Voltage sensed via a divider on GPIO6 — see §3g |
| Regulator | **L7805CV** (U8) | INPUT ← battery rail, OUTPUT → regulated 5V logic rail (net "VCC" throughout the schematic) |
| Capacitors | 9× 150µF electrolytic (U11–U19) | One per: I2C/OLED rail, each MPU6050 (×2), each A4988 logic rail (×4), ESP32 5V input, main battery input. No voltage rating specified in the schematic — pick per rail (25V+ for the 12V/battery side, lower is fine for the 5V logic side) |


## 3. ESP32-S3PRO GPIO Pinout — verified against schematic rev 1.0 (as revised 2026-08-16)

### 3a. Connected / used pins

| GPIO | Net label | Connects to | Notes |
|---|---|---|---|
| 1 | COM_ENA | EN# on **all four** A4988 drivers (shared) | Single enable line for all steppers |
| 2 | B_RIGHT | Right button | Active-HIGH |
| 3 | B_SF | Special-function button | Active-HIGH. |
| 4 | FL_STEP | Front-Left A4988 STEP | |
| 5 | FL_DIR | Front-Left A4988 DIR | |
| 6 | BATT_SENSE | Battery voltage divider (3S LiPo → 0–3.3V) | ADC1_5 — see §3g for divider values and cutoff thresholds |
| 8 | SDA | I2C bus (shared: OLED + both MPU6050s) | |
| 9 | SCL | I2C bus (shared) | Not part of the microSD wiring — confirmed against the board's pinout diagram; the SD socket only uses GPIO10–13 (§9b) |
| 10 | SD_CS | microSD slot, `FSPICS0` | SD card SPI chip-select — confirmed from EdgeHax pinout diagram (§9b) |
| 11 | SD_MOSI | microSD slot, `FSPID` | SD card SPI data-out (host→card) — confirmed from EdgeHax pinout diagram (§9b) |
| 12 | SD_CLK | microSD slot, `FSPICLK` | SD card SPI clock — confirmed from EdgeHax pinout diagram (§9b) |
| 13 | SD_MISO | microSD slot, `FSPIQ` | SD card SPI data-in (card→host) — confirmed from EdgeHax pinout diagram (§9b) |
| 14 | BUZZ | Buzzer Module IO | Not part of the microSD wiring — confirmed against the board's pinout diagram; the SD socket only uses GPIO10–13 (§9b) |
| 15 | RL_STEP | Rear-Left A4988 STEP | |
| 16 | RL_DIR | Rear-Left A4988 DIR | |
| 17 | (net "RX") | ExLRS Rx **Tx** pin | ESP32 role = UART **TX** (sends to receiver) — see §3c |
| 18 | (net "TX") | ExLRS Rx **Rx** pin | ESP32 role = UART **RX** (receives CRSF from receiver) — see §3c |
| 21 | RR_STEP | Rear-Right A4988 STEP | |
| 39 (MTCK) | B_DOWN | Down button | Active-HIGH. Moved here from GPIO35 on 2026-08-16 to clear the PSRAM-conflict pins (§3b) |
| 40 (MTDO) | FR_DIR | Front-Right A4988 DIR | |
| 41 (MTDI) | FR_STEP | Front-Right A4988 STEP | |
| 42 (MTMS) | RR_DIR | Rear-Right A4988 DIR | |
| 45 | B_SELECT | Select button | Active-HIGH. |
| 47 | B_LEFT | Left button | Active-HIGH |
| 48 | B_UP | Up button | Active-HIGH |

### 3b. Unconnected / spare pins (per revised schematic)

GPIO 0, 7, 35, 36, 37, 38, 43 (U0TX), 44 (U0RX), 46, 19 (USB D−), 20 (USB D+). Also both 3V3 pins and RST are unconnected/not wired to anything in this schematic. Two of the board's GND pins on the right side and one on the top-right are likewise unwired stubs (the board's other GND pins do the actual return-current work).

GPIO 6 — previously listed here — is now the battery-voltage sense line (§3g). GPIO 10, 11, 12, 13 — also previously listed here — are now claimed by the onboard microSD slot (SPI CS/MOSI/CLK/MISO, §9b); both moved to §3a.

GPIO35–38 are now the spare pins (freed up by the move to 39–42, detailed above), a reversal from the original layout where 35–38 were used and 39–42 were spare.

Note GPIO43/44 (the chip's default UART0 TX/RX, normally used for USB-serial flashing/monitor) are spare here — confirms ExLRS is on a separate UART (17/18), not sharing the flashing port.

### 3c. ExpressLRS UART — net names are from the receiver's perspective

The schematic net called **"RX"** connects to the ExLRS Rx module's own **Rx** input pin, and the net called **"TX"** connects to its **Tx** output pin. Since the receiver's Tx must feed the ESP32's RX, and the receiver's Rx must be fed by the ESP32's TX, the roles are the **reverse** of the label when you configure the ESP32's UART:

- **GPIO17** (on the "RX" net) → configure as ESP32 UART **TX** in firmware
- **GPIO18** (on the "TX" net) → configure as ESP32 UART **RX** in firmware

This is a common gotcha — worth a code comment when you set up the UART driver.

### 3d. Buttons are active-HIGH

All six buttons wire one leg to VCC and the other leg out to the GPIO net. Pressed = pin pulled to VCC (HIGH). Idle = pin floats unless you add a pull-down. **You'll need internal pull-downs enabled (or external pull-down resistors)** on all six button GPIOs — the opposite of the "active-low with internal pull-ups" assumption in earlier notes. **Still open** — internal vs. external pull-downs not yet decided (see §5).

### 3e. Strapping pins reused as button/driver inputs

GPIO3 (B_SF) and GPIO45 (B_SELECT) are two of the ESP32-S3's four strapping pins (sampled only at reset/boot). Since these buttons idle LOW (with a pull-down) and are only pulled HIGH when physically pressed, this should be safe in normal use — just avoid holding Special-Function or Select down while power-cycling the board, since that could alter boot behavior (GPIO45 in particular affects VDD_SPI voltage selection).

Related consideration: GPIO39–42 are the chip's default JTAG pins (MTCK/MTDO/MTDI/MTMS), and whether they behave as plain GPIO depends on the JTAG signal-source strap, which is GPIO3 (B_SF) itself. Since B_SF idles low, JTAG stays on its USB-JTAG default and 39–42 behave as ordinary GPIO in normal operation — the only failure mode is holding B_SF down through a power-cycle, the same caution as above, now extended to four load-bearing driver pins instead of just spares.

### 3f. A4988 driver wiring (all four identical: U2–U5)

- **EN#** ← COM_ENA (shared)
- **MS1/MS2/MS3/RST#/SLP#** — all five bridged together and tied to VCC (5V logic), which fixes microstepping at **1/16** (MS1=MS2=MS3=HIGH per the A4988 truth table) — 200 full steps/rev × 16 = 3200 microsteps/rev. Referenced by §10's kinematics prompt.
- **STEP/DIR** → per-wheel GPIOs (§3a)
- **VMOT/GND** → battery rail / GND (motor power)
- **VDD/GND** → 5V logic rail / GND (driver logic power), each with its own 150µF decoupling cap
- **2B/2A/1A/1B** (motor coil outputs) → 4-pin header per driver (H1–H4) going to the physical motor connector
- **Current-limit (Vref) trim pot** — not yet set. Each A4988 has an onboard pot setting `Imax = Vref / (8 × Rsense)`; get this wrong and you either starve the motors of torque or overheat a driver/motor on first power-up. Set per the KV4239-T3B004's rated current before ever spinning a wheel — see §13c for why generous margin here also matters for balance-mode reliability.

### 3g. Battery voltage sense (GPIO6)

3S LiPo pack voltage sensed via a resistive divider into GPIO6 (ADC1_5 — deliberately an ADC1 pin rather than ADC2, since ADC2 is the one that gets unreliable while Wi-Fi is active on this chip family; this sidesteps that for free).

Divider: **R_top = 120kΩ** (battery+ side) to **R_bottom = 33kΩ** (GPIO6 to GND), ratio ≈ 0.216. At a full 3S charge (12.6V) that's ~2.72V at the pin — safely under the 3.3V ADC ceiling with margin for an overvoltage fault; at a near-empty pack (9V) it's ~1.94V, still well inside the ADC's usable range. Add a 100nF cap across R_bottom to filter switching noise from the nearby steppers.

Use ESP-IDF's ADC calibration API (`adc_cali_create_scheme_curve_fitting` on ESP32-S3) rather than raw ADC counts — the raw-to-voltage mapping is non-linear enough on this chip to matter for a threshold-triggered cutoff.

Firmware thresholds (standard 3S LiPo guidance, stored in `params.json` → `battery`, §9f): warn at 3.5V/cell (10.5V pack, buzzer chirp), hard cutoff at 3.3V/cell (9.9V pack, force COM_ENA disable regardless of drive mode) — over-discharging a LiPo is a safety issue, not just a performance one.

---

## 5. Open Items / Not Yet Finalized

- **RC channel/switch semantics (§7d)** — "emergency skill switch" read as **emergency kill switch** (assumed transcription); heading-hold momentary button's hold-vs-toggle behavior; and what the IMU trim potentiometer trims (static balance-setpoint bias vs. live gain) are all still undefined. Pick these before writing the input-handling state machine, not while debugging it.


## 6. Related Files

- `vscode-setup.md` — fresh-install VS Code + ESP-IDF + FreeRTOS environment setup for this project

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

- "Emergency skill switch" → read as emergency kill switch (immediately drive COM_ENA / GPIO1 to the disabled state, independent of drive mode). If "skill" was literal, the whole channel assignment above needs revisiting.
- Heading-hold: hold-for-duration (release = free yaw) vs. single-press toggle — different firmware (level-trigger vs. edge-trigger). Pick one.
- IMU trim pot: a static bias added to the balance-angle setpoint (compensates for an off-center CG, see §13b) vs. a live gain/sensitivity trim on the EKF's control response — these are different variables in the control loop. The former is the far more common use of a trim pot on a balancing platform.

### 7e. Position-hold — resolved (confirmed by builder)

"Position hold" means the robot balances in place and does not translate — it is not a station-keeping/anti-push feature and does not require any position fix. With joysticks disabled on this switch, the drive-mixing inputs are simply zeroed, and in balance mode the existing EKF attitude/balance loop keeps the platform upright exactly as it already does moment-to-moment. No odometry, encoders, or BOM change needed — this only affects the mode/input-mixing state machine (§7c), not sensing.

### 7f. Failsafe on link loss — not in the original spec, surfacing because it matters here

CRSF/ExpressLRS failsafe behavior (no-pulses / hold-last-value / defined failsafe value) is configurable in the transmitter/receiver pairing. "Hold last value" on the drive channels is the wrong default here — if the link drops mid-deflection, the robot keeps executing that command with no radio watching it. Handle this in firmware (on CRSF frame timeout, not just the receiver's own failsafe pulses): drive COM_ENA HIGH immediately. In balance mode, cutting power makes the robot fall — decide whether that's acceptable (most Segway-style bots are designed to just sit/fall safely) or whether a controlled sit-down sequence is wanted first.

### 7g. Practical firmware build order this implies

1. CRSF frame parser + channel decode (GPIO17/18) — nothing else works without it.
2. Switch/button debounce + mode-state machine (drive mode, speed limiter, kill, position-hold, heading-hold) — settle momentary-vs-toggle behavior here (7d).
3. Flat-mode mecanum mixing — bench-testable with the robot on its wheels, no balance loop needed yet.
4. Balance-mode control loop (§13) — depends on the EKF work in §11.
5. Kill-switch and link-loss failsafe wired in at the top of the control loop from the start, not bolted on last — it's a single GPIO write (shared EN# line, §3f) and should be one of the earliest-tested pieces of the whole firmware.

---

## 8. OTA (Over-the-Air) Firmware Updates over Wi-Fi & Partition Scheme

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
- Disable COM_ENA before starting the OTA write and keep it disabled through the post-OTA reboot, on top of the "parked only" gate above.

### 8h. Practical firmware build order this adds

1. Bring up `esp_https_ota` against a local dev HTTP server (`python -m http.server`) before any real/remote update flow.
2. Enable and test rollback with a deliberately broken test image before trusting it in the field.
3. Wire version string + build/git-hash reporting (OLED + a status query) so a field failure can confirm which image is actually running post-rollback.

---

## 9. SD Card Storage — Data Logging & JSON Parameter File

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

No conflict with I2C SCL (GPIO9) or the buzzer (GPIO14) — both stay exactly as originally assigned in §3a. SDMMC mode is off the table (only 4 lines, no DAT0–3/CMD), so `sdmmc_host` isn't needed, only `sdspi_host`.

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

## 10. Mecanum Inverse-Kinematics Equations — derive externally

Left as a prompt rather than derived here — hand the block below to a separate chat session, then paste the result back in to replace this section once you have it.

```
I'm building a 4-wheel holonomic robot using Mecanum-style ("Magnum") wheels, one
NEMA17-class 2-phase stepper per wheel, driven by A4988 drivers in fixed 1/16
microstepping (MS1/MS2/MS3 all tied HIGH), 200 full steps/revolution motors
(1.8°/step) — so 3200 microsteps/revolution.

Standard 4-wheel Mecanum layout: front-left, front-right, rear-left, rear-right,
rollers at 45° in an X-pattern (confirm the sign convention in your derivation).

Please derive, symbolically, parameterized by wheel_radius_mm, wheelbase_mm
(front-to-back wheel-center distance), and track_width_mm (left-to-right
wheel-center distance) — these exact variable names, since they're already fields
in my firmware's params.json:

1. Inverse kinematics: given desired body-frame velocities vx (forward/back), vy
   (strafe left/right), and yaw rate ω (rotate in place), compute each wheel's
   required angular velocity (rad/s), then convert to A4988 step-pulse frequency
   (microsteps/sec) using the 3200-microsteps/rev figure above.
2. Forward kinematics (odometry): given the four wheels' commanded step rates,
   compute the resulting body-frame vx, vy, ω. I have no encoders (open-loop
   steppers), so this is a commanded-velocity estimate, not a measurement — it
   feeds a dead-reckoning / velocity-bias control loop elsewhere in my firmware.
3. Sign conventions clearly stated per wheel (FL/FR/RL/RR), matching this RC
   input mapping I've already fixed: throttle → vx, yaw stick → ω, pitch+roll
   together → vy and secondary vx.
4. A short worked numeric example with placeholder values (e.g.
   wheel_radius_mm=30, wheelbase_mm=200, track_width_mm=180) so I can sanity-check
   my implementation.

Output as clean equations plus a small C-style pseudocode block I can drop
directly into an ESP-IDF/FreeRTOS firmware function.
```

---

## 11. EKF — Attitude & Balance State Estimation

### 11a. Scope and approach

Two independent MPU6050s (§2, diagonally opposite corners) each run their own lightweight attitude EKF; a fusion layer above combines the two estimates and cross-checks for sensor faults. Full 3D quaternion/AHRS isn't needed — yaw isn't used by the balance controller (§13) and is left to drift; only lean angle (roll or pitch, whichever axis is currently "down") and its rate matter.

### 11b. Per-IMU EKF (run independently for U6 and U7)

4-state filter: `[roll, pitch, gyro_bias_roll, gyro_bias_pitch]`.

- **Process model**: `roll_dot = gyro_x − bias_roll`, `pitch_dot = gyro_y − bias_pitch`; biases modeled as a slow random walk.
- **Measurement model**: accelerometer-derived tilt via `roll = atan2(ay, az)`, `pitch = atan2(−ax, sqrt(ay² + az²))`.
- **Adaptive measurement trust**: inflate the accelerometer's measurement-noise covariance whenever `|accel magnitude − 1g|` exceeds a threshold (e.g. 0.2g) — that deviation means the platform is accelerating linearly, not just tilted, and the accel-derived angle is briefly untrustworthy. This adaptive-R trick is most of what separates a good tilt EKF from a bad one on a moving robot.
- **Known limitation**: both IMUs sit at corners, not at the CG, so yaw rotation or hard acceleration adds a lever-arm centripetal/tangential component the model above ignores. Full compensation needs `a_corrected = a_measured − ω̇×r − ω×(ω×r)` per IMU — worth adding if balance quality in aggressive turns needs it, but skip for v1: the adaptive-R gating above already distrusts the accelerometer exactly when this error is largest.

### 11c. Dual-IMU fusion and side detection

- **Fused estimate**: inverse-covariance-weighted average of the two independent (roll, pitch) estimates.
- **Fault detection**: if the two IMUs disagree beyond ~15°, that's a bad mount, sensor, or cable, not something to average through — flag a fault (buzzer) and fail toward the link-loss failsafe (§7f, disable steppers) rather than balance on an untrustworthy number.
- **"Which side is down" detection** (needed for §1's active-wheel-pair remapping): at rest (low gyro on both IMUs, accel ≈ 1g), read which body axis each IMU reports as aligned with gravity — that gives current "up," which maps to which wheel pair is grounded. Re-evaluate only at mode-entry or after a detected flip, not continuously, or it'll fight the controller mid-balance.

### 11d. Update rate

Target 500Hz for the EKF/balance loop — a well-established range for small, aggressive balancing robots, achievable on the ESP32-S3's dual 240MHz cores. The shared I2C bus (§3a: OLED + both MPU6050s) is the likely bottleneck, not CPU — read both IMUs every cycle and keep the OLED update off the balance loop's critical path, same task-isolation principle as §9h.

---

## 12. IMU / Sensor Calibration Procedure

Populates the `imu_calibration` block already scaffolded in `params.json` (§9f).

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

- **Inner loop (fast, 500Hz, matching §11d)**: PID on lean angle. `angle_error = target_lean_angle − fused_lean_angle` (§11c); `angle_rate` taken directly from gyro rather than differentiated from the EKF output, for lower latency. Output = commanded wheel acceleration, applied equally to both active wheels (§11c side-detection) for the fore-aft component.
- **Turn component**: differential step-rate bias between the two active wheels, from the roll-stick "turn" input (§7c), added after the fore-aft PID output.
- **Outer loop (slow, ~10–20Hz) — velocity-bias via commanded-step integration**: with no encoders there's no true velocity feedback, so use the *integral of commanded step-rate* as a proxy for sustained drive effort, and slowly bias `target_lean_angle` to pull that integral back toward zero. This is standard on encoder-less balance bots — it's what stops the robot settling into "balanced but slowly drifting across the room," and it's where CG-offset trim naturally lives: exactly what §7d's IMU-trim pot should feed (a static offset on this outer loop's target, not the raw EKF angle).
- **RC steering** enters as a bias on `target_lean_angle` (fore-aft) and the turn differential (left-right) — the standard "controlled fall" method: moving forward means commanding a small forward lean, and the inner loop's job is to prevent falling while achieving it.

### 13c. What protects against the open-loop-stepping risk

No step-loss detection is possible without encoders, so the mitigation is preventive: generous A4988 current-limit margin (§3f's Vref setting — tune for headroom, not just nominal torque) and acceleration limits in the step-rate command path, so the outer loop never asks for more than the motor can deliver without slipping. Both are cheap to get right and expensive to get wrong here, since a lost step is invisible to this architecture until the robot is visibly wrong.

### 13d. Gain tuning

No numeric PID gains are given here — they depend on physical parameters (mass, CG height, wheel radius) this file doesn't have, and a number offered without them would be a guess dressed up as a spec. Tune empirically: start conservative (small Kp, no Ki/Kd), test on a soft surface or a tether rig, increase Kp until oscillation appears then back off ~30%, add Kd to damp what's left, add a small Ki last (only to correct steady-state lean — too aggressive and it fights the outer velocity-bias loop in §13b).

---

## 14. On-Screen Config Menu (B_SF)

Deferred by design — the menu tree, navigation model, and OLED layout get built once the rest of the firmware (RC, drive modes, EKF, balance, OTA, SD) is working and its actual settings surface is known, rather than designing a UI around guesses now. Every other section that references "the on-screen menu" (§2, §7a, §8g, §9f) describes what it will eventually need to expose — treat those as the requirements list when this section gets filled in.
