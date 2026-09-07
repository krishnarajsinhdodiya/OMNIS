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
