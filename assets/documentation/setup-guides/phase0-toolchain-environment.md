# Phase 0 — Toolchain & Environment (OMNIS)

Status markers used below:
- ✅ Verified — you personally ran this and confirmed the output
- ⬜ Not yet done — written from spec, do this yourself before calling Phase 0 complete

---

## 0. Two separate environments exist for this project — know which one you're in

| | VS Code Devcontainer (`.devcontainer/`) | Native macOS install |
|---|---|---|
| IDF version | v6.1-dev (dev branch) | v5.3.1 (stable) |
| Can reach physical EdgeHax S3 Pro board? | **No** — no `/dev/ttyUSB*`/`/dev/ttyACM*` passthrough configured, only Linux virtual consoles | Yes, via macOS `/dev/cu.*` |
| Purpose | QEMU emulation (`qemu-system-xtensa` is installed), isolated FreeRTOS experiments | Primary environment — all real hardware work: build, flash, monitor |
| Requires | Docker Desktop running + "Dev Containers" extension (`ms-vscode-remote.remote-containers`) installed | Nothing beyond steps below |

**Rule going forward: all real board work happens natively. The container is optional, for later QEMU experiments only.**

---

## 1. Prerequisites (native macOS)

```bash
git --version
python3 --version   # need 3.9+
```

✅ Confirmed: Python 3.11.0 present via the official python.org installer at
`/Library/Frameworks/Python.framework/Versions/3.11/`. A second install, Python 3.14, is
also present in `/Applications/`. Worth remembering if `idf.py` or `install.sh` ever
silently picks the wrong interpreter later — not an issue now, just a known fact about
this machine.

If Python/Git are missing on a fresh machine:
```bash
xcode-select --install        # gives you git
brew install python3          # prefer over relying on macOS system Python
```

---

## 2. Get the ESP-IDF SDK (separate from your project repo)

IDF is a ~2GB SDK — clone it *outside* your OMNIS project folder.

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3.1 --recursive https://github.com/espressif/esp-idf.git
```

Pinned to `v5.3.1` (a stable release branch) deliberately — not `master`/dev. The
devcontainer runs a `-dev` branch; treat the two as intentionally different, not a bug to
reconcile. Mixing them casually will produce confusing version-specific bugs.

✅ Confirmed: clone completed, `esp-idf` directory present at `~/esp/esp-idf`.

---

## 3. Run the install script (pulls the Xtensa/RISC-V toolchains)

```bash
cd ~/esp/esp-idf
./install.sh esp32s3
```

**Only install the target(s) you actually own.** `./install.sh esp32s3` installs just the
S3 toolchain. Do **not** run `./install.sh all` (or bare `./install.sh`) — that pulls
toolchains for every Espressif chip (ESP32, S2, C2, C3, C5, C6, C61, H2, P4...), multiple
extra GB, zero benefit for a single-board project. Re-run with a new target later, if
you ever add a different chip — it's additive, not a locked-in decision.

Note: even for esp32s3, `install.sh` also pulls `riscv32-esp-elf` — this is correct and
expected, not a mistake. The S3's onboard ULP (low-power coprocessor) is RISC-V-based
even though the main CPU cores are Xtensa.

### Known failure mode: SSL certificate error

```
WARNING: Download failure: <urlopen error [SSL: CERTIFICATE_VERIFY_FAILED]
certificate verify failed: unable to get local issuer certificate (_ssl.c:992)>
...
ERROR: Failed to download, and retry count has expired
```

**Not a network problem.** Python installed via the official python.org `.pkg` ships its
own bundled CA store, which requires a one-time post-install script to populate. If that
script never ran, *no* HTTPS download from that Python will verify, ever, until fixed.

Fix, in order of preference:
```bash
# 1. Find which Python.org version folder you have:
ls /Applications/ | grep -i python

# 2. Run its cert installer (adjust version number to match):
/Applications/Python\ 3.11/Install\ Certificates.command

# 3. Fallback if step 2's folder doesn't exist (e.g. Homebrew Python):
/Library/Frameworks/Python.framework/Versions/3.11/bin/python3 -m pip install --upgrade certifi
```

✅ Confirmed on this machine: `Install Certificates.command` ran clean, `certifi` upgraded
2026.5.20 → 2026.7.22, symlink + permissions updated. Re-running `install.sh esp32s3`
afterward completed successfully — full toolchain + Python venv (esptool, esp-idf-monitor,
esp-coredump, etc.) installed with no further errors.

---

## 4. Source the environment (do this **every new terminal session**)

```bash
. ~/esp/esp-idf/export.sh
idf.py --version    # should print: ESP-IDF v5.3.1
```

✅ Confirmed working in macOS Terminal.app.

**Critical, non-obvious behavior confirmed by direct experience on this project:**
Sourcing `export.sh` only affects the *current shell process*. It is not global, not
per-machine, not per-project — it's per-terminal-tab. Opening a new terminal window, a
new VS Code integrated terminal tab, a new SSH session, etc. — each one is a fresh shell
with no memory of a previous `export.sh` call. `idf.py` will report
`zsh: command not found: idf.py` in every new session until you source it again there.

This was directly observed: `idf.py build` succeeded in Terminal.app (already sourced)
while the *same command in a freshly opened VS Code integrated terminal tab* failed with
`command not found`, despite being the same machine and same project folder open in both.

✅ Automated: after the sourcing-scoping issue was directly observed (see above — worked
in Terminal.app, failed with `command not found` in a freshly opened VS Code terminal
tab), an alias was added to `~/.zshrc`:

```bash
alias get_idf=". ~/esp/esp-idf/export.sh"
```

Applied with `source ~/.zshrc`, confirmed working in a brand-new terminal via
`get_idf && idf.py --version`. This is a *shortcut*, not full silent auto-sourcing —
`get_idf` must still be typed once per new terminal session. Deliberately not going
further to auto-source on every shell launch, since that would add overhead to every
terminal opened for unrelated work, not just ESP-IDF sessions.

---

## 5. Point the toolchain at the OMNIS project

```bash
cd /Volumes/Projects/OMNIS
idf.py set-target esp32s3
```

Rewrites `sdkconfig` for the S3 target; runs a CMake configure pass. Expect a short wait.
Confirm success by checking for `Build files have been written to...` in the output, and
that `sdkconfig`'s mtime updates (`ls -la sdkconfig`).

```bash
idf.py build
```

✅ Confirmed: clean build.
```
OMNIS.bin binary size 0x32a40 bytes. Smallest app partition is 0x100000 bytes. 0xcd5c0 bytes (80%) free.
Bootloader binary size 0x5260 bytes. 0x2da0 bytes (36%) free.
Project build complete. To flash, run:
  idf.py flash
```

---

## 6. Flash + monitor on real hardware

### 6a. Identify the board's serial port

```bash
ls /dev/cu.*
```
Run once with the EdgeHax S3 Pro **unplugged**, then again **plugged in**, and diff the
two lists by eye — the new entry that appears is your port. Don't guess the name from the
USB-serial chip datasheet; confirm it directly.

### 6b. Flash and open the monitor

```bash
idf.py -p /dev/cu.<your-port-here> flash monitor
```

`Ctrl+]` exits the monitor back to the shell.

✅ Confirmed: real board flashed and booted successfully. `hello_world` example ran end
to end — bootloader loaded, partition table read, app booted, `Hello world!` printed,
heap size reported (389836 bytes free), then the example's built-in 10-second countdown
triggered a clean self-restart, repeating.

**Important distinction actually observed at this step, worth keeping:** the restart
cycle produced output that *looks* like crash-decode output but isn't:
```
rst:0xc (RTC_SW_CPU_RST),boot:0x8 (SPI_FAST_FLASH_BOOT)
Saved PC:0x40375904
--- 0x40375904: esp_restart_noos at .../esp32s3/system_internal.c:158
```
`RTC_SW_CPU_RST` = software-requested clean restart (this example calls
`esp_restart()` on purpose after its countdown). The PC resolves to `esp_restart_noos`,
the normal restart routine — a single, expected frame, not a fault. This confirmed the
monitor's symbol resolution is working, but it is **not** the same thing as seeing a real
fault. Do not mistake a clean-restart trace for a crash trace — the reset-reason string
(`RTC_SW_CPU_RST` vs something like `Guru Meditation Error: ... panic'ed`) is the
tell.

### 6c. Crash decoding — ✅ CONFIRMED

Deliberately triggered a real fault: `int *bad_ptr = NULL; *bad_ptr = 42;` inserted into
`app_main()`. Result, read and understood line by line:

```
Guru Meditation Error: Core 0 panic'ed (StoreProhibited). Exception was unhandled.
...
PC : 0x42006781 ... app_main at .../hello_world_main.c:46
...
EXCVADDR: 0x00000000
...
Backtrace: 0x4200677e:... 0x42005f0b:... 0x4200bc92:...
--- 0x4200677e: app_main at hello_world_main.c:46
--- 0x42005f0b: main_task at .../app_startup.c:206
--- 0x4200bc92: vPortTaskWrapper at .../port.c:147
```

Key reading skills confirmed:
- `StoreProhibited` = illegal **write** (vs `LoadProhibited` = illegal read)
- `EXCVADDR` = the actual faulting address — `0x00000000` here confirmed this was
  genuinely the planted null-deref, not something else
- `PC` resolves directly to the source file/line of the fault when `idf.py monitor` has
  the matching `.elf` — no manual `addr2line` needed in the common case
- Backtrace reads bottom-to-top as a call stack: `vPortTaskWrapper` (FreeRTOS's generic
  task entry trampoline) → `main_task` (ESP-IDF's wrapper that invokes `app_main`) →
  `app_main` (the actual fault site). A deeper bug would show more frames.

**The distinguishing pattern vs. a clean restart (§6b), now confirmed by direct
comparison:** a real panic shows `Guru Meditation Error:` and a populated `EXCVADDR`
before any reset-reason line; a clean restart shows only a reset-reason line
(`RTC_SW_CPU_RST` etc.) resolving to an intentional function like `esp_restart_noos`,
with no panic block and no `EXCVADDR` at all. This is the check to run first on any
future unexpected reboot, before assuming anything else.

Crash line removed from `hello_world_main.c` after this exercise — file is back to the
stock example.

---

## 7. Deferred / explicitly not covered yet

These were named as Phase 0 scope but intentionally not exercised yet — listed so they
aren't silently forgotten, not because they're low priority:

- **6c above (real crash/panic + backtrace)** — highest priority remaining item, next
  session
- `menuconfig` navigation (`idf.py menuconfig`) — component config, FreeRTOS-specific
  settings (tick rate, minimal stack size)
- `idf.py monitor` keyboard shortcuts beyond `Ctrl+]`
- `xtensa-esp32s3-elf-gdb` / OpenOCD JTAG debugging (awareness only for now)
- `esptool.py` direct usage (`chip_id`, `flash_id`, reading partition tables) — separate
  from `idf.py flash`, which wraps it
- Git `.gitignore` conventions specific to ESP-IDF (`build/`, `sdkconfig.old`,
  `managed_components/`) — partially covered by the project's existing `.gitignore`,
  not yet reviewed line-by-line
- Vendoring the FreeRTOS-LTS kernel as a custom component (optional/advanced, shelved —
  reading kernel source via the installed copy at `$IDF_PATH/components/freertos/`
  achieves the same learning goal with no version-drift risk)

See `esp32-command-reference.md` (separate file) for a standing lookup of commands for
day-to-day board interfacing — this file stays a narrative/status record, that one is
the quick-reference.

---

## 8. Environment fingerprint (for future debugging reference)

- Machine: MacBook Air, macOS, Apple Silicon (arm64)
- **Native IDF: v6.0.2**, at `~/.espressif/v6.0.2/esp-idf` (not `~/esp/esp-idf` — that
  was an earlier v5.3.1 install, since removed; see §9 below for the full story)
- Project path: `/Volumes/Projects/OMNIS`
- Python: three installs present — 3.11.0, 3.14.0, and system/homebrew variants.
  `get_idf` alias explicitly forces `/Library/Frameworks/Python.framework/Versions/3.14/bin`
  to the front of PATH before sourcing `export.sh` — required, not optional, because
  plain `which python3` resolves 3.11 first on this machine and ESP-IDF v6.0.2's tooling
  needs the 3.14 venv specifically. See §9 for why this matters.
- Devcontainer IDF (separate, QEMU-only, untouched by any of this): v6.1-dev, target
  `esp32s3`, `qemu-system-xtensa` present at
  `/opt/esp/tools/qemu-xtensa/esp_develop_9.2.2_20260417/qemu/bin/`

## 9. Post-mortem: the v5.3.1 → v6.0.2 migration (read before repeating this)

What actually happened, briefly, because the specific failure modes are worth
remembering even though the end state is simple:

1. Discovered a **second, pre-existing IDF install** at `~/.espressif/v6.0.2/esp-idf`
   (dated Aug 10, predating this project's setup — almost certainly placed there
   silently by the VS Code Espressif extension at some point) was winning over the
   native `~/esp/esp-idf` v5.3.1 install on PATH.
2. Both installs share the same underlying tool cache (`~/.espressif/tools/`), and
   running `install.sh` for one can overwrite tool versions the other depends on —
   this produced `"tool X has no installed versions"` errors that looked like a broken
   install but were actually a version collision between the two checkouts.
3. Verified v6.0.2 as genuinely functional (clean `install.sh`, clean `export.sh`
   activation, clean project build) **before** deleting v5.3.1 — the correct order,
   confirmed working before removing the fallback.
4. After switching the `get_idf` alias to point at v6.0.2 and deleting `~/esp/esp-idf`,
   `export.sh` started failing in **every fresh terminal** with a missing-venv error —
   traced to `export.sh` picking up `python3.11` (first on this machine's PATH) instead
   of the `python3.14` that v6.0.2's tools were actually installed against. This is a
   different failure from #2 — not a version collision, a Python-interpreter-selection
   bug specific to how `export.sh` resolves `python3`.
5. Fixed durably by making the `get_idf` alias itself prepend the correct Python 3.14
   path before sourcing `export.sh`, rather than relying on the system default — see the
   current alias definition in §4 above.

**The lesson worth keeping, generalized:** a script that "just runs `python3`" is
implicitly depending on whatever your system's default PATH ordering happens to
resolve that to — which may silently differ from the interpreter that script's own
dependencies were actually installed against. If a `command not found` or
`venv not found`-style error reappears after a fix seemed to work, suspect the fix was
session-scoped (see the diagnostic playbook's Class 4) rather than assuming the earlier
fix failed to take effect at all.
