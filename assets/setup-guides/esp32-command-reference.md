# ESP32-S3 / ESP-IDF Command Reference (OMNIS)

Quick lookup, not a tutorial. Assumes native macOS install at `~/esp/esp-idf`, project at
`/Volumes/Projects/OMNIS`. See `phase0-toolchain-environment.md` for the setup story and
gotchas behind these commands.

## Every new terminal session — do this first

```bash
get_idf              # alias for: . ~/esp/esp-idf/export.sh
idf.py --version      # sanity check — should print: ESP-IDF v5.3.1
```
Required once per terminal tab/window/SSH session. Not persistent, not global — see
Phase 0 doc §4 for why.

---

## Finding the board

```bash
ls /dev/cu.*                    # list all serial devices
```
Run with board unplugged, then plugged in, diff the two lists. The new entry is your
port. Typical patterns:
- `/dev/cu.SLAB_USBtoUART` — CP210x-based boards
- `/dev/cu.usbserial-*` or `/dev/cu.wchusbserial*` — CH340-based boards

Once known, worth exporting for convenience in a session:
```bash
export ESPPORT=/dev/cu.<your-port>     # lets you drop -p <port> from idf.py commands below
```
(Session-scoped too — same caveat as `get_idf`.)

---

## Build / flash / monitor

```bash
idf.py build                          # compile only, no flash
idf.py -p $ESPPORT flash              # flash only, no monitor
idf.py -p $ESPPORT monitor            # open serial monitor only (board must already be flashed)
idf.py -p $ESPPORT flash monitor      # flash, then immediately open monitor — most common
idf.py -p $ESPPORT app-flash          # flash ONLY the app partition, skip bootloader/partition-table — faster iteration once those are stable
```

Inside the monitor:
- `Ctrl+]` — exit monitor, return to shell
- `Ctrl+T` then `Ctrl+H` — show monitor's own help/shortcut menu (chord, not simultaneous)
- `Ctrl+T` then `Ctrl+R` — reset the board without leaving the monitor

---

## Reconnecting to an already-flashed board

If the board is already flashed and running (or was power-cycled, restarted itself, etc.)
and you just want to see its serial output without reflashing:

```bash
idf.py -p $ESPPORT monitor
```
No `build` or `flash` needed — this just opens the serial connection. Safe to run
repeatedly; won't disturb what's already running on the chip.

To force a fresh boot without reflashing (e.g. after a monitor session where you want to
see the startup log again from the top):
```bash
idf.py -p $ESPPORT monitor          # then Ctrl+T, Ctrl+R inside it to trigger a reset
```

---

## Cleaning / reconfiguring

```bash
idf.py fullclean                # wipes build/ entirely — use when switching targets or after suspicious build errors
idf.py reconfigure              # re-runs CMake configure step without a full clean
idf.py set-target esp32s3       # (re)writes sdkconfig for the S3 — only needed once per project, or after fullclean
```

---

## Inspecting the build

```bash
idf.py size                     # summary: flash/RAM usage totals
idf.py size-components          # breakdown by component (freertos, lwip, your main app, etc.)
idf.py size-files               # breakdown by individual source file
idf.py docs                     # opens relevant ESP-IDF docs page in browser
```

---

## menuconfig (sdkconfig editor)

```bash
idf.py menuconfig
```
Relevant categories for this project (see Phase 0 doc §7 for what's not yet explored
here):
- `Component config → ESP PSRAM` — enable, N16R8 variant has 8MB PSRAM
- `Component config → FreeRTOS` — tick rate, task stack sizing defaults
- `Partition Table` — flash layout for firmware vs. logging/data storage

---

## esptool.py — lower-level than idf.py flash

`idf.py flash` wraps `esptool.py`; these are for when you need something idf.py doesn't
expose directly.

```bash
esptool.py --port $ESPPORT chip_id           # confirm chip identity/revision
esptool.py --port $ESPPORT flash_id          # confirm flash chip info (size, manufacturer)
esptool.py --port $ESPPORT read_flash 0x0 0x1000 partition_dump.bin   # read raw bytes from flash, e.g. partition table region
esptool.py --port $ESPPORT erase_flash       # full chip erase — wipes everything, including NVS data
```

---

## Crash / panic debugging

```bash
idf.py -p $ESPPORT monitor      # panics auto-decode here IF build/ has the matching .elf
```
Look for `Guru Meditation Error: Core X panic'ed (<reason>)` — this is a real fault, not
a clean restart. Compare against a clean restart's reset reason
(`RTC_SW_CPU_RST`/`POWERON_RESET`/etc. with a single-frame trace to something like
`esp_restart_noos`) — see Phase 0 doc §6b for a worked example of telling these apart.

Manual backtrace decode, if the monitor doesn't auto-resolve an address for some reason:
```bash
xtensa-esp32s3-elf-addr2line -pfiaC -e build/OMNIS.elf <address>
```

---

## Git hygiene for this project (minimum)

```bash
# .gitignore should include at minimum:
build/
sdkconfig.old
managed_components/
```
`sdkconfig` itself is generally committed (project-specific config); `sdkconfig.old` is
an auto-generated backup and should not be.

---

## JTAG / OpenOCD (awareness only — not yet set up)

```bash
openocd -f board/esp32s3-builtin.cfg     # example only — exact config depends on debug probe/wiring, not yet exercised on this project
xtensa-esp32s3-elf-gdb build/OMNIS.elf   # then connect to OpenOCD's gdb stub from within
```
Flagged in Phase 0 doc §7 as deferred — listed here for when it becomes relevant.
