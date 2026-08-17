# VS Code Setup Guide — OMNIS (ESP32-S3 + ESP-IDF + FreeRTOS)

This assumes VS Code is already installed on your machine — no VS Code install step here, this starts directly from extensions.

## 1. Prerequisites (OS-level)

Before touching VS Code, make sure your system has:

- **Git** installed and on PATH (`git --version` to check)
- **Python 3.9+** installed and on PATH (`python3 --version`) — ESP-IDF's tooling depends on this
- On Windows: enable long path support if you haven't (ESP-IDF's toolchain has deep nested paths)
- USB driver for your EdgeHax S3 Pro board's USB-C-to-serial chip (usually CP210x or CH340 — check the board's docs; Windows sometimes needs the driver installed manually, Linux/macOS usually don't)

## 2. Install the Core Extensions

Open the Extensions panel (`Ctrl+Shift+X` / `Cmd+Shift+X`) and install:

1. **Espressif IDF** (publisher: Espressif Systems) — this is the big one. It bundles ESP-IDF (the framework), the Xtensa/RISC-V toolchains, and **FreeRTOS comes with it automatically** since ESP-IDF is built on top of FreeRTOS.
2. **C/C++** (publisher: Microsoft) — IntelliSense, debugging, go-to-definition for your C code. The IDF extension will prompt to install this if missing.
3. **C/C++ Extension Pack** (optional but convenient — bundles CMake Tools, which ESP-IDF's build system uses under the hood, useful if you want to inspect the CMake side directly)
4. **GitLens** (optional) — richer git history/blame view, useful once the repo has some commits
5. **Better C++ Syntax** (optional, cosmetic — improves highlighting for modern C/C++ features)

## 3. Run the ESP-IDF Setup Wizard

1. `Ctrl+Shift+P` → type "ESP-IDF: Configure ESP-IDF Extension" → Enter
2. Choose **Express** setup for the first install (Advanced lets you point to an existing IDF install, not needed on a fresh machine)
3. Pick the ESP-IDF **version** — use the latest stable release (avoid `master`/dev branches for a portfolio project; you want stability, not bleeding-edge features)
4. Let it download: this pulls the IDF source, the Xtensa toolchain (needed for ESP32-S3, which is Xtensa LX7-based), and a dedicated Python virtual environment. This step is large (~1–2GB) and can take a while depending on your connection
5. Once it finishes, the status bar at the bottom of VS Code should show the ESP-IDF version and a few new icons (build, flash, monitor, etc.)

## 4. Create/Open the OMNIS Project

- New project: `Ctrl+Shift+P` → "ESP-IDF: New Project" → pick a template (start from `template-app` or `hello_world` and strip it down) or, once you have your GitHub repo, just `git clone` it and open the folder in VS Code
- Set the target chip: `Ctrl+Shift+P` → "ESP-IDF: Set Espressif Device Target" → select **esp32s3**
- Select the serial port for your EdgeHax S3 Pro board via the bottom status bar port selector (or `Ctrl+Shift+P` → "ESP-IDF: Select Port to Use")

## 5. Build / Flash / Monitor Workflow

These are the commands you'll use constantly, available both as bottom-status-bar icons and command-palette entries:

| Action | Command Palette | Status Bar Icon |
|---|---|---|
| Build | ESP-IDF: Build your Project | 🔧 |
| Flash | ESP-IDF: Flash your Project | ⚡ |
| Monitor (serial console) | ESP-IDF: Monitor your Device | 🔌 |
| Build+Flash+Monitor combined | ESP-IDF: Build, Flash and Start a Monitor on your Device | ▶ (play icon) |

`Ctrl+]` inside the monitor exits it back to the terminal.

## 6. Quick Sanity Check

Before changing anything else, build and flash a stock `hello_world` example first. This confirms the toolchain, board, and serial port are all correctly wired up before you start layering your own kernel or logic on top of it.

## 7. Project Configuration (sdkconfig)

`Ctrl+Shift+P` → "ESP-IDF: SDK Configuration Editor (menuconfig)" opens a searchable GUI over `sdkconfig`. Relevant early settings for OMNIS:

- **Component config → ESP PSRAM** → enable, since the N16R8 variant has 8MB PSRAM you'll want available
- **Component config → FreeRTOS** → FreeRTOS-specific tuning (tick rate, task stack sizes, etc.)
- **Partition Table** → revisit once you decide how much flash to reserve for logging/data vs. firmware, given the 16MB flash

## 8. Git / GitHub Setup

VS Code has git built in (no extension needed for basics):

1. `Ctrl+Shift+P` → "Git: Clone" if starting from the GitHub repo, or "Git: Initialize Repository" if starting local-first and pushing up later
2. Source Control panel (`Ctrl+Shift+G`) handles stage/commit/push visually
3. If you haven't set global git identity yet: `git config --global user.name "..."` and `git config --global user.email "..."` in the integrated terminal
4. Add a `.gitignore` — for ESP-IDF projects, at minimum ignore `build/`, `sdkconfig.old`, and `managed_components/` if you end up using the component manager

## 9. Learning FreeRTOS Alongside This Project

A practical path once the base project builds:

1. Start with basic `xTaskCreate` calls to get a feel for tasks vs. a bare-metal loop
2. Move to queues (`xQueueSend`/`xQueueReceive`) — maps directly onto the uORB-style pub/sub messaging planned for OMNIS
3. Then semaphores/mutexes for shared resource protection (e.g., the shared I2C bus between both MPU6050s and the LCD)
4. Espressif's own FreeRTOS SMP docs are worth reading since the S3 is dual-core — task-to-core pinning (`xTaskCreatePinnedToCore`) will matter for keeping the control loop deterministic

## 10. Vendoring Your Own FreeRTOS Kernel (Optional, Advanced)

For deeper learning, the FreeRTOS-LTS kernel downloaded directly from the FreeRTOS website can be swapped in as a custom ESP-IDF component instead of relying solely on the version bundled inside ESP-IDF. This step is optional — the project builds fine without it — but it's the way to actually study and run the exact kernel source rather than the version buried inside the SDK install.

1. **Create the component folder** — inside the OMNIS project, next to `main/`, create `components/freertos/`. ESP-IDF auto-discovers anything in `components/`, and a component named exactly `freertos` there takes priority over the one bundled inside the SDK.
2. **Copy in the kernel source** — from the downloaded zip's `FreeRTOS-Kernel/` folder, copy `tasks.c`, `queue.c`, `list.c`, `timers.c`, `event_groups.c`, `stream_buffer.c`, and the `include/` headers into the new component. Also copy `portable/ThirdParty/GCC/Xtensa_ESP32/` (the CPU port) and one file from `portable/MemMang/` — `heap_4.c` is the standard choice.
3. **Write the component's CMakeLists.txt** — register the sources with `idf_component_register(SRCS ... INCLUDE_DIRS ...)`, and list `REQUIRES hal soc esp_hw_support esp_timer esp_rom esp_system` in it. The port files' `#include "hal/..."` and `"soc/..."` lines still resolve against the installed ESP-IDF — this swaps the kernel, not the whole HAL.
4. **Clean rebuild and read every error** — the likely friction point is version drift, since this LTS kernel snapshot may expect slightly different hal/soc function signatures than the installed ESP-IDF version. Working through those mismatches by hand is where the real learning happens.
5. **Prove the swap works before building on it** — flash a minimal test (two tasks toggling an LED or printing to serial on independent timers) to confirm real context-switching before writing any OMNIS-specific logic on top.
