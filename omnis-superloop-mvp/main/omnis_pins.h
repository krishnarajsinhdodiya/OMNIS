/**
 * @file    omnis_pins.h
 * @brief   GPIO map for the OMNIS superloop MVP — board Rev 2.0.
 *
 * SOURCE OF TRUTH: assets/pcb/OMNIS.kicad_sch (Rev 2.0, supplied 2026-09-15).
 * Every assignment below was traced pin-by-pin through the schematic's wire
 * segments to its global label, not read off a picture. One place, named
 * constants, no bare GPIO numbers anywhere else in this build.
 *
 * Rev 2.0 changes firmware BEHAVIOUR, not just numbers:
 *
 *   1. COM_ENA IS GONE. All four A4988 EN# pins are hardwired to GND
 *      (assets/pcb/rev2-pin-assignment.md §3). The drivers cannot be disabled
 *      in software: whenever 12 V is present, all four motors are energised and
 *      holding. The firmware failsafe is therefore "stop generating STEP
 *      pulses" — the motors hold position, they do not go limp.
 *   2. Buzzer moved 14 -> 16 (GPIO14 belongs to the onboard microSD).
 *
 * The planning doc assets/pcb/rev2-pin-assignment.md disagrees with the final
 * schematic on seven nets. The schematic wins; the doc is older:
 *
 *     net          schematic   planning doc
 *     FR_STEP       1          38
 *     FR_DIR       38          40
 *     BL_DIR       46          41
 *     B_SF         39          48
 *     B_LEFT       41          46
 *     B_RIGHT      47          39
 *     B_SELECT     48          47
 */

#ifndef OMNIS_PINS_H
#define OMNIS_PINS_H

#include "driver/gpio.h"

/* --- Stepper enable: does not exist on Rev 2.0 ---------------------------
 * EN# is tied to GND. Kept as a named constant set to GPIO_NUM_NC so that code
 * which wants to disable the drivers can test for it, and so that restoring the
 * line later (the plan's hedge: a cuttable EN# link rewired to GPIO0/3/45) is a
 * one-line change here rather than a hunt through the codebase.
 * ------------------------------------------------------------------------ */
#define PIN_STEPPER_EN          GPIO_NUM_NC

/* --- A4988 STEP / DIR, one pair per wheel --------------------------------
 * Firmware corner names are the kinematics names FL / FR / RL / RR. The
 * schematic calls the rear wheels "back": BL = RL, BR = RR. Front is the OLED /
 * button end (assets/kinematics/mecanum-kinematics-reference.md §4).
 * ------------------------------------------------------------------------ */
#define PIN_FL_STEP             GPIO_NUM_4    /* FL_STEP */
#define PIN_FL_DIR              GPIO_NUM_5    /* FL_DIR  */
#define PIN_FR_STEP             GPIO_NUM_1   /* FR_STEP */
#define PIN_FR_DIR              GPIO_NUM_38   /* FR_DIR  */
#define PIN_RL_STEP             GPIO_NUM_15   /* BL_STEP */
#define PIN_RL_DIR              GPIO_NUM_46   /* BL_DIR, STRAPPING PIN */
#define PIN_RR_STEP             GPIO_NUM_21   /* BR_STEP */
#define PIN_RR_DIR              GPIO_NUM_2   /* BR_DIR */

/* Pin hazards worth knowing before a wheel misbehaves:
 *
 * GPIO1 carries FR_STEP and GPIO2 carries BR_DIR on the revised PCB. Older
 *   documentation listed these motor signals on GPIO40 and GPIO42.
 *
 * GPIO46 (RL_DIR) is a strapping pin sampled at reset. The A4988 DIR input has
 *   no pull of its own, so the chip's internal pull-down wins and the board
 *   boots normally. Nothing may drive this line high during reset.
 *
 * GPIO38-42 behave as plain GPIO only because the JTAG-source strap, GPIO3, is
 *   left unconnected. Do not wire anything to GPIO3. */

/* --- I2C: both MPU6050s (and the OLED in the full build) ----------------- */
#define PIN_I2C_SDA             GPIO_NUM_8    /* I2C_SDA */
#define PIN_I2C_SCL             GPIO_NUM_9    /* I2C_SCL */

#define I2C_ADDR_IMU_A          0x68   /**< MPU1: AD0 low  */
#define I2C_ADDR_IMU_B          0x69   /**< MPU2: AD0 high */

/* --- CRSF / ExpressLRS UART ----------------------------------------------
 * Rev 2.0 names these nets from the ESP32's point of view (Rev 1.0 used the
 * receiver's, which is why the old header carried a paragraph about crossing
 * them):
 *
 *   net "TX" -> GPIO17 -> receiver Rx   (ESP32 would transmit here)
 *   net "RX" -> GPIO18 -> receiver Tx   (ESP32 receives CRSF here)
 *
 * The MVP never transmits to the receiver, so the UART is opened RX-only on
 * GPIO18. If no valid CRSF frame arrives there, crsf.c also tries GPIO17 as the
 * receive pin, which rescues a crossed harness. Both pins stay inputs the whole
 * time, so a crossed wire can never put two outputs against each other.
 * ------------------------------------------------------------------------ */
#define PIN_CRSF_ESP_TX         GPIO_NUM_17   /* net "TX" */
#define PIN_CRSF_ESP_RX         GPIO_NUM_18   /* net "RX" */

/* --- Buzzer --------------------------------------------------------------
 * A 3-pin module with its own driver IC, not a bare piezo. The IO pin is a
 * logic-level enable: HIGH = sound. No pitch control — patterns are built from
 * on/off timing only.
 * ------------------------------------------------------------------------ */
#define PIN_BUZZER              GPIO_NUM_16   /* BUZZER */
#define BUZZER_ON               1
#define BUZZER_OFF              0

/* ==========================================================================
 * RESERVED — wired on the board, NOT touched by this build.
 *
 * Listed so nothing here claims one by accident. Each belongs to an excluded
 * feature (BUILD-LOG.md, delivered-against-scope): deferred, not abandoned.
 *
 *   GPIO 6        BATT_SENSE  33k/10k divider, ADC1_CH5
 *   GPIO 39       B_SF        button (MTCK)
 *   GPIO 41       B_LEFT      button (MTDI; EdgeHax white LED pin)
 *   GPIO 47       B_RIGHT     button
 *   GPIO 48       B_SEL       button
 *
 *   Buttons must be configured as FLOATING inputs when they return — the
 *   external pull-downs do the work, and an internal pull-up would fight them.
 *
 * DO NOT USE — deliberately unconnected in Rev 2.0:
 *
 *   GPIO 0        BOOT strap
 *   GPIO 3        JTAG-source strap (keeps GPIO39-42 as GPIO)
 *   GPIO 10-14    onboard microSD
 *   GPIO 19, 20   native USB D-/D+ (USB-Serial-JTAG)
 *   GPIO 35-37    octal PSRAM on N16R8 (the Stage 1 heartbeat on 35 was a bug)
 *   GPIO 43, 44   UART0 console
 *   GPIO 45       VDD_SPI strap — high at boot selects 1.8 V flash
 * ========================================================================== */

#endif /* OMNIS_PINS_H */
