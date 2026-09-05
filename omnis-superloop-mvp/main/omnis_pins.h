/**
 * @file    omnis_pins.h
 * @brief   GPIO map for the OMNIS superloop MVP.
 *
 * Transcribed from omnis-info.md §3a, verified against schematic rev 1.0
 * (as revised 2026-08-16). One place, named constants, no bare GPIO numbers
 * anywhere else in this build.
 *
 * Pins belonging to EXCLUDED features are listed at the bottom as reserved, so
 * nothing in this build quietly claims one and creates a conflict when the
 * OLED / SD / buttons come back.
 */

#ifndef OMNIS_PINS_H
#define OMNIS_PINS_H

#include "driver/gpio.h"

/* --- Stepper enable ------------------------------------------------------
 * ONE line, shared by all four A4988s. Active LOW (EN# on the driver), so
 * driving it HIGH disables every motor.
 *
 * This is the failsafe actuator for the whole machine (§7f, §7g point 5). It is
 * the first thing configured in main(), before anything else can spin a wheel.
 * ------------------------------------------------------------------------ */
#define PIN_COM_ENA         GPIO_NUM_1

#define COM_ENA_DISABLED    1   /**< EN# high  -> drivers off, motors free */
#define COM_ENA_ENABLED     0   /**< EN# low   -> drivers on               */

/* --- A4988 STEP / DIR, one pair per wheel --------------------------------
 * Corner labelling is as-built and confirmed: front is the OLED/button end.
 * See assets/kinematics/mecanum-kinematics-reference.md §4.
 * ------------------------------------------------------------------------ */
#define PIN_FL_STEP         GPIO_NUM_4
#define PIN_FL_DIR          GPIO_NUM_5
#define PIN_FR_STEP         GPIO_NUM_41   /* MTDI */
#define PIN_FR_DIR          GPIO_NUM_40   /* MTDO */
#define PIN_RL_STEP         GPIO_NUM_15
#define PIN_RL_DIR          GPIO_NUM_16
#define PIN_RR_STEP         GPIO_NUM_21
#define PIN_RR_DIR          GPIO_NUM_42   /* MTMS */

/* GPIO39-42 are the chip's default JTAG pins (MTCK/MTDO/MTDI/MTMS). They behave
 * as ordinary GPIO because the JTAG source strap is GPIO3 (B_SF), which idles
 * low (§3e). The one failure mode is holding the Special-Function button down
 * through a power cycle — which would put four load-bearing driver pins into
 * JTAG mode. Do not do that. */

/* --- I2C: both MPU6050s (and, in the full build, the OLED) --------------- */
#define PIN_I2C_SDA         GPIO_NUM_8
#define PIN_I2C_SCL         GPIO_NUM_9

#define I2C_ADDR_IMU_A      0x68   /**< front-left corner, faces forward  */
#define I2C_ADDR_IMU_B      0x69   /**< rear-right corner, faces rearward */

/* --- CRSF / ExpressLRS UART ----------------------------------------------
 * THE NET NAMES ARE FROM THE RECEIVER'S PERSPECTIVE AND ARE THEREFORE
 * REVERSED FROM THE ESP32'S ROLE (omnis-info.md §3c).
 *
 *   Schematic net "RX" -> receiver's Rx input  -> ESP32 must TRANSMIT -> GPIO17
 *   Schematic net "TX" -> receiver's Tx output -> ESP32 must RECEIVE  -> GPIO18
 *
 * Getting this backwards is the classic first-day CRSF bug: no frames arrive
 * and everything looks correctly wired.
 * ------------------------------------------------------------------------ */
#define PIN_CRSF_TX         GPIO_NUM_17   /* ESP32 TX -> receiver Rx */
#define PIN_CRSF_RX         GPIO_NUM_18   /* ESP32 RX <- receiver Tx */

/* --- Buzzer --------------------------------------------------------------
 * A 3-pin module with its own driver IC, not a bare piezo (§2). The IO pin is a
 * logic-level enable: drive HIGH for sound. It is NOT a PWM tone input, so
 * "pitch" is not available — patterns must be made from on/off timing.
 * ------------------------------------------------------------------------ */
#define PIN_BUZZER          GPIO_NUM_14
#define BUZZER_ON           1
#define BUZZER_OFF          0

/* --- Stage 1 bring-up only ----------------------------------------------
 * A spare pin toggled once per tick so the 500 Hz spine can be verified on a
 * scope or logic analyser. GPIO35 is spare per §3b (freed by the PSRAM-conflict
 * move to 39-42). Toggling every tick gives a 250 Hz square wave.
 *
 * Remove this when the tick is trusted; it is scaffolding, not a feature.
 * ------------------------------------------------------------------------ */
#define PIN_TICK_HEARTBEAT  GPIO_NUM_35

/* ==========================================================================
 * RESERVED — wired on the board, NOT used by this build.
 *
 * Listed so nothing here claims one by accident. Each is deferred, not
 * abandoned; see PLAN.md "explicitly excluded".
 *
 *   GPIO 6            BATT_SENSE   battery divider, ADC1_5      (§3g)
 *   GPIO 10,11,12,13  microSD      CS / MOSI / CLK / MISO       (§9b)
 *   GPIO 2            B_RIGHT      button, active-HIGH          (§3d)
 *   GPIO 3            B_SF         button, active-HIGH, STRAP   (§3e)
 *   GPIO 39           B_DOWN       button, active-HIGH
 *   GPIO 45           B_SELECT     button, active-HIGH, STRAP   (§3e)
 *   GPIO 47           B_LEFT       button, active-HIGH
 *   GPIO 48           B_UP         button, active-HIGH
 *
 * NOTE GPIO39 is listed twice in the schematic's story: it is B_DOWN in §3a.
 * This build does not use the buttons, so there is no conflict today, but the
 * full build must resolve B_DOWN against nothing else — it is genuinely just
 * the Down button.
 *
 * Truly spare: GPIO 0, 7, 35(used above for heartbeat), 36, 37, 38, 43, 44, 46.
 * ========================================================================== */

#endif /* OMNIS_PINS_H */
