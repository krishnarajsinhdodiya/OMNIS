/**
 * @file    omnis_imu_mounting.h
 * @brief   As-built IMU mounting and corner labelling for the OMNIS chassis.
 *
 * This is the one file in assets/control/ that is NOT generic: everything else
 * is reusable filter maths, this records the physical facts of one board.
 *
 * Confirmed by the builder against the assembled board, 2026-09-05.
 *
 *
 *   CHASSIS, VIEWED FROM ABOVE, COMPONENT SIDE UP
 *   (matches the reference photo: front = the OLED / button end)
 *
 *                        +X  FRONT
 *                          ^
 *                          |
 *      ┌───────────────────┼───────────────────┐
 *      │  [FL motor]       |       [FR motor]  │
 *      │   IMU_A 0x68      |                   │
 *      │   arrow -> FRONT  |                   │
 *      │  [A4988]   OLED + buttons   [A4988]   │
 *      │                   |                   │
 * +Y   │- - - - - - - - - -+- - - - - - - - - -│   -Y
 * LEFT │                   |                   │  RIGHT
 *      │  [A4988]      ESP32-S3      [A4988]   │
 *      │                   |    IMU_B 0x69     │
 *      │  [RL motor]  buzzer|   arrow -> REAR   │
 *      │                   |       [RR motor]  │
 *      └───────────────────┼───────────────────┘
 *                          |
 *                         REAR
 *
 *   +Z is out of the page (up). Right-handed, ROS REP-103, identical to the
 *   frame used by mecanum-kinematics-reference.md §1.
 *
 *
 * TWO FACTS THIS FILE EXISTS TO RECORD
 *
 * 1. The two MPU6050 modules are mounted ANTIPARALLEL — 180 degrees apart about
 *    Z. IMU_A at the front-left corner points forward; IMU_B at the rear-right
 *    corner points rearward. Uncorrected, two perfectly healthy sensors read
 *    ~20 degrees apart and trip the 15-degree disagreement fault at every boot.
 *
 * 2. Corner labelling follows from the front edge: FL = front-left as drawn,
 *    and the mecanum sign convention in mecanum-kinematics-reference.md §4
 *    (the LEFT PAIR, FL and RL, share roller handedness delta = -1) is stated
 *    in these labels.
 *
 * The two IMUs sit on the FL/RR diagonal. Under the old X-drive roller layout
 * that diagonal was also a roller-handedness pair; under the as-built lateral
 * parallel layout it is not — the pairs are the left and right SIDES. Nothing
 * in the code ever depended on the coincidence, and this note is kept only
 * because the earlier one claimed it.
 *
 * Reference: assets/control/sensor-fusion-reference.md §2
 */

#ifndef OMNIS_IMU_MOUNTING_H
#define OMNIS_IMU_MOUNTING_H

#include "imu_fusion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- I2C addresses (omnis-info.md §2, AD0 collision resolved) ------------ */
#define OMNIS_IMU_A_I2C_ADDR   0x68   /**< front-left corner  */
#define OMNIS_IMU_B_I2C_ADDR   0x69   /**< rear-right corner  */

/* --- As-built mounting ---------------------------------------------------
 * IMU_A points along chassis +X, so its sensor axes already are the body axes.
 * IMU_B is turned to face the rear: sensor +X is chassis -X, sensor +Y is
 * chassis -Y, and +Z is unchanged because the module is still component-side
 * up. That is exactly {-1, -2, +3}.
 * ------------------------------------------------------------------------ */
#define OMNIS_IMU_A_MOUNT   IMU_MOUNT_IDENTITY
#define OMNIS_IMU_B_MOUNT   IMU_MOUNT_ROT_Z_180

/* These two read the marked arrow on each module as its sensor +X axis. That
 * reading is an interpretation, and a wrong GLOBAL orientation is invisible to
 * every check downstream — the two IMUs still agree with each other, so the
 * disagreement fault stays silent while the balance controller drives the wrong
 * body axis.
 *
 * Do not trust these constants on faith. imu_mount_resolve() derives the
 * descriptor from two poses (level, then nose-down) with no interpretation at
 * all; run it once per IMU on the bench and replace these if it disagrees. It
 * is verified against all 24 right-handed mountings. See
 * sensor-fusion-reference.md §2.5. */

/* --- Balance mode --------------------------------------------------------
 * Front is a short edge, so tipping up onto two wheels rotates the chassis
 * about the body Y (left-right) axis, by about 90 degrees. The grounded pair is
 * the front pair or the rear pair, never a side pair — which is what
 * omnis-info.md §1's "balance on either side" means for this geometry.
 *
 * In the FLAT frame, neither Euler angle is usable at that attitude: roll is
 * gimbal-locked, and pitch FOLDS (see the balance frames below). Balance mode
 * therefore estimates attitude in its own frame.
 * ------------------------------------------------------------------------ */
typedef enum {
    OMNIS_BALANCE_ON_FRONT_PAIR = 0,   /**< FL + FR grounded, pitch ~ +90 deg */
    OMNIS_BALANCE_ON_REAR_PAIR,        /**< RL + RR grounded, pitch ~ -90 deg */
} omnis_balance_pair_t;

/**
 * @brief Which wheel pair is grounded, from the fused pitch estimate.
 *
 * SIGN, because this looks backwards and someone will "fix" it: with the atan2
 * measurement model, pitch = atan2(-ax, hypot(ay, az)), so
 *
 *      POSITIVE pitch  ==  nose DOWN
 *
 * which is the OPPOSITE of the aerospace convention. Verified: a 30 deg
 * nose-down attitude reads accel (-0.500, 0, +0.866) and pitch +30.0 deg; fully
 * tipped forward onto the front wheels reads +90 deg. Hence pitch >= 0 maps to
 * the FRONT pair being grounded. Asserted in test_control.c Case 12.
 *
 * Pass the FLAT-frame pitch. Near +-90 deg its magnitude folds (85 and 95 both
 * read 85) but its SIGN does not, so it still answers "which pair is down" —
 * the one question needed at the moment of arming, before switching frames. It
 * must not be used as the lean angle. Asserted in test_control.c Case 13.
 *
 * Deliberately trivial and deliberately NOT called every tick — omnis-info.md
 * §11c is explicit that a live side-detector fights the balance controller
 * mid-balance. Call it at mode entry or after a detected flip.
 */
static inline omnis_balance_pair_t omnis_balance_pair_from_pitch(float pitch_rad)
{
    return (pitch_rad >= 0.0f) ? OMNIS_BALANCE_ON_FRONT_PAIR
                               : OMNIS_BALANCE_ON_REAR_PAIR;
}

/* --- Balance-mode estimation frames -------------------------------------
 * THE FLAT-FRAME PITCH CANNOT BE THE BALANCE LEAN ANGLE.
 *
 * pitch = atan2(-ax, hypot(ay, az)) is confined to [-90, +90] deg because the
 * hypot is never negative. Balance mode sits at +-90 deg, the edge of that
 * range, where the formula FOLDS: nose-down 85 deg and nose-down 95 deg both
 * read 85 deg. The one thing a balancer must know — which way it is falling — is
 * destroyed at the operating point. (Verified in test_control.c Case 13. An
 * earlier revision of attitude-ekf-derivation.md §7 called this "survivable";
 * it is not.)
 *
 * The fix changes frames, not formulas. In balance mode the body-frame accel and
 * gyro are rotated a further 90 deg about Y before they reach the EKF, so the
 * balancing pose looks LEVEL to the filter: lean sits near 0, where pitch is
 * continuous and signed, and roll is no longer gimbal-locked either.
 *
 * Both frames are defined so that, while balancing:
 *   +X'  points toward the chassis TOP face (the OLED side) — "balance forward"
 *   +Z'  points up
 *   +Y'  completes a right-handed frame
 *
 * so in EITHER pose a positive balance-frame pitch means "falling forward"
 * (toward the top face), and gyro_y' is its rate. One sign convention serves
 * both pairs.
 *
 *   front pair down (nose down 90):  X' = +Z, Y' = +Y, Z' = -X  ->  {+3, +2, -1}
 *   rear  pair down (nose up   90):  X' = +Z, Y' = -Y, Z' = +X  ->  {+3, -2, +1}
 *
 * Y' flips in the rear pose: with X' on the top face and Z' up, the robot's LEFT
 * (+Y') is the chassis RIGHT side. When balancing on the rear pair, the "left"
 * wheel is RR and the "right" wheel is RL.
 *
 * Apply AFTER the sensor mount (OMNIS_IMU_A_MOUNT / _B_MOUNT) and after gyro-bias
 * subtraction, then re-seed the EKFs from the accelerometer: their old states
 * are meaningless in the new frame.
 * ------------------------------------------------------------------------ */
#define OMNIS_IMU_BALANCE_FRAME_FRONT_DOWN  ((imu_mount_t){ { +3, +2, -1 } })
#define OMNIS_IMU_BALANCE_FRAME_REAR_DOWN   ((imu_mount_t){ { +3, -2, +1 } })

/** The estimation frame to use while balancing on the given pair. */
static inline imu_mount_t omnis_balance_frame_for_pair(omnis_balance_pair_t pair)
{
    return (pair == OMNIS_BALANCE_ON_FRONT_PAIR) ? OMNIS_IMU_BALANCE_FRAME_FRONT_DOWN
                                                 : OMNIS_IMU_BALANCE_FRAME_REAR_DOWN;
}

/**
 * @brief Boot-time assertion helper. Returns false if either mount descriptor
 *        is malformed or mirrored.
 *
 * All four constants above are known-good, so this only fires after someone
 * edits them — which is precisely when a silent axis error would be hardest to
 * find. Cheap enough to run unconditionally at startup.
 */
static inline bool omnis_imu_mounting_selfcheck(void)
{
    const imu_mount_t a  = OMNIS_IMU_A_MOUNT;
    const imu_mount_t b  = OMNIS_IMU_B_MOUNT;
    const imu_mount_t bf = OMNIS_IMU_BALANCE_FRAME_FRONT_DOWN;
    const imu_mount_t br = OMNIS_IMU_BALANCE_FRAME_REAR_DOWN;

    return imu_mount_is_valid(&a)  && imu_mount_is_right_handed(&a)
        && imu_mount_is_valid(&b)  && imu_mount_is_right_handed(&b)
        && imu_mount_is_valid(&bf) && imu_mount_is_right_handed(&bf)
        && imu_mount_is_valid(&br) && imu_mount_is_right_handed(&br);
}

#ifdef __cplusplus
}
#endif

#endif /* OMNIS_IMU_MOUNTING_H */
