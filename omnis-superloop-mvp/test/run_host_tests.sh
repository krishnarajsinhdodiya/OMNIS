#!/bin/sh
# Host-side verification of every hardware-independent module in the build.
# No ESP-IDF, no hardware, no board attached.
#
#   ./test/run_host_tests.sh
#
# Each suite is a separate binary. The script runs all of them, then exits
# non-zero if any failed, so a single failure cannot hide behind later passes.
set -u
cd "$(dirname "$0")/.."

OUT="${TMPDIR:-/tmp}/omnis_host_tests"
mkdir -p "$OUT"
CFLAGS="-std=c99 -Wall -Wextra -Werror -O2 -I main"

suites=0
failed=0
failed_names=""

run_suite() {
    name="$1"; shift
    suites=$((suites + 1))
    printf '\n================ %s ================\n' "$name"
    if ! cc $CFLAGS -o "$OUT/$name" "$@" -lm; then
        echo "*** $name: COMPILE FAILED"
        failed=$((failed + 1)); failed_names="$failed_names $name"
        return
    fi
    if ! "$OUT/$name"; then
        failed=$((failed + 1)); failed_names="$failed_names $name"
    fi
}

run_suite test_kinematics test/test_kinematics.c \
    main/mecanum_kinematics.c main/drive.c main/omnis_params.c

run_suite test_mpu6050 test/test_mpu6050.c

if [ -f test/test_control.c ]; then
    run_suite test_control test/test_control.c \
        main/attitude_ekf.c main/imu_fusion.c main/pid.c
fi
if [ -f test/test_crsf.c ]; then
    run_suite test_crsf test/test_crsf.c \
        main/crsf_parser.c main/rc_input.c main/omnis_params.c
fi
if [ -f test/test_step_wave.c ]; then
    run_suite test_step_wave test/test_step_wave.c \
        main/step_wave.c main/drive.c main/mecanum_kinematics.c main/omnis_params.c
fi
if [ -f test/test_supervisor.c ]; then
    run_suite test_supervisor test/test_supervisor.c \
        main/fault.c main/buzzer_pattern.c main/supervisor.c main/imu_fusion.c main/attitude_ekf.c
fi
if [ -f test/test_balance.c ]; then
    run_suite test_balance test/test_balance.c \
        main/balance.c main/pid.c main/imu_fusion.c main/attitude_ekf.c main/omnis_params.c
fi

printf '\n================ SUMMARY ================\n'
if [ "$failed" -eq 0 ]; then
    echo "ALL $suites HOST TEST SUITES PASSED"
    exit 0
fi
echo "$failed of $suites suites FAILED:$failed_names"
exit 1
