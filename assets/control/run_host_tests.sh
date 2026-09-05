#!/bin/sh
# Host-side verification of the OMNIS control stack. No ESP-IDF, no hardware.
#   ./run_host_tests.sh
set -e
cd "$(dirname "$0")"
cc -std=c99 -Wall -Wextra -Werror -O2 -o /tmp/omnis_test_control \
   test_control.c attitude_ekf.c imu_fusion.c pid.c -lm
exec /tmp/omnis_test_control
