#!/bin/sh
# Host-side verification of the OMNIS kinematics port and drive pipeline.
# No ESP-IDF, no hardware, no board attached.
#
#   ./test/run_host_tests.sh
set -e
cd "$(dirname "$0")/.."
cc -std=c99 -Wall -Wextra -Werror -O2 -I main \
   -o /tmp/omnis_test_kinematics \
   test/test_kinematics.c main/mecanum_kinematics.c main/drive.c main/omnis_params.c -lm
exec /tmp/omnis_test_kinematics
