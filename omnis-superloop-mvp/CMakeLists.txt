# OMNIS Superloop MVP — standalone ESP-IDF project.
#
# Deliberately separate from the repo-root OMNIS project so the two do not
# collide. Build from the repo root with:
#
#     idf.py -C omnis-superloop-mvp build
#
# Target: ESP32-S3-WROOM-1-N16R8 on EdgeHax S3 Pro. ESP-IDF v6.0.2.
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Pull in only main and what it depends on. This build has no Wi-Fi, no
# Bluetooth, no FATFS, no OTA — see PLAN.md "explicitly excluded".
idf_build_set_property(MINIMAL_BUILD ON)

project(omnis_superloop_mvp)
