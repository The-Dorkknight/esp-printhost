# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
cmake_minimum_required(VERSION 3.16)
# Three builds from one source, by board memory (build.sh builds all three):
#   idf.py -B build build                                  DECKHAND  (any ESP32-S3, 2 MB flash)
#   idf.py -B build-captain -DSF_TIER=captain build        CAPTAIN   (4 MB flash, PSRAM if present)
#   idf.py -B build-skipper -DSF_TIER=skipper build        SKIPPER   (8 MB flash, 8 MB octal PSRAM)
if(NOT DEFINED SF_TIER)
    set(SF_TIER deckhand)
endif()
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
if(NOT SF_TIER STREQUAL "deckhand")
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/tiers/${SF_TIER}.defaults")
endif()
if(SF_FIXED)   # the int32/int64 motion test build (2026-10-27): idf.py -B build-captain-int -DSF_TIER=captain -DSF_FIXED=1 build
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/tiers/fixed.defaults")
endif()
if(DEFINED SF_EXTRA_DEFAULTS)   # e.g. a QEMU test build with the console on UART0
    list(APPEND SDKCONFIG_DEFAULTS "${SF_EXTRA_DEFAULTS}")
endif()
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")   # one per build folder, so the tiers never mix
set(PROJECT_VER "0.1.0-wip-${SF_TIER}")          # checked by the WiFi update (no cross-tier images)
# The protocol library in ../../host is built as the "host" component (same code as the PC tests).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../host")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(skipper_flipper_esp32s3)
