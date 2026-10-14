# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
cmake_minimum_required(VERSION 3.16)
set(PROJECT_VER "0.1.0-wip")
# The protocol library in ../../host is built as the "host" component (same code as the PC tests).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../host")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(skipper_flipper_esp32s3)
