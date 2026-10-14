# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
# Dual-mode: ESP-IDF component when IDF is building, plain library + tests on PC.
set(PH_SRCS
    src/wire.cpp
    src/klink.cpp
    src/kboard.cpp
    src/board_session.cpp
    src/clocksync.cpp
    src/boardcfg.cpp
    src/thermistor.cpp
    src/machine.cpp
    src/toolhead.cpp
    src/json_mini.cpp
    src/msgproto.cpp
    src/config_ini.cpp
    src/gcode.cpp
    src/motion.cpp
    src/planner.cpp
    src/engine.cpp)
# Shared C core (also built into the MCU firmware): see ../firmware/skipper/
set(PH_SEGQ_DIR ${CMAKE_CURRENT_LIST_DIR}/../firmware/skipper)
set(PH_C_SRCS ${PH_SEGQ_DIR}/segq_core.c)

if(ESP_PLATFORM)
    idf_component_register(SRCS ${PH_SRCS} ${PH_C_SRCS} INCLUDE_DIRS include ${PH_SEGQ_DIR})
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(printhost_core C CXX)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT MSVC)
    add_compile_options(-Wall -Wextra -Werror -O2)
endif()

add_library(ph_core STATIC ${PH_SRCS} ${PH_C_SRCS})
target_include_directories(ph_core PUBLIC include ${PH_SEGQ_DIR})
if(NOT MSVC)
    set_source_files_properties(${PH_C_SRCS} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-pedantic")
endif()

add_executable(ph_tests test/test_main.cpp test/test_segq_core.cpp test/test_engine.cpp test/test_kboard.cpp test/test_machine.cpp test/test_toolhead.cpp)
target_link_libraries(ph_tests ph_core)

find_package(ZLIB)
if(ZLIB_FOUND AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(mcu_probe tools/mcu_probe.cpp)
    target_link_libraries(mcu_probe ph_core ZLIB::ZLIB)
    target_compile_options(mcu_probe PRIVATE -Wno-missing-field-initializers)
    add_executable(klink_probe tools/klink_probe.cpp)
    target_link_libraries(klink_probe ph_core ZLIB::ZLIB)
    target_compile_options(klink_probe PRIVATE -Wno-missing-field-initializers)
endif()

enable_testing()
add_test(NAME ph_tests COMMAND ph_tests)
