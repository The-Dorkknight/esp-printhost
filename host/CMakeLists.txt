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
    src/engine.cpp
    src/multipart.cpp
    src/shaping.cpp
    src/shaping_fx.cpp
    src/printbench.cpp
    src/bench.cpp
    src/resonance.cpp
    src/bedmesh.cpp
    src/machine_pro.cpp
    src/machine_moon.cpp
    src/machine_panel.cpp
    src/panel.cpp)

if(ESP_PLATFORM)
    idf_component_register(SRCS ${PH_SRCS} INCLUDE_DIRS include)
    # The motion hot path (step compression, shaping, planning) built for speed instead of size
    # (2026-10-27) on the builds with room for it; DECKHAND stays -Os.
    if(CONFIG_SKIPPER_PRO)
        set_source_files_properties(src/toolhead.cpp src/shaping.cpp src/shaping_fx.cpp src/planner.cpp
                                    PROPERTIES COMPILE_OPTIONS "-O2")
    endif()
    # The int32/int64 motion test build (2026-10-27, idf.py -DSF_FIXED=1): integer shaping and step times
    if(CONFIG_SKIPPER_FIXED)
        target_compile_definitions(${COMPONENT_LIB} PUBLIC PH_FIXED=1)
    endif()
    # DECKHAND (no CONFIG_SKIPPER_PRO): no shaping, calibration, mesh... (flash space)
    if(NOT CONFIG_SKIPPER_PRO)
        target_compile_definitions(${COMPONENT_LIB} PUBLIC PH_PRO=0)
    endif()
    # Kinematics (2026-10-31): the DECKHAND delta build has delta instead of CoreXY
    if(CONFIG_SKIPPER_KIN_DELTA)
        target_compile_definitions(${COMPONENT_LIB} PUBLIC PH_KIN_DELTA=1 PH_KIN_COREXY=0)
    endif()
    if(CONFIG_SKIPPER_BENCH_AT_BOOT)
        target_compile_definitions(${COMPONENT_LIB} PUBLIC PH_BENCH_LEGACY=1)
    endif()
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

add_library(ph_core STATIC ${PH_SRCS})
target_include_directories(ph_core PUBLIC include)

# test/segq_core.c: test-only step replay (C99), used by test_engine.cpp to check the planner
if(NOT MSVC)
    set_source_files_properties(test/segq_core.c PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-pedantic")
endif()
add_executable(ph_tests test/segq_core.c test/test_main.cpp test/test_segq_core.cpp test/test_engine.cpp test/test_kboard.cpp test/test_machine.cpp test/test_toolhead.cpp test/test_multipart.cpp test/test_shaping.cpp test/test_resonance.cpp)
target_link_libraries(ph_tests ph_core)

add_executable(bench_gcode tools/bench_gcode.cpp)
target_link_libraries(bench_gcode ph_core)

# The same tests on the int32/int64 motion path (2026-10-27): the whole library built again with
# PH_FIXED=1. Both must pass.
add_library(ph_core_fixed STATIC ${PH_SRCS})
target_include_directories(ph_core_fixed PUBLIC include)
target_compile_definitions(ph_core_fixed PUBLIC PH_FIXED=1)
add_executable(ph_tests_fixed test/segq_core.c test/test_main.cpp test/test_segq_core.cpp test/test_engine.cpp test/test_kboard.cpp test/test_machine.cpp test/test_toolhead.cpp test/test_multipart.cpp test/test_shaping.cpp test/test_resonance.cpp)
target_link_libraries(ph_tests_fixed ph_core_fixed)

# The little OLED's drawing code (firmware/esp32s3/components/ktft_oled, from KlipperTFT-Pico): plain
# C++, tested on the PC like the rest; the I2C driver part is ESP-IDF only.
set(OLED_DIR ${CMAKE_CURRENT_LIST_DIR}/../firmware/esp32s3/components/ktft_oled)
add_executable(oled_tests ${OLED_DIR}/oled.cpp ${OLED_DIR}/oled_fonts.cpp ${OLED_DIR}/tests/oled_test.cpp)
target_include_directories(oled_tests PRIVATE ${OLED_DIR}/include)

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
add_test(NAME ph_tests_fixed COMMAND ph_tests_fixed)
add_test(NAME oled_tests COMMAND oled_tests)
