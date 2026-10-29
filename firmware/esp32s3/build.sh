#!/usr/bin/env bash
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
# Maintainer step: build the three ESP32-S3 firmware tiers with ESP-IDF and store them as the shipped
# images (firmware/prebuilt-esp32s3/):
#   DECKHAND  any ESP32-S3                      build/
#   CAPTAIN   4 MB flash + PSRAM                build-captain/   (tiers/captain.defaults)
#   SKIPPER   8 MB flash + 8 MB octal PSRAM     build-skipper/   (tiers/skipper.defaults)
# Needs ESP-IDF v5.4.x already installed and exported (. ~/esp/esp-idf/export.sh). Linux/macOS.
set -euo pipefail
cd "$(dirname "$0")"
command -v idf.py >/dev/null || { echo "idf.py not found: install ESP-IDF v5.4 and run its export.sh first"; exit 1; }
export IDF_COMPONENT_MANAGER="${IDF_COMPONENT_MANAGER:-0}"   # no downloads needed: every component is local
for tier in deckhand captain skipper; do
    dir=build; [ "$tier" = deckhand ] || dir="build-$tier"
    echo "=== $tier ($dir)"
    idf.py -B "$dir" -DSF_TIER="$tier" set-target esp32s3
    idf.py -B "$dir" -DSF_TIER="$tier" build
    idf.py -B "$dir" -DSF_TIER="$tier" merge-bin -o skipper-esp32s3-merged.bin
done
IDF_VER="$(idf.py --version | sed -E 's/^ESP-IDF //')"
python3 ../../host/tools/skipper_esp.py bundle-tiers --build-root . --idf "$IDF_VER"
