#!/usr/bin/env bash
# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
# Maintainer step: build the ESP32-S3 firmware with ESP-IDF and store it as the shipped image.
# Needs ESP-IDF v5.4.x already installed and exported (. ~/esp/esp-idf/export.sh). Linux/macOS.
set -euo pipefail
cd "$(dirname "$0")"
command -v idf.py >/dev/null || { echo "idf.py not found: install ESP-IDF v5.4 and run its export.sh first"; exit 1; }
idf.py set-target esp32s3
idf.py build
idf.py merge-bin -o skipper-esp32s3-merged.bin
IDF_VER="$(idf.py --version | sed -E 's/^ESP-IDF //')"
python3 ../../host/tools/skipper_esp.py bundle-image build/skipper-esp32s3-merged.bin --idf "$IDF_VER"
