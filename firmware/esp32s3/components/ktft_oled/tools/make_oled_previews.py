# Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
# SPDX-License-Identifier: GPL-3.0-only. From KlipperTFT-Pico 0.6.1 (same author).
"""Builds the OLED layout preview (pc-preview/oled_preview.cpp) with the PC's C++ compiler and
writes the screens as PNG pictures into docs/images/.

    python3 firmware/esp32s3/components/ktft_oled/tools/make_oled_previews.py
"""
import glob
import os
import subprocess
import sys
import tempfile

from PIL import Image

COMP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # firmware/esp32s3/components/ktft_oled
REPO = os.path.normpath(os.path.join(COMP, "..", "..", "..", ".."))
OUT = os.path.join(REPO, "docs", "images", "oled")
SCALE = 3


def main():
    tmp = tempfile.mkdtemp()
    exe = os.path.join(tmp, "oled_preview")
    srcs = [os.path.join(COMP, f) for f in ("oled.cpp", "oled_fonts.cpp")]
    subprocess.check_call(["g++", "-std=c++17", "-O1", "-I", os.path.join(COMP, "include")] + srcs +
                          [os.path.join(COMP, "pc-preview", "oled_preview.cpp"), "-o", exe])
    subprocess.check_call([exe, tmp])
    os.makedirs(OUT, exist_ok=True)
    for pbm in sorted(glob.glob(os.path.join(tmp, "*.pbm"))):
        src = Image.open(pbm).convert("1")
        img = Image.new("RGB", src.size, (0, 0, 0))
        px, out = src.load(), img.load()
        for y in range(src.height):
            for x in range(src.width):
                if px[x, y] == 0:            # PBM "1" (set pixel) loads as black
                    out[x, y] = (120, 220, 255)
        img = img.resize((src.width * SCALE, src.height * SCALE), Image.NEAREST)
        name = os.path.splitext(os.path.basename(pbm))[0] + ".png"
        img.save(os.path.join(OUT, name))
        print("wrote docs/images/oled/" + name)


if __name__ == "__main__":
    sys.exit(main())
