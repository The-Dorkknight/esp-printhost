#!/bin/sh
# Skipper Flipper Flasher (needs python3 with Tk)
cd "$(dirname "$0")" || exit 1
python3 flasher.py || { echo; echo "The flasher stopped with an error. Press Enter to close."; read -r _; }
