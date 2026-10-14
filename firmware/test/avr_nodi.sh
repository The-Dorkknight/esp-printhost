#!/bin/sh
# The AVR core must not need 64-bit helper calls: segq_core_push runs with
# interrupts off and load_next runs in the DDA ISR, and libgcc's 64-bit routines
# cost hundreds of cycles each on an 8-bit CPU.
cd "$(dirname "$0")"
command -v avr-gcc >/dev/null 2>&1 || { echo "avr_nodi: avr-gcc not installed, skipped"; exit 0; }
calls=$(avr-gcc -mmcu=atmega2560 -Os -std=gnu99 -S -o - ../skipper/segq_core.c | grep -E 'call|jmp' | grep -E '__[a-z]+di[0-9a-z_]*' || true)
if [ -n "$calls" ]; then echo "avr_nodi: FAIL, 64-bit libgcc calls in segq_core.c:"; echo "$calls"; exit 1; fi
echo "avr_nodi: ok (no 64-bit helper calls)"
