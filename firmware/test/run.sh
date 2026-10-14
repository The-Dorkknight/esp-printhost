#!/bin/sh
# Build and run the PC simulation of the Klipper glue (needs only gcc).
set -e
cd "$(dirname "$0")"
gcc -std=gnu99 -Wall -Wextra -g -fsanitize=address,undefined -Imock -I../skipper \
    -o /tmp/segq_glue_sim segq_glue_sim.c ../skipper/segq.c ../skipper/segq_core.c
ASAN_OPTIONS=detect_leaks=0 /tmp/segq_glue_sim
gcc -std=gnu99 -Wall -Wextra -g -fsanitize=address,undefined -fno-sanitize-recover=undefined \
    -I../skipper -o /tmp/segq_core_edge segq_core_edge.c ../skipper/segq_core.c
ASAN_OPTIONS=detect_leaks=0 /tmp/segq_core_edge
sh ./avr_nodi.sh
