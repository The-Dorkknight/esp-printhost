# Wire protocol

Two layers:

1. **Transport** — Klipper's message-block format, unchanged, so the forked
   MCU firmware keeps its existing framing/serial code.
2. **Commands** — stock Klipper commands (config, pins, ADC, shutdown) plus a
   new **segment queue** command set (draft, below).

## 1. Transport (Klipper compatible)

```
<len:1> <seq:1> <content: 0..58> <crc_hi:1> <crc_lo:1> <sync:1 = 0x7E>
```

- `len` = whole block incl. header+trailer, 5..64.
- `seq` = `0x10 | (n & 0x0F)`. Host→MCU: next sequence number. MCU→host: ack of
  next expected seq.
- CRC16-CCITT (Klipper variant, init 0xFFFF) over `len, seq, content`.
- An empty block (len 5) from the MCU is a pure ACK.
- `content` = one or more messages: `<msgid:vlq> <params...>`.

### Parameter encoding

| format | encoding |
|---|---|
| `%u %i %hu %hi %c` | VLQ (Klipper variant, see `host/src/wire.cpp`) |
| `%s` | 1-byte length + bytes (NUL-terminated string on MCU side) |
| `%.*s` / `%*s` | 1-byte length + bytes |

### Handshake

1. Host sends `identify offset=%u count=%c` (msgid 1) repeatedly, 40 bytes at a
   time, until a short chunk arrives.
2. MCU replies `identify_response offset=%u data=%.*s` (msgid 0).
3. Concatenated data = zlib-compressed JSON data dictionary:
   `{"commands":{fmt:id}, "responses":{fmt:id}, "enumerations":{...},
     "config":{"CLOCK_FREQ":..,"MCU":..}, "version":...}`
4. ESP32: inflate with ROM miniz (`tinfl_decompress`). PC tests: system zlib.

## 2. Segment queue (NEW, draft v0)

Replaces Klipper's `queue_step` / stepcompress for our boards.
Host plans moves and emits constant-rate segments. MCU runs a DDA.

### Config (sent once after identify)

```
config_segq oid=%c axes=%c depth=%hu
segq_set_stepper oid=%c axis=%c stepper_oid=%c
```

`stepper_oid` refers to a stock `config_stepper` (pins, invert, pulse ticks).

### Runtime

```
segq_push oid=%c ticks=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu
```

- `ticks` = segment duration in MCU clock ticks.
- `x..e` = **signed** step counts for this segment. Sign = direction.
  E may be negative mid-print (pressure advance retract on decel).
- `flags`:
  - bit0 `SYNC` — when this segment *starts*, apply pending pin actions with
    matching `tag` (fan/heater changes lined up with motion).
  - bit1 `LAST` — end of a motion burst; MCU may idle.
  - bit2 `RESERVED_PA` — reserved for v2.
- MCU distributes each axis's steps evenly over `ticks` (Bresenham/DDA).

```
segq_pin_on_tag oid=%c pin_oid=%c tag=%hu value=%hu
segq_status oid=%c free=%hu executing_tag=%hu
segq_stop oid=%c
```

- `segq_status` is sent by the MCU when free space crosses a threshold and on
  request. Host keeps ≥ ~100 ms of segments queued.
- `segq_stop` = flush queue, stop now (pause/emergency). MCU replies with
  `segq_position oid=%c x=%i y=%i z=%i e=%i` (absolute steps).

### Homing

```
segq_endstop_arm oid=%c axis=%c pin=%u pull_up=%c invert=%c
segq_endstop_hit oid=%c axis=%c x=%i y=%i z=%i e=%i      (MCU -> host)
```

MCU polls the endstop in its DDA ISR; on trigger it flushes the queue and
reports absolute step position. No host-side timing needed.

### Watchdog

If no `segq_push` / `get_uptime` arrives for 2 s while heaters are on, MCU
calls `shutdown("host timeout")` (turns everything off, Klipper behaviour).

### Budget

Segment ~ 5–8 bytes payload; at 200 segments/s → ~2 KB/s. Fine at 250 kbaud.
