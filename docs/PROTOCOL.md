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

## 2. Segment queue (protocol v1)

Replaces Klipper's `queue_step` / stepcompress for our boards. The host plans
moves and sends constant-velocity **segments**; the MCU runs a DDA that turns
them into step pulses. The MCU side is `firmware/skipper/segq_core.[ch]`
(portable algorithm, unit tested on a PC) plus `firmware/skipper/segq.c`
(Klipper glue). `tools/fake_mcu.py` simulates it for host tests.

The firmware announces `SEGQ_PROTOCOL=1`, `SEGQ_MAX_DEPTH=255` and `SEGQ_AXES=4`
in the data dictionary's `config` section. A host must check `SEGQ_PROTOCOL`.

### Commands (host -> MCU)

```
config_segq oid=%c period=%u depth=%c start_threshold=%c
config_segq_axis oid=%c axis=%c step_pin=%u dir_pin=%u invert_step=%c invert_dir=%c
segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu
segq_stop oid=%c
segq_query oid=%c
```

| command | meaning |
|---|---|
| `config_segq` | Create the queue (once, during config, after `allocate_oids`). `period` = length of one DDA tick in MCU clock ticks (`CLOCK_FREQ / dda_hz`), 1..2^30 (larger values break Klipper's wrapping time compares and are rejected). `depth` = queue length in segments, 1..255 (RAM: depth x 24 bytes on ARM, 23 on AVR). `start_threshold` = how many segments must be queued before an idle queue starts (0 or 1 = at once, must be <= depth). A segment with the LAST flag also starts it. |
| `config_segq_axis` | Attach pins to an axis: `axis` 0..3 = X, Y, Z, E. `invert_step` / `invert_dir` = 1 flips the pin level. Axes never configured are simply never stepped (their motion is dropped, but still counted in `segq_position`). Direction pins start at the "positive" level. |
| `segq_push` | Append a segment: move each axis by exactly the signed step count over `n` DDA ticks (sign = direction; E may be negative). |
| `segq_stop` | Drop everything queued and stop now (pause / emergency). The position is kept. |
| `segq_query` | Ask for status and position; answered by `segq_status` **and** `segq_position`. |

`tag` of every segment is echoed in `segq_status` once it starts. `flags` of
`segq_push`: bit0 `SYNC` (0x01) is accepted but **has no effect in v1** (no pin
action); bit1 `LAST` (0x02) marks the
end of a motion burst, so running dry after it is normal (not an underrun).

Segment rules (checked on the MCU; also by `fake_mcu.py`):

- `1 <= n <= 2^30` DDA ticks.
- `2 * |steps[axis]| <= n` for every axis. Max step rate is therefore half the
  DDA rate (`dda_hz / 2` per axis); every step pulse is one DDA tick high and at
  least one tick low.
- A segment moves each axis by exactly its step count (no rounding drift).
- The first tick of a segment never steps, so the direction pin has a full tick
  to settle.

### Responses (MCU -> host), both sent in answer to `segq_query`

```
segq_status oid=%c free=%c active=%c tag=%hu underruns=%hu
segq_position oid=%c x=%i y=%i z=%i e=%i
```

- `free` free queue slots (capped at 255), `active` 1 while a segment is
  executing or queued or a step pulse is still high, `tag` = tag of the segment
  being executed (last started), `underruns` = times the queue ran dry in the
  middle of a burst (no LAST flag) since config.
- `x..e` absolute executed step position since config.

### Errors

An invalid segment (rule violation) or a push into a full queue (a host
flow-control bug: check `free` before pushing) makes the MCU call Klipper's
`shutdown()` with a clear message ("segq: invalid segment ...", "segq: queue
overflow ..."). The same applies to bad `config_segq` / `config_segq_axis`
arguments (including a second `config_segq` for an oid in use) and to a DDA timer that cannot keep up ("segq DDA timer too slow").
After a shutdown all pins go to their idle level and the board must be reset.

### Timing model

One Klipper timer runs every `period` clock ticks **only while there is work**:
it starts when a push arrives while idle and stops when the queue is empty, no
segment is executing and no pulse is high. Each tick first lowers the step pins raised on the previous tick,
then raises the pins of the axes stepping now. If the MCU cannot finish a tick
before the next is due, it shuts down instead of stepping late. The host should
keep roughly 100 ms of segments queued; a segment is a few tens of bytes on the wire, so
200 segments/s is well within 250 kbaud.

### Not in v1

- **Sync pin actions** (`segq_pin_on_tag`: fan/heater changes lined up with a
  segment): not implemented. `SYNC` only echoes the tag in `segq_status`.
- **Endstop homing in the DDA** (`segq_endstop_arm` / `segq_endstop_hit`): not
  implemented. Use Klipper's stock endstop/trsync commands or a separate move.
- **Host-timeout watchdog**: not implemented in the segq glue (Klipper's own
  clocksync/shutdown behaviour applies).
- Pressure-advance flags (`RESERVED_PA`): reserved for a later version.

### Status

Compile-tested for RP2040, STM32F103 and AVR (ATmega2560/1284P); **never run on
hardware**. The unit tests cover the algorithm core on a PC and the wire format
via `fake_mcu.py`; timing, pin polarity and ISR load are unverified.
