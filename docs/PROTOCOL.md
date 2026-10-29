# Wire protocol

Two layers:

1. **Transport**: Klipper's message-block format, unchanged.
2. **Commands**: stock Klipper commands only; the printer board runs unmodified Klipper.

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

## 2. Commands

The printer board runs **plain Klipper v0.13.0**, so the commands are exactly Klipper's own (from
the board's data dictionary): configuration, pins, ADC, PWM, `queue_step` / `set_next_step_dir` /
`reset_step_clock` for motion, `trsync_*` and `endstop_home` for homing, `tmcuart_send` for TMC2209
drivers, `sensor_bulk_*` for accelerometers. The ESP32-S3 uses them the way Klipper's own host does.

Until 2026-10-25 this section described a draft **segment queue** (`segq_*`) added to the board's
Klipper; it was never used by the host and has been removed. Its full specification is kept in
[research/segq-retired.md](research/segq-retired.md).
