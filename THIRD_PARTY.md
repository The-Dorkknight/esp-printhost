# Third-party material

The GPL-3.0 licence in this repository covers Matthew Armstrong's code only. Everything below keeps its own licence and belongs to its authors. This project is not affiliated with, endorsed by or sponsored by any of them.

| What | How it is used | Licence / link |
|---|---|---|
| **Klipper** (MCU firmware and `msgproto` protocol) | The wire format, data-dictionary layout and command names were learned from Klipper's source and implemented independently in C++. A future firmware fork would be a derivative work. | GPL-3.0, <https://github.com/Klipper3d/klipper> |
| **FluidNC** | Planned donor for the ESP32 side; no FluidNC code is in this repository yet. | GPL-3.0, <https://github.com/bdring/FluidNC> |
| **Marlin** | Inspiration for the pressure-advance idea; no code copied. | GPL-3.0, <https://github.com/MarlinFirmware/Marlin> |
| **zlib** | Linked by `mcu_probe` (system library) to unpack the board's data dictionary. | zlib licence, <https://zlib.net> |
| **Tk / Python** | The GUI uses the Python standard library and tkinter. | PSF licence |

Licences are linked, not reproduced. Check each project for its current terms.
