# Changelog

Format follows [Keep a Changelog](https://keepachangelog.com/); versions follow [Semantic Versioning](https://semver.org/). Everything before 1.0 is experimental.

## [0.1.0-wip] - 2026-10-01

First public snapshot. **Proof of concept; many features may be broken.**

### Added
- Portable C++17 core: Klipper wire format, data dictionary, `printer.cfg` parser, G-code parser, step accumulator, pseudo pressure advance (122 unit tests)
- `mcu_probe`: identify a real Klipper board, list pins, validate pin names, reset if already configured, toggle one pin
- `fake_mcu.py`: simulated board for testing without hardware
- `printhost.py`: one-command Linux setup and launcher
- `printhost_gui.py`: Tkinter GUI with copyable output and logging, version and build date in the title and status bar
- Design, protocol (draft) and task documents

### Fixed
- GUI status bar and horizontal scrollbar were clipped at the default window size
