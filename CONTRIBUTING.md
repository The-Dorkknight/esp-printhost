# Contributing

This is a slow-moving proof of concept. Feedback is welcome; expect replies to be slow and the design to change.

- **Most useful:** hardware reports ("probe on board X printed Y"), protocol corrections, bug reports with logs.
- **Before a pull request,** please open an issue first so we can agree on the direction. Big changes are premature while the design is moving.
- Build and test with `python3 host/tools/build.py`; all unit tests must pass.
- C++17, warnings are errors (`-Wall -Wextra -Werror`). Python: standard library only.
- By contributing you agree your contribution is licensed under the same GPL-3.0 licence as the project.

Third-party code must come with a compatible licence and be listed in [THIRD_PARTY.md](THIRD_PARTY.md).
