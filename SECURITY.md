# Security

This is an early proof of concept and is not intended for production or safety-critical use.

## What anyone on the ESP32's WiFi can do

The ESP32-S3 has no user accounts. Anyone who can reach its pages (its own WPA2 network
SkipperFlipper-XXXX, and, if you set one, your home WiFi) can use the printer: heat, move, print,
change `printer.cfg` and settings. On the CAPTAIN and SKIPPER builds that includes **flashing new
firmware over WiFi** (`/update`; only an image of the same build is accepted, and a build that crashes
in its first 30 s is rolled back, but it is not signed). The slicer upload (`/api/files/local`) does
not check the API key. So: change the default WiFi password (`idf.py menuconfig`, see
firmware/esp32s3/README.md) and only join the ESP32 to a home network you trust.

To report a security problem, use GitHub's private reporting: **Security tab > Report a vulnerability** on this repository. Please do not open a public issue for it.
