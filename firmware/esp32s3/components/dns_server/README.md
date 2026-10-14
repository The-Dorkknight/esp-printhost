# dns_server (third-party)

Copied unchanged from ESP-IDF v5.4.1, `examples/protocols/http_server/captive_portal/components/dns_server`.
Copyright Espressif Systems; licensed **Unlicense OR CC0-1.0** (see the SPDX header in each file).
Answers every DNS "A" query with the access point's own address, so phones that check for
internet are sent to the Skipper Flipper status page (captive portal) instead of giving up.
