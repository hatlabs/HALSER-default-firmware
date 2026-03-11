# HALSER Default Firmware

NMEA 0183 to NMEA 2000 gateway firmware for the [HALSER](https://shop.hatlabs.fi/products/halser) ESP32-C3 serial interface board.

## Features

- Receives NMEA 0183 sentences on UART1 (4800 baud)
- Translates to NMEA 2000 PGNs on CAN bus
- Supported sentences: GGA, RMC, VTG, HDG, VHW, DPT, MWV
- SensESP-based: WiFi AP/client, web UI configuration, Signal K output, OTA updates
- Unified binary includes production test mode (activated by test jig)

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run
```

## NMEA 0183 → NMEA 2000 Translation

| Input | N2K PGN | Description |
|-------|---------|-------------|
| GGA/RMC | 129029 | GNSS Position |
| RMC/VTG | 129026 | COG & SOG |
| HDG | 127250 | Vessel Heading |
| VHW | 128259 | Speed, Water Referenced |
| DPT | 128267 | Water Depth |
| MWV | 130306 | Wind Data |

## License

MIT
