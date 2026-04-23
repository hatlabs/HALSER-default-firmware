# AGENTS.md

## Project Overview

HALSER default firmware — a unified ESP32-C3 binary that serves as both the NMEA 0183→N2K gateway (normal operation) and the production test handler (test jig mode).

## Build Commands

```bash
# Build firmware
pio run

# Upload to connected board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Architecture

### Boot Routing

GPIO 0 is checked at startup:
- **HIGH** (test jig drives it) → test command mode over USB CDC serial
- **LOW** (standalone operation) → NMEA 0183→N2K gateway with full SensESP

### Source Layout

- `src/main.cpp` — Boot routing (GPIO 0 check)
- `src/halser_const.h` — Pin assignments and constants
- `src/test_mode.h/.cpp` — Production test command handler (PING, CAN_TEST, serial loopback, GPIO, HALL_TEST)
- `src/nmea_gateway.h/.cpp` — SensESP application: NMEA 0183 parsing, N2K transmission, WiFi/web UI
- `src/n2k_senders.h` — N2K message senders with value expiry

### Data Flow (Gateway Mode)

```
UART1 (4800 baud, GPIO 3 RX)
  → NMEA0183IOTask (dedicated FreeRTOS task)
  → Sentence parsers (GGA, RMC, VTG, HDG, VHW, DPT, MWV)
  → LambdaConsumer callbacks → ExpiringValue updates
  → Periodic N2K message senders
  → tNMEA2000_esp32 (TWAI, GPIO 4 TX / GPIO 5 RX)
```

### Hardware Pin Assignments

| Pin | Function |
|-----|----------|
| GPIO 0 | Test jig indicator |
| GPIO 1 | Hall effect sensor |
| GPIO 2 | UART1 TX |
| GPIO 3 | UART1 RX |
| GPIO 4 | CAN TX |
| GPIO 5 | CAN RX |
| GPIO 6 | I2C SDA |
| GPIO 7 | I2C SCL |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |
| GPIO 10 | 1-Wire |

## Dependencies

- SensESP 3.2.0 — IoT framework (WiFi, web UI, Signal K)
- SensESP/NMEA0183 — NMEA 0183 sentence parsing
- NMEA2000-library — NMEA 2000 message handling
- NMEA2000_twai — ESP32 TWAI (CAN) driver
- Adafruit NeoPixel — RGB LED control
