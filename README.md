# Room Sensor

[![License: CERN OHL-S v2](https://img.shields.io/badge/License-CERN%20OHL--S%20v2-green.svg)](LICENSE)

An open-source, mains-powered room sensor PCB based on the ESP32-C6. Measures CO2, temperature, humidity, illuminance, and presence, with a WS2812B LED status indicator. Supports both **ESPHome** (Wi-Fi / Home Assistant) and **Zigbee** firmware out of the box.

---

> **WARNING — MAINS AC VOLTAGE**
>
> This board connects directly to mains AC (100–240 V). Incorrect wiring or handling can cause **electric shock, fire, or death**. This project is intended for **experienced makers** who are comfortable working with mains-voltage electronics. Always work with the power disconnected. Comply with all local electrical codes. The authors accept no liability for any damage or injury.

---

## Features

- Direct mains AC input via onboard HLK-5M05 AC-DC converter (no external power brick needed)
- CO2, temperature, and humidity via CozIR sensor (NDIR CO2, UART)
- Ambient light via BH1750 (I2C, lux)
- Secondary temperature + humidity via AHT20 (I2C)
- Presence/occupancy detection via LD2410 mmWave radar (UART)
- Visual CO2 status indicator: 2× WS2812B addressable LEDs
- Polyfuse (F1) overcurrent protection
- Reset/factory-reset button (BOOT pin)
- ESP32-C6 SuperMini module — supports both Wi-Fi 6 and native Zigbee
- Two equal-priority firmware options: ESPHome and Zigbee (Arduino)

## Sensors

| Sensor | Part | Interface | Measures |
|---|---|---|---|
| CO2 (NDIR) | CozIR | UART (9600 baud) | CO2 (ppm), temperature, humidity |
| Ambient light | BH1750FVI-TR | I2C (0x23) | Illuminance (lux) |
| Temperature + humidity | AHT20 | I2C | Temperature (°C), humidity (%RH) |
| Presence / occupancy | LD2410 | UART (256000 baud) | Presence, moving/stationary targets |
| Status LEDs | WS2812B-2020 × 2 | GPIO (RMT) | CO2 level indicator |

## Hardware

### Main components

| Ref | Part | Description |
|---|---|---|
| U5 | ESP32-C6 SuperMini | Main MCU — Wi-Fi 6 + Zigbee |
| PS1 | HLK-5M05 | AC-DC converter, 100–240 V AC → 5 V DC |
| U1 | CozIR | NDIR CO2 / temp / humidity sensor |
| U2 | BH1750FVI-TR | Ambient light sensor |
| U3 | AHT20 | Temperature + humidity sensor |
| J4 | 5-pin header (1.27 mm) | LD2410 radar connection |
| D1, D2 | WS2812B-2020 | Addressable RGB LEDs |
| F1 | Polyfuse | Overcurrent protection |

### Versions

| Version | Notes |
|---|---|
| v4.1 | Current release — improved silkscreen |
| v4 | Production-ready layout |
| v3.1 | Earlier revision (3D model reference) |
| v3 | Initial working prototype |

## Building / Ordering

All files needed to order and assemble the PCB are in `pcb/production/`:

| File | Purpose |
|---|---|
| `room-pcb.zip` | Gerber + drill files — upload directly to your PCB fab (JLCPCB, PCBWay, etc.) |
| `bom.csv` | Bill of materials for component sourcing |
| `positions.csv` | Pick-and-place / component placement file |
| `designators.csv` | Designator reference |
| `netlist.ipc` | IPC-D-356 netlist for electrical testing |

KiCad source files are in `pcb/`.

## Firmware

### ESPHome (Wi-Fi / Home Assistant)

Configuration is in `code/esphome/room_sensor.yaml`. A custom ESPHome component for the CozIR sensor is included under `code/esphome/components/cozir/`.

1. Copy `room_sensor.yaml` and the `components/` directory to your ESPHome config folder.
2. Create a `secrets.yaml` with your `wifi_ssid` and `wifi_password`.
3. Flash via `esphome run room_sensor.yaml`.

The LED indicates CO2 level:

| Color | CO2 |
|---|---|
| Green | < 800 ppm (good) |
| Yellow | 800–1200 ppm (moderate) |
| Red | > 1200 ppm (poor) |

Calibration buttons are exposed to Home Assistant:
- **Calibrate Fresh Air (400 ppm)** — use outdoors in clean air
- **Calibrate Nitrogen (0 ppm)** — use with pure N2

### Zigbee (Arduino / esp32-arduino)

Sketch is in `code/combined_working_zigbee/combined_working_zigbee.ino`. Requires the Arduino ESP32 core (≥ 3.x) with Zigbee support.

**Before flashing:**
- In Arduino IDE → Tools → Zigbee Mode → select **Zigbee ED (End Device)**
- Select a partition scheme that includes Zigbee (e.g., *Zigbee 4MB with spiffs*)

The sketch exposes four Zigbee endpoints:

| Endpoint | Cluster | Data |
|---|---|---|
| 10 | CO2 (CarbonDioxide) | CO2 ppm |
| 11 | Temp + Humidity | Temperature °C, humidity %RH |
| 13 | Illuminance | Lux |
| 14 | Occupancy | Presence (boolean) |

Hold the BOOT button for 3 seconds to perform a Zigbee factory reset.

## Repository layout

```
room-pcb/
├── pcb/                  KiCad project files
│   └── production/       Gerbers, BOM, placement — ready to order
├── code/
│   ├── esphome/          ESPHome YAML + custom CozIR component
│   └── combined_working_zigbee/  Arduino Zigbee sketch
├── 3Dmodels/             STEP model of the enclosure / board
├── CHANGELOG.md
├── CONTRIBUTING.md
└── LICENSE
```

## License

Hardware (PCB and schematics) is licensed under the **CERN Open Hardware Licence Version 2 — Strongly Reciprocal (CERN OHL-S v2)**. See [LICENSE](LICENSE) for the full text.

Firmware is licensed under the same CERN OHL-S v2 unless a file header states otherwise.
