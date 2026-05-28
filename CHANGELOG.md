# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Hardware versions follow a `vMAJOR.MINOR` scheme; minor bumps are layout/silkscreen changes, major bumps indicate schematic changes.

---

## [Unreleased]

---

## [v4.1] - 2026-04

### Changed
- Improved silkscreen legibility — component labels and reference designators cleaned up across the board.
- Updated Gerbers and production files to match v4.1 layout.

---

## [v4] - 2026-03

### Added
- Production-ready layout with all sensors placed and routed.
- HLK-5M05 footprint corrected and mains clearances verified.
- Polyfuse (F1) added for overcurrent protection on the 5 V rail.
- LD2410 radar header (J4, 1.27 mm pitch, 5-pin) added.
- JST-SH connector (J3) for I2C breakout.
- Two WS2812B-2020 LEDs (D1, D2) as CO2 status indicators.
- Combined working Zigbee sketch (`combined_working_zigbee.ino`) with CO2, temp/humidity, illuminance, and occupancy endpoints.
- ESPHome configuration (`room_sensor.yaml`) with custom CozIR component.

### Changed
- Switched MCU to ESP32-C6 SuperMini module for native Zigbee support.
- Resolved KiCad merge conflicts from concurrent edits.

---

## [v3.1] - 2026-02

### Changed
- Minor layout adjustments (captured in STEP model `home sensor v31.step`).
- Test points (TP1–TP3) repositioned for easier probing.

---

## [v3] - 2026-01

### Added
- Initial working prototype with all sensor ICs populated.
- First ESPHome YAML and CozIR Arduino library integrated.
- 3D model added to repository.

---

[Unreleased]: https://github.com/Jerzeek/room-pcb/compare/v4.1...HEAD
[v4.1]: https://github.com/Jerzeek/room-pcb/compare/v4...v4.1
[v4]: https://github.com/Jerzeek/room-pcb/compare/v3.1...v4
[v3.1]: https://github.com/Jerzeek/room-pcb/compare/v3...v3.1
[v3]: https://github.com/Jerzeek/room-pcb/releases/tag/v3
