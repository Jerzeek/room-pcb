# Contributing to Room Sensor

Thank you for your interest in contributing! All contributions — bug reports, hardware suggestions, firmware improvements, and documentation fixes — are welcome.

## Reporting issues

- Use the GitHub Issues tracker.
- For hardware bugs, include the board version (see silkscreen or CHANGELOG.md).
- For firmware bugs, state which firmware you are using (ESPHome or Zigbee) and include your ESPHome/Arduino core version.
- Attach serial logs or Home Assistant diagnostics where relevant.

## Submitting pull requests

1. Fork the repository and create a branch from `main`.
2. Keep changes focused — one logical change per PR.
3. Update CHANGELOG.md under an `[Unreleased]` heading.
4. Open the PR against the `main` branch and describe what you changed and why.

## Hardware / KiCad contributions

- This project uses **KiCad 8**. Please do not submit files saved with an older or newer version without discussing it first in an issue.
- Keep the KiCad project files inside `pcb/`. Do not commit lock files (`~*.lck`), the `fp-info-cache`, or auto-generated backup zips — they are excluded by `.gitignore`.
- If you regenerate production outputs (Gerbers, BOM, positions), place them in `pcb/production/` and note the change in your PR description.
- 3D models (STEP files) go in `3Dmodels/`.

## Firmware contributions

Both ESPHome and Zigbee firmware are equally maintained. Contributions to either are welcome.

### ESPHome

- Config lives in `code/esphome/room_sensor.yaml`.
- The custom CozIR component is in `code/esphome/components/cozir/`. Follow the ESPHome external component conventions.
- Test your YAML with `esphome compile room_sensor.yaml` before submitting.
- Do not commit your `secrets.yaml` or any file containing Wi-Fi credentials or API keys.

### Zigbee (Arduino)

- Sketch lives in `code/combined_working_zigbee/combined_working_zigbee.ino`.
- Requires the Arduino ESP32 core (version 3.x or later) with Zigbee support enabled.
- When flashing, select **Zigbee ED (End Device)** in Tools → Zigbee Mode and a Zigbee-capable partition scheme.

## Code style

- ESPHome YAML: follow the upstream ESPHome style (2-space indentation).
- Arduino/C++: keep the existing formatting; add comments for non-obvious logic.

## Safety note

Any contribution that touches the AC mains section of the schematic or PCB layout will be reviewed with extra care. Please document your reasoning and cite relevant standards (IEC 60950, creepage/clearance requirements, etc.) in the PR description.

## Licence

By submitting a contribution you agree that it will be released under the project licence (CERN OHL-S v2).
