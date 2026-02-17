# Cozir CO2 Sensor Component for ESPHome

This component allows using Cozir CO2 sensors (like CozIR-A) with ESPHome via UART.

## Configuration

```yaml
uart:
  tx_pin: GPIO1
  rx_pin: GPIO3
  baud_rate: 9600

sensor:
  - platform: cozir
    id: my_cozir
    co2:
      name: "Cozir CO2"
    temperature:
      name: "Cozir Temperature"
    humidity:
      name: "Cozir Humidity"
    update_interval: 60s
```

## Calibration

You can calibrate the sensor using `lambda` calls or by exposing template buttons.

### Fresh Air Calibration (400ppm)
```yaml
button:
  - platform: template
    name: "Calibrate Fresh Air"
    on_press:
      - lambda: 'id(my_cozir)->calibrate_fresh_air();'
```

### Nitrogen Calibration (0ppm)
```yaml
button:
  - platform: template
    name: "Calibrate Nitrogen"
    on_press:
      - lambda: 'id(my_cozir)->calibrate_nitrogen();'
```

### Known Gas Calibration
```yaml
# Example using a template number to set target PPM
number:
  - platform: template
    id: target_ppm
    name: "Target Calibration PPM"
    min_value: 0
    max_value: 5000
    step: 10
    optimistic: true
    set_action: []

button:
  - platform: template
    name: "Calibrate Known Gas"
    on_press:
      - lambda: 'id(my_cozir)->calibrate_known_gas((uint16_t)id(target_ppm).state);'
```

## Notes
- The component automatically detects the PPM multiplier (1, 10, or 100) on each update.
- Ensure the sensor is powered correctly (3.3V typically).
