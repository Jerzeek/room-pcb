#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include <string>

namespace esphome {
namespace cozir {

class CozirComponent : public PollingComponent, public uart::UARTDevice {
 public:
  void set_co2_sensor(sensor::Sensor *co2_sensor) { co2_sensor_ = co2_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }
  void set_ppm_factor(uint16_t ppm_factor) { ppm_factor_ = ppm_factor; } // Allow manual override if needed

  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;

  // Calibration actions
  void calibrate_fresh_air();
  void calibrate_nitrogen();
  void calibrate_known_gas(uint16_t ppm);
  void set_digital_filter(uint8_t value);
  void get_version();

 protected:
  sensor::Sensor *co2_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  
  uint16_t ppm_factor_{1}; // Default multiplier

  enum State {
    STATE_IDLE,
    STATE_WAIT_CO2,
    STATE_WAIT_TEMP,
    STATE_WAIT_HUMIDITY,
    STATE_WAIT_PPM_FACTOR,
  } state_{STATE_IDLE};

  uint32_t last_transmission_{0};
  std::string buffer_;

  void send_command_(const std::string &command);
  void process_line_(std::string &line);
};

}  // namespace cozir
}  // namespace esphome
