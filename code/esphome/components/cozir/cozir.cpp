#include "cozir.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstdlib>
#include <cstdio>

namespace esphome {
namespace cozir {

static const char *const TAG = "cozir";

void CozirComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Cozir CO2 Sensor...");
  
  // Set to polling mode
  ESP_LOGD(TAG, "Sending 'K 2' to set polling mode");
  this->write_str("K 2\r\n");
  delay(100); // Give it a moment
  
  // Clear buffer
  int flushed = 0;
  while (this->available()) {
    this->read();
    flushed++;
  }
  ESP_LOGD(TAG, "Flushed %d bytes from buffer", flushed);
}

void CozirComponent::update() {
  ESP_LOGD(TAG, "Update cycle started");
  if (this->state_ != STATE_IDLE) {
    ESP_LOGW(TAG, "Previous update cycle not finished yet! State: %d", this->state_);
    return;
  }

  // Send request for PPM Factor
  this->send_command_(".");
  this->state_ = STATE_WAIT_PPM_FACTOR;
}

void CozirComponent::loop() {
  while (this->available()) {
    char c = this->read();
    ESP_LOGV(TAG, "Received char: '%c' (0x%02X)", c, c); // Verbose log every char
    if (c == '\n') {
      ESP_LOGD(TAG, "Line received: %s", this->buffer_.c_str());
      this->process_line_(this->buffer_);
      this->buffer_.clear();
    } else if (c != '\r') {
      this->buffer_ += c;
    }
  }

  if (this->state_ != STATE_IDLE && millis() - this->last_transmission_ > 500) {
    ESP_LOGW(TAG, "Timeout waiting for response in state %d", this->state_);
    this->state_ = STATE_IDLE;
    this->buffer_.clear();
  }
}

void CozirComponent::send_command_(const std::string &command) {
  ESP_LOGD(TAG, "Sending command: %s", command.c_str());
  this->write_str(command.c_str());
  this->write_str("\r\n");
  this->last_transmission_ = millis();
}

void CozirComponent::process_line_(std::string &line) {
  if (line.empty()) return;

  // Trim leading spaces
  size_t first_char = line.find_first_not_of(" \t\r\n");
  if (first_char == std::string::npos) return; // Empty or all whitespace

  char cmd = line[first_char];
  
  // Value starts after the command char
  // e.g., " . 00001" -> cmd='.', value="00001"
  // e.g., "Z 00400" -> cmd='Z', value="00400"
  
  size_t value_start = first_char + 1;
  while (value_start < line.length() && line[value_start] == ' ') {
    value_start++;
  }
  
  std::string value_str = line.substr(value_start);
  int value = atoi(value_str.c_str());

  ESP_LOGV(TAG, "Parsed cmd: '%c', value: %d", cmd, value);

  switch (this->state_) {
    case STATE_WAIT_PPM_FACTOR:
      if (cmd == '.') {
        this->ppm_factor_ = value;
        ESP_LOGD(TAG, "PPM Factor: %d", this->ppm_factor_);
        
        // Next: CO2
        if (this->co2_sensor_ != nullptr) {
          this->send_command_("Z");
          this->state_ = STATE_WAIT_CO2;
        } else {
            // Skip to Temp
             if (this->temperature_sensor_ != nullptr) {
                this->send_command_("T");
                this->state_ = STATE_WAIT_TEMP;
             } else if (this->humidity_sensor_ != nullptr) {
                this->send_command_("H");
                this->state_ = STATE_WAIT_HUMIDITY;
             } else {
                this->state_ = STATE_IDLE;
             }
        }
      }
      break;

    case STATE_WAIT_CO2:
      if (cmd == 'Z') {
        float co2 = value * this->ppm_factor_;
        if (this->co2_sensor_ != nullptr) {
            this->co2_sensor_->publish_state(co2);
        }

        // Next: Temp
        if (this->temperature_sensor_ != nullptr) {
          this->send_command_("T");
          this->state_ = STATE_WAIT_TEMP;
        } else if (this->humidity_sensor_ != nullptr) {
          this->send_command_("H");
          this->state_ = STATE_WAIT_HUMIDITY;
        } else {
          this->state_ = STATE_IDLE;
        }
      }
      break;

    case STATE_WAIT_TEMP:
      if (cmd == 'T') {
        // Formula from library: 0.1 * (rv - 1000.0)
        float temp = 0.1f * (value - 1000.0f);
        if (this->temperature_sensor_ != nullptr) {
          this->temperature_sensor_->publish_state(temp);
        }

        // Next: Humidity
        if (this->humidity_sensor_ != nullptr) {
          this->send_command_("H");
          this->state_ = STATE_WAIT_HUMIDITY;
        } else {
          this->state_ = STATE_IDLE;
        }
      }
      break;

    case STATE_WAIT_HUMIDITY:
      if (cmd == 'H') {
        // Formula from library: 0.1 * rv
        float humidity = 0.1f * value;
        if (this->humidity_sensor_ != nullptr) {
          this->humidity_sensor_->publish_state(humidity);
        }
        this->state_ = STATE_IDLE;
      }
      break;
      
    default:
      // Ignore unsolicited messages or echo
      break;
  }
}

void CozirComponent::calibrate_fresh_air() {
  ESP_LOGI(TAG, "Calibrating Fresh Air (400ppm)...");
  this->write_str("G\r\n");
}

void CozirComponent::calibrate_nitrogen() {
  ESP_LOGI(TAG, "Calibrating Nitrogen (0ppm)...");
  this->write_str("U\r\n");
}

void CozirComponent::calibrate_known_gas(uint16_t ppm) {
  ESP_LOGI(TAG, "Calibrating Known Gas (%d ppm)...", ppm);
  char buffer[16];
  sprintf(buffer, "X %u\r\n", ppm);
  this->write_str(buffer);
}

void CozirComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Cozir:");
  LOG_SENSOR("  ", "CO2", this->co2_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

}  // namespace cozir
}  // namespace esphome