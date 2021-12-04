#pragma once

#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace esphome {
namespace emporia_vue {

struct __attribute__((__packed__)) ReadingPowerEntry {
  int32_t phase_black;
  int32_t phase_red;
  int32_t phase_blue;
};

struct __attribute__((__packed__)) SensorReading {
  bool is_unread;
  uint8_t checksum;
  uint8_t unknown;
  uint8_t sequence_num;

  ReadingPowerEntry power[19];

  uint16_t voltage[3];
  uint16_t frequency;
  uint16_t degrees[2];

  uint16_t current[19];

  uint16_t end;
};

class PhaseConfig;
class CTClampConfig;

class EmporiaVueComponent : public Component, public i2c::I2CDevice {
 public:
  void dump_config() override;

  void set_sensor_poll_interval(uint32_t sensor_poll_interval) { this->sensor_poll_interval_ = sensor_poll_interval; }
  uint32_t get_sensor_poll_interval() const { return this->sensor_poll_interval_; }
  void set_phases(std::vector<PhaseConfig *> phases) { this->phases_ = phases; }
  void set_ct_clamps(std::vector<CTClampConfig *> ct_clamps) { this->ct_clamps_ = ct_clamps; }

  void setup() override;
  void loop() override;

 private:
  static void i2c_request_task(void *pv);

  uint32_t sensor_poll_interval_;
  std::vector<PhaseConfig *> phases_;
  std::vector<CTClampConfig *> ct_clamps_;
  QueueHandle_t i2c_data_queue_;
};

enum PhaseInputWire : uint8_t {
  BLACK = 0,
  RED = 1,
  BLUE = 2,
};

class PhaseConfig {
 public:
  void set_input_wire(PhaseInputWire input_wire) { this->input_wire_ = input_wire; }
  PhaseInputWire get_input_wire() const { return this->input_wire_; }
  void set_calibration(float calibration) { this->calibration_ = calibration; }
  float get_calibration() const { return this->calibration_; }
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { this->voltage_sensor_ = voltage_sensor; }
  sensor::Sensor *get_voltage_sensor() const { return this->voltage_sensor_; }

  void update_from_reading(const SensorReading &sensor_reading);

  int32_t extract_power_for_phase(const ReadingPowerEntry &power_entry);

 private:
  PhaseInputWire input_wire_;
  float calibration_;
  sensor::Sensor *voltage_sensor_{nullptr};
};

enum CTInputPort : uint8_t {
  A = 0,
  B = 1,
  C = 2,
  ONE = 3,
  TWO = 4,
  THREE = 5,
  FOUR = 6,
  FIVE = 7,
  SIX = 8,
  SEVEN = 9,
  EIGHT = 10,
  NINE = 11,
  TEN = 12,
  ELEVEN = 13,
  TWELVE = 14,
  THIRTEEN = 15,
  FOURTEEN = 16,
  FIFTEEN = 17,
  SIXTEEN = 18,
};

class CTClampConfig : public sensor::Sensor {
 public:
  void set_phase(PhaseConfig *phase) { this->phase_ = phase; };
  const PhaseConfig *get_phase() const { return this->phase_; }
  void set_input_port(CTInputPort input_port) { this->input_port_ = input_port; };
  CTInputPort get_input_port() const { return this->input_port_; }
  void set_power_sensor(sensor::Sensor *power_sensor) { this->power_sensor_ = power_sensor; }
  sensor::Sensor *get_power_sensor() const { return this->power_sensor_; }
  void set_current_sensor(sensor::Sensor *current_sensor) { this->current_sensor_ = current_sensor; }
  sensor::Sensor *get_current_sensor() const { return this->current_sensor_; }

  void update_from_reading(const SensorReading &sensor_reading);

 private:
  float get_calibrated_power(int32_t raw_power) const;

  PhaseConfig *phase_;
  CTInputPort input_port_;
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
};

}  // namespace emporia_vue
}  // namespace esphome
