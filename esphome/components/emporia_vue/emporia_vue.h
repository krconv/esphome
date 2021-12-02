#pragma once

#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace emporia_vue {

struct __attribute__((__packed__)) ReadingPowerEntry {
  int32_t phase_black;  // TODO: Verify order
  int32_t phase_red;
  int32_t phase_blue;
};

struct __attribute__((__packed__)) SensorReading {
  bool is_unread;
  uint8_t checksum;  // TODO: verify
  uint8_t unknown;
  uint8_t sequence_num;

  ReadingPowerEntry power[19];

  uint16_t voltage[3];
  uint16_t frequency;
  uint16_t degrees[2];

  uint16_t current[19];

  uint16_t end;
};

enum PhaseInputWire : uint8_t { BLACK, RED, BLUE };

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

class PhaseConfig;
class PowerSensor;

class EmporiaVueComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void dump_config() override;

  void set_sensor_poll_interval(uint32_t sensor_poll_interval);
  void set_phases(std::vector<PhaseConfig *> phases);
  void set_power_sensors(std::vector<PowerSensor *> sensors);

  void loop() override;
  void update() override;

 private:
  uint32_t sensor_poll_interval_;
  uint32_t last_poll_ = 0;
  uint8_t last_sequence_num_ = 0;
  std::vector<PhaseConfig *> phases_;
  std::vector<PowerSensor *> power_sensors_;
};

class PhaseConfig {
 public:
  void set_input_wire(PhaseInputWire input_wire);
  PhaseInputWire get_input_wire() const;

  void set_calibration(float calibration);
  float get_calibration() const;

  int32_t extract_reading_for_phase(const ReadingPowerEntry &power_entry) const;

 private:
  PhaseInputWire input_wire_;
  float calibration_;
};
class PowerSensor : public sensor::Sensor {
 public:
  void set_phase(PhaseConfig *phase);
  const PhaseConfig *get_phase() const;

  void set_input_port(CTInputPort input_port);
  CTInputPort get_input_port() const;

  void record(const SensorReading &sensor_reading);

  void publish();

 private:
  friend class EmporiaVueComponent;

  float calibrate_value(float raw_value) const;

  int64_t total_since_publish_ = 0;
  uint32_t readings_since_publish_ = 0;

  PhaseConfig *phase_;
  CTInputPort input_port_;
};

}  // namespace emporia_vue
}  // namespace esphome
