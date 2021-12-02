#include "emporia_vue.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace emporia_vue {

static const char *const TAG = "emporia_vue";

void EmporiaVueComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Emporia Vue");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Sensor Poll Interval: %dms", this->sensor_poll_interval_);

  for (auto *phase : this->phases_) {
    std::string wire;
    switch (phase->get_input_wire()) {
      case PhaseInputWire::BLACK:
        wire = "BLACK";
        break;
      case PhaseInputWire::RED:
        wire = "RED";
        break;
      case PhaseInputWire::BLUE:
        wire = "BLUE";
        break;
    }
    ESP_LOGCONFIG(TAG, "  Phase Config");
    ESP_LOGCONFIG(TAG, "    Wire: %s", wire.c_str());
    ESP_LOGCONFIG(TAG, "    Calibration: %f", phase->get_calibration());
  }

  for (auto *power_sensor : this->power_sensors_) {
    LOG_SENSOR("  ", "Power", power_sensor);
    ESP_LOGCONFIG(TAG, "    Phase Calibration: %f", power_sensor->get_phase()->get_calibration());
    ESP_LOGCONFIG(TAG, "    CT Port Index: %d", power_sensor->get_input_port());
  }
}

void EmporiaVueComponent::set_sensor_poll_interval(uint32_t sensor_poll_interval) {
  this->sensor_poll_interval_ = sensor_poll_interval;
}

void EmporiaVueComponent::set_phases(std::vector<PhaseConfig *> phases) { this->phases_ = phases; }

void EmporiaVueComponent::set_power_sensors(std::vector<PowerSensor *> power_sensors) {
  this->power_sensors_ = power_sensors;
}

void EmporiaVueComponent::loop() {
  uint32_t now = millis();

  if (now < this->last_poll_ + this->sensor_poll_interval_) {
    return;
  }

  SensorReading sensor_reading;

  i2c::ErrorCode err = this->read(reinterpret_cast<uint8_t *>(&sensor_reading), sizeof(sensor_reading));
  if (err != i2c::ErrorCode::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read from sensor due to I2C error %d", err);
    return;
  }

  if (!sensor_reading.is_unread) {
    ESP_LOGV(TAG, "Ignoring sensor reading that is marked as read");
    return;
  }

  ESP_LOGV(TAG, "Received sensor reading with sequence #%d", sensor_reading.sequence_num);

  if (this->last_sequence_num_ && sensor_reading.sequence_num > this->last_sequence_num_ + 1) {
    ESP_LOGW(TAG, "Detected %d missing reading(s), data may not be accurate!",
             sensor_reading.sequence_num - this->last_sequence_num_ - 1);
  }

  ESP_LOGV(TAG, "Recording sensor reading for %d power sensor(s)...", this->power_sensors_.size());
  for (auto power_sensor : this->power_sensors_) {
    power_sensor->record(sensor_reading);
  }

  this->last_sequence_num_ = sensor_reading.sequence_num;
  this->last_poll_ = now;
}

void EmporiaVueComponent::update() {
  ESP_LOGD(TAG, "Publishing %d power sensor(s)...", this->power_sensors_.size());
  for (PowerSensor *power_sensor : this->power_sensors_) {
    power_sensor->publish();
  }
}

void PhaseConfig::set_input_wire(PhaseInputWire input_wire) { this->input_wire_ = input_wire; }

PhaseInputWire PhaseConfig::get_input_wire() const { return this->input_wire_; }

void PhaseConfig::set_calibration(float calibration) { this->calibration_ = calibration; }

float PhaseConfig::get_calibration() const { return this->calibration_; }

int32_t PhaseConfig::extract_reading_for_phase(const ReadingPowerEntry &power_entry) const {
  switch (this->input_wire_) {
    case PhaseInputWire::BLACK:
      return power_entry.phase_black;
    case PhaseInputWire::RED:
      return power_entry.phase_red;
    case PhaseInputWire::BLUE:
      return power_entry.phase_blue;
    default:
      ESP_LOGE(TAG, "Unsupported phase input wire, this should never happen");
      return -1;
  }
}

void PowerSensor::set_phase(PhaseConfig *phase) { this->phase_ = phase; }

const PhaseConfig *PowerSensor::get_phase() const { return this->phase_; }

void PowerSensor::set_input_port(CTInputPort input_port) { this->input_port_ = input_port; }

CTInputPort PowerSensor::get_input_port() const { return this->input_port_; }

void PowerSensor::record(const SensorReading &sensor_reading) {
  ReadingPowerEntry power_entry = sensor_reading.power[this->input_port_];
  int32_t raw_value = this->phase_->extract_reading_for_phase(power_entry);

  this->total_since_publish_ += raw_value;
  this->readings_since_publish_++;
  ESP_LOGV(TAG, "'%s': Recorded raw reading %d; total sum %lli, %d readings since publish, accumulated average is %.2f",
           this->get_name().c_str(), raw_value, this->total_since_publish_, this->readings_since_publish_,
           this->total_since_publish_ / (float) this->readings_since_publish_);
}

void PowerSensor::publish() {
  float raw_value_avg = this->total_since_publish_ / (float) this->readings_since_publish_;
  float calibrated_power = this->calibrate_value(raw_value_avg);
  this->publish_state(calibrated_power);

  this->total_since_publish_ = 0;
  this->readings_since_publish_ = 0;
}

float PowerSensor::calibrate_value(float raw_value) const {
  float phase_calibration = this->phase_->get_calibration();
  float ct_multiplier = 1 / 22.0;
  if (this->input_port_ == CTInputPort::A || this->input_port_ == CTInputPort::B ||
      this->input_port_ == CTInputPort::C) {
    ct_multiplier = -1 / 5.5;
  }

  float calibrated_value = raw_value * phase_calibration * ct_multiplier;

  ESP_LOGD(TAG, "'%s': Calculated calibrated %.2f W from raw %.2f, phase calibration %.2f, CT multiplier %.2f",
           this->get_name().c_str(), calibrated_value, raw_value, phase_calibration, ct_multiplier);

  return calibrated_value;
}

}  // namespace emporia_vue
}  // namespace esphome
