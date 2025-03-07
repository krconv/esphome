#include "waveshare_epaper.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace waveshare_epaper {

static const char *const TAG = "waveshare_epaper";

static const uint8_t LUT_SIZE_WAVESHARE = 30;

static const uint8_t FULL_UPDATE_LUT[LUT_SIZE_WAVESHARE] = {0x02, 0x02, 0x01, 0x11, 0x12, 0x12, 0x22, 0x22, 0x66, 0x69,
                                                            0x69, 0x59, 0x58, 0x99, 0x99, 0x88, 0x00, 0x00, 0x00, 0x00,
                                                            0xF8, 0xB4, 0x13, 0x51, 0x35, 0x51, 0x51, 0x19, 0x01, 0x00};

static const uint8_t PARTIAL_UPDATE_LUT[LUT_SIZE_WAVESHARE] = {
    0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const uint8_t LUT_SIZE_TTGO = 70;

static const uint8_t FULL_UPDATE_LUT_TTGO[LUT_SIZE_TTGO] = {
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,  // LUT0: BB:     VS 0 ~7
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00,  // LUT1: BW:     VS 0 ~7
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,  // LUT2: WB:     VS 0 ~7
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00,  // LUT3: WW:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT4: VCOM:   VS 0 ~7
    0x03, 0x03, 0x00, 0x00, 0x02,              // TP0 A~D RP0
    0x09, 0x09, 0x00, 0x00, 0x02,              // TP1 A~D RP1
    0x03, 0x03, 0x00, 0x00, 0x02,              // TP2 A~D RP2
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP3 A~D RP3
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP4 A~D RP4
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP5 A~D RP5
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP6 A~D RP6
};

static const uint8_t PARTIAL_UPDATE_LUT_TTGO[LUT_SIZE_TTGO] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT0: BB:     VS 0 ~7
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT1: BW:     VS 0 ~7
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT2: WB:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT3: WW:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT4: VCOM:   VS 0 ~7
    0x0A, 0x00, 0x00, 0x00, 0x00,              // TP0 A~D RP0
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP1 A~D RP1
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP2 A~D RP2
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP3 A~D RP3
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP4 A~D RP4
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP5 A~D RP5
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP6 A~D RP6
};

static const uint8_t LUT_SIZE_TTGO_B73 = 100;

static const uint8_t FULL_UPDATE_LUT_TTGO_B73[LUT_SIZE_TTGO_B73] = {
    0xA0, 0x90, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x90, 0xA0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xA0, 0x90, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x90, 0xA0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    0x0F, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t PARTIAL_UPDATE_LUT_TTGO_B73[LUT_SIZE_TTGO_B73] = {
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t LUT_SIZE_TTGO_B1 = 29;

static const uint8_t FULL_UPDATE_LUT_TTGO_B1[LUT_SIZE_TTGO_B1] = {
    0x22, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x01, 0x00, 0x00, 0x00, 0x00};

static const uint8_t PARTIAL_UPDATE_LUT_TTGO_B1[LUT_SIZE_TTGO_B1] = {
    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void WaveshareEPaper::setup_pins_() {
  this->init_internal_(this->get_buffer_length_());
  this->dc_pin_->setup();  // OUTPUT
  this->dc_pin_->digital_write(false);
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();  // OUTPUT
    this->reset_pin_->digital_write(true);
  }
  if (this->busy_pin_ != nullptr) {
    this->busy_pin_->setup();  // INPUT
  }
  this->spi_setup();

  this->reset_();
}
float WaveshareEPaper::get_setup_priority() const { return setup_priority::PROCESSOR; }
void WaveshareEPaper::command(uint8_t value) {
  this->start_command_();
  this->write_byte(value);
  this->end_command_();
}
void WaveshareEPaper::data(uint8_t value) {
  this->start_data_();
  this->write_byte(value);
  this->end_data_();
}
bool WaveshareEPaper::wait_until_idle_() {
  if (this->busy_pin_ == nullptr) {
    return true;
  }

  const uint32_t start = millis();
  while (this->busy_pin_->digital_read()) {
    if (millis() - start > this->idle_timeout_()) {
      ESP_LOGE(TAG, "Timeout while displaying image!");
      return false;
    }
    delay(10);
  }
  return true;
}
void WaveshareEPaper::update() {
  this->do_update_();
  this->display();
}
void WaveshareEPaper::fill(Color color) {
  // flip logic
  const uint8_t fill = color.is_on() ? 0x00 : 0xFF;
  for (uint32_t i = 0; i < this->get_buffer_length_(); i++)
    this->buffer_[i] = fill;
}
void HOT WaveshareEPaper::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x >= this->get_width_internal() || y >= this->get_height_internal() || x < 0 || y < 0)
    return;

  const uint32_t pos = (x + y * this->get_width_internal()) / 8u;
  const uint8_t subpos = x & 0x07;
  // flip logic
  if (!color.is_on())
    this->buffer_[pos] |= 0x80 >> subpos;
  else
    this->buffer_[pos] &= ~(0x80 >> subpos);
}
uint32_t WaveshareEPaper::get_buffer_length_() { return this->get_width_internal() * this->get_height_internal() / 8u; }
void WaveshareEPaper::start_command_() {
  this->dc_pin_->digital_write(false);
  this->enable();
}
void WaveshareEPaper::end_command_() { this->disable(); }
void WaveshareEPaper::start_data_() {
  this->dc_pin_->digital_write(true);
  this->enable();
}
void WaveshareEPaper::end_data_() { this->disable(); }
void WaveshareEPaper::on_safe_shutdown() { this->deep_sleep(); }

// ========================================================
//                          Type A
// ========================================================

void WaveshareEPaperTypeA::initialize() {
  // COMMAND DRIVER OUTPUT CONTROL
  this->command(0x01);
  this->data(this->get_height_internal() - 1);
  this->data((this->get_height_internal() - 1) >> 8);
  this->data(0x00);  // ? GD = 0, SM = 0, TB = 0

  // COMMAND BOOSTER SOFT START CONTROL
  this->command(0x0C);
  this->data(0xD7);
  this->data(0xD6);
  this->data(0x9D);

  // COMMAND WRITE VCOM REGISTER
  this->command(0x2C);
  this->data(0xA8);

  // COMMAND SET DUMMY LINE PERIOD
  this->command(0x3A);
  this->data(0x1A);

  // COMMAND SET GATE TIME
  this->command(0x3B);
  this->data(0x08);  // 2µs per row

  // COMMAND DATA ENTRY MODE SETTING
  this->command(0x11);
  switch (this->model_) {
    case TTGO_EPAPER_2_13_IN_B1:
      this->data(0x01);  // x increase, y decrease : as in demo code
      break;
    case WAVESHARE_EPAPER_2_9_IN_V2:
      this->data(0x03);  // from top left to bottom right
      // RAM content option for Display Update
      this->command(0x21);
      this->data(0x00);
      this->data(0x80);
      break;
    default:
      this->data(0x03);  // from top left to bottom right
  }
}
void WaveshareEPaperTypeA::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this);
  switch (this->model_) {
    case WAVESHARE_EPAPER_1_54_IN:
      ESP_LOGCONFIG(TAG, "  Model: 1.54in");
      break;
    case WAVESHARE_EPAPER_1_54_IN_V2:
      ESP_LOGCONFIG(TAG, "  Model: 1.54inV2");
      break;
    case WAVESHARE_EPAPER_2_13_IN:
      ESP_LOGCONFIG(TAG, "  Model: 2.13in");
      break;
    case TTGO_EPAPER_2_13_IN:
      ESP_LOGCONFIG(TAG, "  Model: 2.13in (TTGO)");
      break;
    case TTGO_EPAPER_2_13_IN_B73:
      ESP_LOGCONFIG(TAG, "  Model: 2.13in (TTGO B73)");
      break;
    case TTGO_EPAPER_2_13_IN_B1:
      ESP_LOGCONFIG(TAG, "  Model: 2.13in (TTGO B1)");
      break;
    case WAVESHARE_EPAPER_2_9_IN:
      ESP_LOGCONFIG(TAG, "  Model: 2.9in");
      break;
    case WAVESHARE_EPAPER_2_9_IN_V2:
      ESP_LOGCONFIG(TAG, "  Model: 2.9inV2");
      break;
  }
  ESP_LOGCONFIG(TAG, "  Full Update Every: %u", this->full_update_every_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}
void HOT WaveshareEPaperTypeA::display() {
  bool full_update = this->at_update_ == 0;
  bool prev_full_update = this->at_update_ == 1;

  if (!this->wait_until_idle_()) {
    this->status_set_warning();
    return;
  }

  if (this->full_update_every_ >= 1) {
    if (full_update != prev_full_update) {
      switch (this->model_) {
        case TTGO_EPAPER_2_13_IN:
          this->write_lut_(full_update ? FULL_UPDATE_LUT_TTGO : PARTIAL_UPDATE_LUT_TTGO, LUT_SIZE_TTGO);
          break;
        case TTGO_EPAPER_2_13_IN_B73:
          this->write_lut_(full_update ? FULL_UPDATE_LUT_TTGO_B73 : PARTIAL_UPDATE_LUT_TTGO_B73, LUT_SIZE_TTGO_B73);
          break;
        case TTGO_EPAPER_2_13_IN_B1:
          this->write_lut_(full_update ? FULL_UPDATE_LUT_TTGO_B1 : PARTIAL_UPDATE_LUT_TTGO_B1, LUT_SIZE_TTGO_B1);
          break;
        default:
          this->write_lut_(full_update ? FULL_UPDATE_LUT : PARTIAL_UPDATE_LUT, LUT_SIZE_WAVESHARE);
      }
    }
    this->at_update_ = (this->at_update_ + 1) % this->full_update_every_;
  }

  // Set x & y regions we want to write to (full)
  switch (this->model_) {
    case TTGO_EPAPER_2_13_IN_B1:
      // COMMAND SET RAM X ADDRESS START END POSITION
      this->command(0x44);
      this->data(0x00);
      this->data((this->get_width_internal() - 1) >> 3);
      // COMMAND SET RAM Y ADDRESS START END POSITION
      this->command(0x45);
      this->data(this->get_height_internal() - 1);
      this->data((this->get_height_internal() - 1) >> 8);
      this->data(0x00);
      this->data(0x00);

      // COMMAND SET RAM X ADDRESS COUNTER
      this->command(0x4E);
      this->data(0x00);
      // COMMAND SET RAM Y ADDRESS COUNTER
      this->command(0x4F);
      this->data(this->get_height_internal() - 1);
      this->data((this->get_height_internal() - 1) >> 8);

      break;

    default:
      // COMMAND SET RAM X ADDRESS START END POSITION
      this->command(0x44);
      this->data(0x00);
      this->data((this->get_width_internal() - 1) >> 3);
      // COMMAND SET RAM Y ADDRESS START END POSITION
      this->command(0x45);
      this->data(0x00);
      this->data(0x00);
      this->data(this->get_height_internal() - 1);
      this->data((this->get_height_internal() - 1) >> 8);

      // COMMAND SET RAM X ADDRESS COUNTER
      this->command(0x4E);
      this->data(0x00);
      // COMMAND SET RAM Y ADDRESS COUNTER
      this->command(0x4F);
      this->data(0x00);
      this->data(0x00);
  }

  if (!this->wait_until_idle_()) {
    this->status_set_warning();
    return;
  }

  // COMMAND WRITE RAM
  this->command(0x24);
  this->start_data_();
  switch (this->model_) {
    case TTGO_EPAPER_2_13_IN_B1: {  // block needed because of variable initializations
      int16_t wb = ((this->get_width_internal()) >> 3);
      for (int i = 0; i < this->get_height_internal(); i++) {
        for (int j = 0; j < wb; j++) {
          int idx = j + (this->get_height_internal() - 1 - i) * wb;
          this->write_byte(this->buffer_[idx]);
        }
      }
      break;
    }
    default:
      this->write_array(this->buffer_, this->get_buffer_length_());
  }
  this->end_data_();

  // COMMAND DISPLAY UPDATE CONTROL 2
  this->command(0x22);
  if (this->model_ == WAVESHARE_EPAPER_2_9_IN_V2 || this->model_ == WAVESHARE_EPAPER_1_54_IN_V2) {
    this->data(full_update ? 0xF7 : 0xFF);
  } else if (this->model_ == TTGO_EPAPER_2_13_IN_B73) {
    this->data(0xC7);
  } else {
    this->data(0xC4);
  }

  // COMMAND MASTER ACTIVATION
  this->command(0x20);
  // COMMAND TERMINATE FRAME READ WRITE
  this->command(0xFF);

  this->status_clear_warning();
}
int WaveshareEPaperTypeA::get_width_internal() {
  switch (this->model_) {
    case WAVESHARE_EPAPER_1_54_IN:
      return 200;
    case WAVESHARE_EPAPER_1_54_IN_V2:
      return 200;
    case WAVESHARE_EPAPER_2_13_IN:
      return 128;
    case TTGO_EPAPER_2_13_IN:
      return 128;
    case TTGO_EPAPER_2_13_IN_B73:
      return 128;
    case TTGO_EPAPER_2_13_IN_B1:
      return 128;
    case WAVESHARE_EPAPER_2_9_IN:
      return 128;
    case WAVESHARE_EPAPER_2_9_IN_V2:
      return 128;
  }
  return 0;
}
int WaveshareEPaperTypeA::get_height_internal() {
  switch (this->model_) {
    case WAVESHARE_EPAPER_1_54_IN:
      return 200;
    case WAVESHARE_EPAPER_1_54_IN_V2:
      return 200;
    case WAVESHARE_EPAPER_2_13_IN:
      return 250;
    case TTGO_EPAPER_2_13_IN:
      return 250;
    case TTGO_EPAPER_2_13_IN_B73:
      return 250;
    case TTGO_EPAPER_2_13_IN_B1:
      return 250;
    case WAVESHARE_EPAPER_2_9_IN:
      return 296;
    case WAVESHARE_EPAPER_2_9_IN_V2:
      return 296;
  }
  return 0;
}
void WaveshareEPaperTypeA::write_lut_(const uint8_t *lut, const uint8_t size) {
  // COMMAND WRITE LUT REGISTER
  this->command(0x32);
  for (uint8_t i = 0; i < size; i++)
    this->data(lut[i]);
}
WaveshareEPaperTypeA::WaveshareEPaperTypeA(WaveshareEPaperTypeAModel model) : model_(model) {}
void WaveshareEPaperTypeA::set_full_update_every(uint32_t full_update_every) {
  this->full_update_every_ = full_update_every;
}

int WaveshareEPaperTypeA::idle_timeout_() {
  switch (this->model_) {
    case TTGO_EPAPER_2_13_IN_B1:
      return 2500;
      break;
    default:
      return WaveshareEPaper::idle_timeout_();
  }
}

// ========================================================
//                          Type B
// ========================================================
// Datasheet:
//  - https://www.waveshare.com/w/upload/7/7f/4.2inch-e-paper-b-specification.pdf
//  - https://github.com/soonuse/epd-library-arduino/blob/master/4.2inch_e-paper/epd4in2/

static const uint8_t LUT_VCOM_DC_2_7[44] = {
    0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x05, 0x00, 0x32, 0x32, 0x00, 0x00, 0x02, 0x00,
    0x0F, 0x0F, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t LUT_WHITE_TO_WHITE_2_7[42] = {
    0x50, 0x0F, 0x0F, 0x00, 0x00, 0x05, 0x60, 0x32, 0x32, 0x00, 0x00, 0x02, 0xA0, 0x0F,
    0x0F, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t LUT_BLACK_TO_WHITE_2_7[42] = {
    0x50, 0x0F, 0x0F, 0x00, 0x00, 0x05, 0x60, 0x32, 0x32, 0x00, 0x00, 0x02, 0xA0, 0x0F,
    0x0F, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t LUT_WHITE_TO_BLACK_2_7[] = {
    0xA0, 0x0F, 0x0F, 0x00, 0x00, 0x05, 0x60, 0x32, 0x32, 0x00, 0x00, 0x02, 0x50, 0x0F,
    0x0F, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t LUT_BLACK_TO_BLACK_2_7[42] = {
    0xA0, 0x0F, 0x0F, 0x00, 0x00, 0x05, 0x60, 0x32, 0x32, 0x00, 0x00, 0x02, 0x50, 0x0F,
    0x0F, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

void WaveshareEPaper2P7In::initialize() {
  // command power setting
  this->command(0x01);
  this->data(0x03);  // VDS_EN, VDG_EN
  this->data(0x00);  // VCOM_HV, VGHL_LV[1], VGHL_LV[0]
  this->data(0x2B);  // VDH
  this->data(0x2B);  // VDL
  this->data(0x09);  // VDHR

  // command booster soft start
  this->command(0x06);
  this->data(0x07);
  this->data(0x07);
  this->data(0x17);

  // Power optimization - ???
  this->command(0xF8);
  this->data(0x60);
  this->data(0xA5);
  this->command(0xF8);
  this->data(0x89);
  this->data(0xA5);
  this->command(0xF8);
  this->data(0x90);
  this->data(0x00);
  this->command(0xF8);
  this->data(0x93);
  this->data(0x2A);
  this->command(0xF8);
  this->data(0xA0);
  this->data(0xA5);
  this->command(0xF8);
  this->data(0xA1);
  this->data(0x00);
  this->command(0xF8);
  this->data(0x73);
  this->data(0x41);

  // command partial display refresh
  this->command(0x16);
  this->data(0x00);

  // command power on
  this->command(0x04);
  this->wait_until_idle_();
  delay(10);

  // Command panel setting
  this->command(0x00);
  this->data(0xAF);  // KW-BF   KWR-AF    BWROTP 0f
  // command pll control
  this->command(0x30);
  this->data(0x3A);  // 3A 100HZ   29 150Hz 39 200HZ    31 171HZ
  // COMMAND VCM DC SETTING
  this->command(0x82);
  this->data(0x12);

  delay(2);
  // COMMAND LUT FOR VCOM
  this->command(0x20);
  for (uint8_t i : LUT_VCOM_DC_2_7)
    this->data(i);

  // COMMAND LUT WHITE TO WHITE
  this->command(0x21);
  for (uint8_t i : LUT_WHITE_TO_WHITE_2_7)
    this->data(i);
  // COMMAND LUT BLACK TO WHITE
  this->command(0x22);
  for (uint8_t i : LUT_BLACK_TO_WHITE_2_7)
    this->data(i);
  // COMMAND LUT WHITE TO BLACK
  this->command(0x23);
  for (uint8_t i : LUT_WHITE_TO_BLACK_2_7)
    this->data(i);
  // COMMAND LUT BLACK TO BLACK
  this->command(0x24);
  for (uint8_t i : LUT_BLACK_TO_BLACK_2_7)
    this->data(i);
}
void HOT WaveshareEPaper2P7In::display() {
  uint32_t buf_len = this->get_buffer_length_();

  // COMMAND DATA START TRANSMISSION 1
  this->command(0x10);
  delay(2);
  for (uint32_t i = 0; i < buf_len; i++) {
    this->data(this->buffer_[i]);
  }
  delay(2);

  // COMMAND DATA START TRANSMISSION 2
  this->command(0x13);
  delay(2);
  for (uint32_t i = 0; i < buf_len; i++) {
    this->data(this->buffer_[i]);
  }

  // COMMAND DISPLAY REFRESH
  this->command(0x12);
}
int WaveshareEPaper2P7In::get_width_internal() { return 176; }
int WaveshareEPaper2P7In::get_height_internal() { return 264; }
void WaveshareEPaper2P7In::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 2.7in");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}

// ========================================================
//               2.90in Type B (LUT from OTP)
// Datasheet:
//  - https://www.waveshare.com/w/upload/b/bb/2.9inch-e-paper-b-specification.pdf
//  - https://github.com/soonuse/epd-library-arduino/blob/master/2.9inch_e-paper_b/epd2in9b/epd2in9b.cpp
// ========================================================

void WaveshareEPaper2P9InB::initialize() {
  // from https://www.waveshare.com/w/upload/b/bb/2.9inch-e-paper-b-specification.pdf, page 37
  // EPD hardware init start
  this->reset_();

  // COMMAND BOOSTER SOFT START
  this->command(0x06);
  this->data(0x17);
  this->data(0x17);
  this->data(0x17);

  // COMMAND POWER ON
  this->command(0x04);
  this->wait_until_idle_();

  // COMMAND PANEL SETTING
  this->command(0x00);
  // 128x296 resolution:        10
  // LUT from OTP:              0
  // B/W mode (doesn't work):   1
  // scan-up:                   1
  // shift-right:               1
  // booster ON:                1
  // no soft reset:             1
  this->data(0x9F);

  // COMMAND RESOLUTION SETTING
  // set to 128x296 by COMMAND PANEL SETTING

  // COMMAND VCOM AND DATA INTERVAL SETTING
  // use defaults for white border and ESPHome image polarity

  // EPD hardware init end
}
void HOT WaveshareEPaper2P9InB::display() {
  // COMMAND DATA START TRANSMISSION 1 (B/W data)
  this->command(0x10);
  delay(2);
  this->start_data_();
  this->write_array(this->buffer_, this->get_buffer_length_());
  this->end_data_();
  delay(2);

  // COMMAND DATA START TRANSMISSION 2 (RED data)
  this->command(0x13);
  delay(2);
  this->start_data_();
  for (int i = 0; i < this->get_buffer_length_(); i++)
    this->write_byte(0x00);
  this->end_data_();
  delay(2);

  // COMMAND DISPLAY REFRESH
  this->command(0x12);
  delay(2);
  this->wait_until_idle_();

  // COMMAND POWER OFF
  // NOTE: power off < deep sleep
  this->command(0x02);
}
int WaveshareEPaper2P9InB::get_width_internal() { return 128; }
int WaveshareEPaper2P9InB::get_height_internal() { return 296; }
void WaveshareEPaper2P9InB::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 2.9in (B)");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}

static const uint8_t LUT_VCOM_DC_4_2[] = {
    0x00, 0x17, 0x00, 0x00, 0x00, 0x02, 0x00, 0x17, 0x17, 0x00, 0x00, 0x02, 0x00, 0x0A, 0x01,
    0x00, 0x00, 0x01, 0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t LUT_WHITE_TO_WHITE_4_2[] = {
    0x40, 0x17, 0x00, 0x00, 0x00, 0x02, 0x90, 0x17, 0x17, 0x00, 0x00, 0x02, 0x40, 0x0A,
    0x01, 0x00, 0x00, 0x01, 0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t LUT_BLACK_TO_WHITE_4_2[] = {
    0x40, 0x17, 0x00, 0x00, 0x00, 0x02, 0x90, 0x17, 0x17, 0x00, 0x00, 0x02, 0x40, 0x0A,
    0x01, 0x00, 0x00, 0x01, 0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t LUT_BLACK_TO_BLACK_4_2[] = {
    0x80, 0x17, 0x00, 0x00, 0x00, 0x02, 0x90, 0x17, 0x17, 0x00, 0x00, 0x02, 0x80, 0x0A,
    0x01, 0x00, 0x00, 0x01, 0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t LUT_WHITE_TO_BLACK_4_2[] = {
    0x80, 0x17, 0x00, 0x00, 0x00, 0x02, 0x90, 0x17, 0x17, 0x00, 0x00, 0x02, 0x80, 0x0A,
    0x01, 0x00, 0x00, 0x01, 0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

void WaveshareEPaper4P2In::initialize() {
  // https://www.waveshare.com/w/upload/7/7f/4.2inch-e-paper-b-specification.pdf - page 8

  // COMMAND POWER SETTING
  this->command(0x01);
  this->data(0x03);  // VDS_EN, VDG_EN
  this->data(0x00);  // VCOM_HV, VGHL_LV[1], VGHL_LV[0]
  this->data(0x2B);  // VDH
  this->data(0x2B);  // VDL
  this->data(0xFF);  // VDHR

  // COMMAND BOOSTER SOFT START
  this->command(0x06);
  this->data(0x17);  // PHA
  this->data(0x17);  // PHB
  this->data(0x17);  // PHC

  // COMMAND POWER ON
  this->command(0x04);
  this->wait_until_idle_();
  delay(10);
  // COMMAND PANEL SETTING
  this->command(0x00);
  this->data(0xBF);  // KW-BF   KWR-AF  BWROTP 0f
  this->data(0x0B);
  // COMMAND PLL CONTROL
  this->command(0x30);
  this->data(0x3C);  // 3A 100HZ   29 150Hz 39 200HZ  31 171HZ

  delay(2);
  // COMMAND LUT FOR VCOM
  this->command(0x20);
  for (uint8_t i : LUT_VCOM_DC_4_2)
    this->data(i);
  // COMMAND LUT WHITE TO WHITE
  this->command(0x21);
  for (uint8_t i : LUT_WHITE_TO_WHITE_4_2)
    this->data(i);
  // COMMAND LUT BLACK TO WHITE
  this->command(0x22);
  for (uint8_t i : LUT_BLACK_TO_WHITE_4_2)
    this->data(i);
  // COMMAND LUT WHITE TO BLACK
  this->command(0x23);
  for (uint8_t i : LUT_WHITE_TO_BLACK_4_2)
    this->data(i);
  // COMMAND LUT BLACK TO BLACK
  this->command(0x24);
  for (uint8_t i : LUT_BLACK_TO_BLACK_4_2)
    this->data(i);
}
void HOT WaveshareEPaper4P2In::display() {
  // COMMAND RESOLUTION SETTING
  this->command(0x61);
  this->data(0x01);
  this->data(0x90);
  this->data(0x01);
  this->data(0x2C);

  // COMMAND VCM DC SETTING REGISTER
  this->command(0x82);
  this->data(0x12);

  // COMMAND VCOM AND DATA INTERVAL SETTING
  this->command(0x50);
  this->data(0x97);

  // COMMAND DATA START TRANSMISSION 1
  this->command(0x10);
  delay(2);
  this->start_data_();
  this->write_array(this->buffer_, this->get_buffer_length_());
  this->end_data_();
  delay(2);
  // COMMAND DATA START TRANSMISSION 2
  this->command(0x13);
  delay(2);
  this->start_data_();
  this->write_array(this->buffer_, this->get_buffer_length_());
  this->end_data_();
  // COMMAND DISPLAY REFRESH
  this->command(0x12);
}
int WaveshareEPaper4P2In::get_width_internal() { return 400; }
int WaveshareEPaper4P2In::get_height_internal() { return 300; }
void WaveshareEPaper4P2In::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 4.2in");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}

void WaveshareEPaper5P8In::initialize() {
  // COMMAND POWER SETTING
  this->command(0x01);
  this->data(0x37);
  this->data(0x00);

  // COMMAND PANEL SETTING
  this->command(0x00);
  this->data(0xCF);
  this->data(0x0B);

  // COMMAND BOOSTER SOFT START
  this->command(0x06);
  this->data(0xC7);
  this->data(0xCC);
  this->data(0x28);

  // COMMAND POWER ON
  this->command(0x04);
  this->wait_until_idle_();
  delay(10);

  // COMMAND PLL CONTROL
  this->command(0x30);
  this->data(0x3C);

  // COMMAND TEMPERATURE SENSOR CALIBRATION
  this->command(0x41);
  this->data(0x00);

  // COMMAND VCOM AND DATA INTERVAL SETTING
  this->command(0x50);
  this->data(0x77);

  // COMMAND TCON SETTING
  this->command(0x60);
  this->data(0x22);

  // COMMAND RESOLUTION SETTING
  this->command(0x61);
  this->data(0x02);
  this->data(0x58);
  this->data(0x01);
  this->data(0xC0);

  // COMMAND VCM DC SETTING REGISTER
  this->command(0x82);
  this->data(0x1E);

  this->command(0xE5);
  this->data(0x03);
}
void HOT WaveshareEPaper5P8In::display() {
  // COMMAND DATA START TRANSMISSION 1
  this->command(0x10);

  this->start_data_();
  for (size_t i = 0; i < this->get_buffer_length_(); i++) {
    uint8_t temp1 = this->buffer_[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t temp2;
      if (temp1 & 0x80)
        temp2 = 0x03;
      else
        temp2 = 0x00;

      temp2 <<= 4;
      temp1 <<= 1;
      j++;
      if (temp1 & 0x80)
        temp2 |= 0x03;
      else
        temp2 |= 0x00;
      temp1 <<= 1;
      this->write_byte(temp2);
    }

    App.feed_wdt();
  }
  this->end_data_();

  // COMMAND DISPLAY REFRESH
  this->command(0x12);
}
int WaveshareEPaper5P8In::get_width_internal() { return 600; }
int WaveshareEPaper5P8In::get_height_internal() { return 448; }
void WaveshareEPaper5P8In::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 5.83in");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}
void WaveshareEPaper7P5In::initialize() {
  // COMMAND POWER SETTING
  this->command(0x01);
  this->data(0x37);
  this->data(0x00);
  // COMMAND PANEL SETTING
  this->command(0x00);
  this->data(0xCF);
  this->data(0x0B);
  // COMMAND BOOSTER SOFT START
  this->command(0x06);
  this->data(0xC7);
  this->data(0xCC);
  this->data(0x28);
  // COMMAND POWER ON
  this->command(0x04);
  this->wait_until_idle_();
  delay(10);
  // COMMAND PLL CONTROL
  this->command(0x30);
  this->data(0x3C);
  // COMMAND TEMPERATURE SENSOR CALIBRATION
  this->command(0x41);
  this->data(0x00);
  // COMMAND VCOM AND DATA INTERVAL SETTING
  this->command(0x50);
  this->data(0x77);
  // COMMAND TCON SETTING
  this->command(0x60);
  this->data(0x22);
  // COMMAND RESOLUTION SETTING
  this->command(0x61);
  this->data(0x02);
  this->data(0x80);
  this->data(0x01);
  this->data(0x80);
  // COMMAND VCM DC SETTING REGISTER
  this->command(0x82);
  this->data(0x1E);
  this->command(0xE5);
  this->data(0x03);
}
void HOT WaveshareEPaper7P5In::display() {
  // COMMAND DATA START TRANSMISSION 1
  this->command(0x10);
  this->start_data_();
  for (size_t i = 0; i < this->get_buffer_length_(); i++) {
    uint8_t temp1 = this->buffer_[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t temp2;
      if (temp1 & 0x80)
        temp2 = 0x03;
      else
        temp2 = 0x00;
      temp2 <<= 4;
      temp1 <<= 1;
      j++;
      if (temp1 & 0x80)
        temp2 |= 0x03;
      else
        temp2 |= 0x00;
      temp1 <<= 1;
      this->write_byte(temp2);
    }
    App.feed_wdt();
  }
  this->end_data_();
  // COMMAND DISPLAY REFRESH
  this->command(0x12);
}
int WaveshareEPaper7P5In::get_width_internal() { return 640; }
int WaveshareEPaper7P5In::get_height_internal() { return 384; }
void WaveshareEPaper7P5In::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 7.5in");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}
void WaveshareEPaper7P5InV2::initialize() {
  // COMMAND POWER SETTING
  this->command(0x01);
  this->data(0x07);
  this->data(0x07);
  this->data(0x3f);
  this->data(0x3f);
  this->command(0x04);

  delay(100);  // NOLINT
  this->wait_until_idle_();
  // COMMAND PANEL SETTING
  this->command(0x00);
  this->data(0x1F);

  // COMMAND RESOLUTION SETTING
  this->command(0x61);
  this->data(0x03);
  this->data(0x20);
  this->data(0x01);
  this->data(0xE0);
  // COMMAND ...?
  this->command(0x15);
  this->data(0x00);
  // COMMAND VCOM AND DATA INTERVAL SETTING
  this->command(0x50);
  this->data(0x10);
  this->data(0x07);
  // COMMAND TCON SETTING
  this->command(0x60);
  this->data(0x22);
}
void HOT WaveshareEPaper7P5InV2::display() {
  uint32_t buf_len = this->get_buffer_length_();
  // COMMAND DATA START TRANSMISSION NEW DATA
  this->command(0x13);
  delay(2);
  for (uint32_t i = 0; i < buf_len; i++) {
    this->data(~(this->buffer_[i]));
  }

  // COMMAND DISPLAY REFRESH
  this->command(0x12);
  delay(100);  // NOLINT
  this->wait_until_idle_();
}

int WaveshareEPaper7P5InV2::get_width_internal() { return 800; }
int WaveshareEPaper7P5InV2::get_height_internal() { return 480; }
void WaveshareEPaper7P5InV2::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 7.5inV2");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}
}  // namespace waveshare_epaper
}  // namespace esphome
