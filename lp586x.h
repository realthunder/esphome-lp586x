#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/light/addressable_light.h"
#include "esphome/components/light/light_output.h"
#include "esphome/core/color.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <driver/gpio.h>
#include <esp_err.h>

namespace esphome {
namespace lp586x {

enum RGBOrder : uint8_t {
  ORDER_RGB,
  ORDER_RBG,
  ORDER_GRB,
  ORDER_GBR,
  ORDER_BGR,
  ORDER_BRG,
};

class LP586XLightOutput : public light::AddressableLight, public i2c::I2CDevice {
 public:
  void setup() override;
  void write_state(light::LightState *state) override;
  float get_setup_priority() const override;

  int32_t size() const override { return this->num_leds_; }
  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::RGB});
    return traits;
  }

  void set_num_leds(uint16_t num_leds) { this->num_leds_ = num_leds; }

  void set_red_current(uint8_t value) {
      set_current(0, value);
  }

  void set_green_current(uint8_t value) {
      set_current(1, value);
  }

  void set_blue_current(uint8_t value) {
      set_current(2, value);
  }

  /// Set a maximum refresh rate in µs as some lights do not like being updated too often.
  void set_max_refresh_rate(uint32_t interval_us) { this->max_refresh_rate_ = interval_us; }

  void set_rgb_order(RGBOrder rgb_order) { this->rgb_order_ = rgb_order; }

  void set_sync_pin(GPIOPin *sync_pin) { sync_pin_ = sync_pin; }
  void set_vio_en_pin(GPIOPin *vio_en_pin) { vio_en_pin_ = vio_en_pin; }

  void clear_effect_data() override {
    for (int i = 0; i < this->size(); i++)
      this->effect_data_[i] = 0;
  }

  void dump_config() override;

 protected:

  void set_current(int idx, uint8_t value) {
    this->current_limits_[idx] = value;
    this->current_limit_changed_ = true;
    if (this->state_parent_)
      this->schedule_show();
  }

  bool write_data(uint16_t a_register, const uint8_t *data, uint8_t len, bool stop = true) {
    uint8_t extra = (uint8_t) ((a_register & 0x300) >> 8);
    this->address_ |= extra;
    bool res = this->write_register((uint8_t)(a_register & 0xff), data, len, stop) == i2c::ERROR_OK;
    this->address_ ^= extra;
    return res;
  }

  bool write_data(uint16_t a_register, uint8_t data, bool stop = true) {
    return write_data(a_register, &data, 1, stop);
  }

  void DC_Color(uint8_t Dot,uint8_t R_DC,uint8_t G_DC, uint8_t B_DC);

  void Dot_Brightness_8bit(uint8_t Dot, uint8_t R_PWM, uint8_t G_PWM, uint8_t B_PWM);

  light::ESPColorView get_view_internal(int32_t index) const override;

  size_t get_buffer_size_() const { return this->num_leds_ * 3; }

  uint8_t *buf_{nullptr};
  uint8_t *effect_data_{nullptr};

  uint16_t num_leds_;

  RGBOrder rgb_order_;

  uint32_t last_refresh_{0};
  optional<uint32_t> max_refresh_rate_{};

  uint8_t current_limits_[3];
  bool current_limit_changed_{false};
  mutable bool current_changed_{false};

  GPIOPin *sync_pin_{nullptr};
  GPIOPin *vio_en_pin_{nullptr};
};

}  // namespace lp586x
}  // namespace esphome

// vim: noai:ts=2:sw=2
