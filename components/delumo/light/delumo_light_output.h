#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/delumo/output/delumo_output.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace delumo {

class DelumoLightOutput : public light::LightOutput {
 public:
  void set_serial(uint16_t serial) { serial_ = serial; }
  void set_output(DelumoOutput *output) { output_ = output; }

  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::ON_OFF});
    return traits;
  }

  void write_state(light::LightState *state) override {
    bool binary;
    state->current_values_as_binary(&binary);
    if (binary) {
      this->output_->turn_on(this->serial_);
    } else {
      this->output_->turn_off(this->serial_);
    }
  }

 protected:
  uint16_t serial_;
  DelumoOutput *output_;
};

}  // namespace delumo
}  // namespace esphome
