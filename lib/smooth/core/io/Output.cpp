/*
Smooth - A C++ framework for embedded programming on top of Espressif's ESP-IDF
Copyright 2019 Per Malmberg (https://gitbub.com/PerMalmberg)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "smooth/core/io/Output.h"

namespace smooth::core::io {
Output::Output(gpio_num_t io, bool active_high, bool pull_up, bool pull_down,
               bool clear_on_creation)
    : io(io), active_high(active_high), ledc_channel() {
  gpio_config_t config;

  config.pin_bit_mask = 1;
  config.pin_bit_mask <<= io;

  config.mode = active_high ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT_OUTPUT_OD;
  config.pull_down_en =
      pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  config.pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;

  ESP_ERROR_CHECK(gpio_config(&config));

  if (clear_on_creation) {
    clr();
  }
}

void Output::set() { gpio_set_level(io, active_high ? 1 : 0); }

void Output::set(bool active) {
  if (active) {
    set();
  } else {
    clr();
  }
}

void Output::clr() { gpio_set_level(io, active_high ? 0 : 1); }

void Output::setupPwm(ledc_channel_t f_channel, ledc_timer_bit_t f_resolution,
                      uint32_t f_frequency) {
    ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_HIGH_SPEED_MODE,  // timer mode
      .duty_resolution = f_resolution,     // resolution of PWM duty
      .timer_num = LEDC_TIMER_0,           // timer index
      .freq_hz = f_frequency,              // frequency of PWM signal
      .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
  };

  // Set configuration of timer0 for high speed channels
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel = {.gpio_num = io,
                  .speed_mode = LEDC_HIGH_SPEED_MODE,
                  .channel = f_channel,
                  .intr_type = LEDC_INTR_DISABLE,
                  .timer_sel = LEDC_TIMER_0,
                  .duty = 0,
                  .hpoint = 0};

  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void Output::setDutyCycle(uint32_t f_pwmDuty) {
  ESP_ERROR_CHECK(
      ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, f_pwmDuty));
  ESP_ERROR_CHECK(
      ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel));
}

bool Output::operator==(const Output& other) {
  return io == other.io && active_high == other.active_high;
}

bool Output::operator!=(const Output& other) { return !(*this == other); }
}  // namespace smooth::core::io
