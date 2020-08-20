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

#pragma once

#include <driver/gpio.h>
#include <driver/include/driver/ledc.h>
#include <stdio.h>

#include "driver/ledc.h"

namespace smooth::core::io {
class Output {
 public:
  Output(gpio_num_t io, bool active_high, bool pull_up, bool pull_down,
         bool clear_on_creation = true);

  void set();

  void set(bool active);

  void clr();

  void setupPwm(ledc_channel_t f_channel, ledc_timer_bit_t f_resolution,
                uint32_t f_frequency);

  void setDutyCycle(uint32_t f_pwmDuty);

  Output(const Output&) = delete;

  Output& operator=(const Output&) = delete;

  bool operator==(const Output& other);

  bool operator!=(const Output& other);

 private:
  gpio_num_t io;
  bool active_high;

  ledc_channel_config_t ledc_channel;
};
}  // namespace smooth::core::io
