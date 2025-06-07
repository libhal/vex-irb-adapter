// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <optional>

#include <libhal/adc.hpp>
#include <libhal/functional.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

struct resource_list
{
  /// Initialized 1st
  hal::callback<void()> reset = +[] {
    while (true) {
      continue;
    }
  };
  std::optional<hal::serial*> console;
  std::optional<hal::steady_clock*> clock;
  std::optional<hal::output_pin*> counter_reset;
  std::optional<hal::output_pin*> counter_clock;
  std::optional<hal::adc*> intensity;
  std::optional<hal::output_pin*> transceiver_direction;
  std::optional<hal::output_pin*> frequency_select;
  [[maybe_unused]] std::optional<hal::serial*> rs485_transceiver;
};

/**
 * @brief Initializes the target platform and the resource list
 *
 * @param p_list - the list of resources to initialize for the application to
 * run properly. The initialize platform library should initialize each resource
 * in the resoure list.
 */
void initialize_platform(resource_list& p_list);
