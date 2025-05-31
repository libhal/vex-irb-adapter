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

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include <resource_list.hpp>

// This is only global so that the terminate handler can use the resources
// provided.
resource_list hardware_map{};

void application();

int main()
{
  using namespace std::literals;
  using namespace hal::literals;

  // Initialize the platform and set as many resources as available for this the
  // supported platforms.
  initialize_platform(hardware_map);

  try {
    application();
  } catch (std::bad_optional_access const& e) {
    if (hardware_map.console) {
      hal::print(*hardware_map.console.value(),
                 "A resource required by the application was not available!\n"
                 "Calling terminate!\n");
    }
  }  // Allow any other exceptions to terminate the application

  // Terminate if the code reaches this point.
  std::terminate();
}

void application()
{
  using namespace std::chrono_literals;

  auto& clock = *hardware_map.clock.value();
  auto& console = *hardware_map.console.value();
  auto& counter_reset = *hardware_map.counter_reset.value();
  auto& counter_clock = *hardware_map.counter_clock.value();
  [[maybe_unused]] auto& frequency_select =
    *hardware_map.frequency_select.value();
  [[maybe_unused]] auto& transceiver_direction =
    *hardware_map.transceiver_direction.value();
  auto& intensity = *hardware_map.intensity.value();
  [[maybe_unused]] auto& rs485_transceiver =
    *hardware_map.rs485_transceiver.value();

  while (true) {
    size_t led_count = 0;
    counter_clock.level(true);
    counter_reset.level(true);
    counter_reset.level(false);
    counter_clock.level(false);
    hal::delay(clock, 3ms);
    hal::print<32>(console, "%X : %X", led_count, intensity.read());
    hal::delay(clock, 2s);
    while (led_count < 8) {
      counter_clock.level(true);
      led_count++;
      counter_clock.level(false);
      hal::delay(clock, 3ms);
      hal::print<32>(console, "%X : %X", led_count, intensity.read());
      hal::delay(clock, 2s);
    }
  }
}
