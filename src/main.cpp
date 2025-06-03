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
std::array<float, 8> get_read_values();
void print_all(std::array<float, 8> read_values);

int main()
{
  using namespace std::literals;
  using namespace hal::literals;

  // Initialize the platform and set as many resources as available for this the
  // supported platforms.
  initialize_platform(hardware_map);
  hal::print<32>(*hardware_map.console.value(), "Starting application...");
  application();
}

void application()
{
  using namespace std::chrono_literals;
  // auto& console = *hardware_map.console.value();
  auto& transceiver_direction = *hardware_map.transceiver_direction.value();
  [[maybe_unused]] auto& frequency_select =
    *hardware_map.frequency_select.value();
  [[maybe_unused]] auto& rs485_transceiver =
    *hardware_map.rs485_transceiver.value();

  while (true) {
    // transceiver_direction low sets transceiver to receive mode
    transceiver_direction.level(false);
    // wait for request
    // static auto request = hal::read<1>(console, hal::never_timeout());
    // hal::print<32>(console, "%X\n", request);
    // get read values
    static std::array<float, 8> read_values = get_read_values();
    // return data based on request

    print_all(read_values);
    hal::delay(*hardware_map.clock.value(), 1s);
  }
}

std::array<float, 8> get_read_values()
{
  using namespace std::chrono_literals;

  auto& clock = *hardware_map.clock.value();
  auto& counter_reset = *hardware_map.counter_reset.value();
  auto& counter_clock = *hardware_map.counter_clock.value();
  auto& intensity = *hardware_map.intensity.value();
  static std::array<float, 8> read_values;

  size_t led_count = 0;
  counter_clock.level(true);
  counter_reset.level(true);
  counter_reset.level(false);
  counter_clock.level(false);
  hal::delay(clock, 3ms);
  read_values[led_count] = intensity.read();
  while (led_count < 8) {
    hal::delay(clock, 2ms);
    counter_clock.level(true);
    led_count++;
    counter_clock.level(false);
    hal::delay(clock, 3ms);
    read_values[led_count] = intensity.read();
  }
  return read_values;
}

void print_all(std::array<float, 8> read_values)
{
  auto& console = *hardware_map.console.value();
  for (uint8_t i = 0; i < 8; i++) {
    hal::print<32>(console, "LED %u: ", i);
    hal::print<32>(console, "%f \n", read_values[i]);
  }
}
