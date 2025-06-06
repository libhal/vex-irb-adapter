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

#include <algorithm>
#include <array>
#include <cstddef>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include <libhal/units.hpp>
#include <resource_list.hpp>

// This is only global so that the terminate handler can use the resources
// provided.
resource_list hardware_map{};

[[noreturn]] void terminate_handler() noexcept
{
  bool valid = hardware_map.status_led && hardware_map.clock;

  if (not valid) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern, and wait for the debugger.
  // In GDB, use the `where` command to see if you have the `terminate_handler`
  // in your stack trace.

  auto& led = *hardware_map.status_led.value();
  auto& clock = *hardware_map.clock.value();

  while (true) {
    using namespace std::chrono_literals;
    led.level(false);
    hal::delay(clock, 100ms);
    led.level(true);
    hal::delay(clock, 100ms);
    led.level(false);
    hal::delay(clock, 100ms);
    led.level(true);
    hal::delay(clock, 1000ms);
  }
}

void application();
std::array<float, 8> read_led_values();
void print_all();
void strongest_signal();

int main()
{
  using namespace std::literals;
  using namespace hal::literals;

  // Setup the terminate handler before we call anything that can throw
  hal::set_terminate(terminate_handler);

  // Initialize the platform and set as many resources as available for this the
  // supported platforms.
  initialize_platform(hardware_map);

  hal::print<32>(*hardware_map.console.value(), "Starting application...");
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
  auto& console = *hardware_map.console.value();
  auto& transceiver_direction = *hardware_map.transceiver_direction.value();
  [[maybe_unused]] auto& frequency_select =
    *hardware_map.frequency_select.value();
  [[maybe_unused]] auto& rs485_transceiver =
    *hardware_map.rs485_transceiver.value();

  while (true) {
    // set transceiver to receive mode
    transceiver_direction.level(false);

    // wait for request
    std::array<hal::byte, 2> request{};
    request = hal::read<2>(console, hal::never_timeout());
    auto request_byte = request[0];

    // set transceiver to send mode
    transceiver_direction.level(true);
    // return data based on request
    switch (request_byte) {
      case 0x61:
        print_all();
        break;
      case 0x62:
        strongest_signal();
        break;
      case 0x63:  // set frequency high
        frequency_select.level(true);
        hal::print<32>(console, "Set frequency to 10kHz\n");
        break;
      case 0x64:  // set frequency low
        frequency_select.level(false);
        hal::print<32>(console, "Set frequency to 1kHz\n");
        break;
      default:
        hal::print<32>(console, "Default action...\n");
    }
  }
}

std::array<float, 8> read_led_values()
{
  using namespace std::chrono_literals;
  std::array<float, 8> read_values;
  auto& clock = *hardware_map.clock.value();
  auto& counter_reset = *hardware_map.counter_reset.value();
  auto& counter_clock = *hardware_map.counter_clock.value();
  auto& intensity = *hardware_map.intensity.value();
  size_t led_count = 0;
  counter_clock.level(true);
  counter_reset.level(true);
  counter_reset.level(false);
  hal::delay(clock, 5ms);
  counter_clock.level(false);
  hal::delay(clock, 3ms);
  read_values[led_count] = intensity.read();
  while (led_count < 7) {
    hal::delay(clock, 2ms);
    counter_clock.level(true);
    led_count++;
    hal::delay(clock, 5ms);
    counter_clock.level(false);
    hal::delay(clock, 3ms);
    read_values[led_count] = intensity.read();
  }
  return read_values;
}

void print_all()
{
  auto& console = *hardware_map.console.value();
  auto read_values = read_led_values();
  for (size_t i = 0; i < 8; i++) {
    hal::print<32>(console, "LED %u: ", i);
    hal::print<32>(console, "%f\n", read_values[i]);
  }
}

void strongest_signal()
{
  auto& console = *hardware_map.console.value();
  auto read_values = read_led_values();
  auto max = std::max_element(read_values.begin(), read_values.end());
  auto position = std::distance(read_values.begin(), max);
  hal::print<32>(console, "MAX VALUE LED %u: ", position);
  hal::print<32>(console, "%f\n", *max);
}
