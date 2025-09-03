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
#include <cstdint>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include <libhal/i2c.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

void application();
std::array<float, 8> read_led_values();
std::array<hal::byte, 2> get_strongest_signal(bool high_frequency);
bool camera_init();
std::array<hal::byte, 8> get_camera_data();

hal::v5::optional_ptr<hal::serial> console;
hal::v5::optional_ptr<hal::serial> rs485_transceiver;
hal::v5::optional_ptr<hal::steady_clock> device_clock;
hal::v5::optional_ptr<hal::output_pin> transceiver_direction;
hal::v5::optional_ptr<hal::output_pin> frequency_select;
hal::v5::optional_ptr<hal::output_pin> counter_reset;
hal::v5::optional_ptr<hal::output_pin> accumulator_reset;
hal::v5::optional_ptr<hal::adc> intensity;
hal::v5::optional_ptr<hal::i2c> i2c;

int main()
{
  using namespace std::literals;
  using namespace hal::literals;
  console = resources::console();
  rs485_transceiver = resources::rs485_transceiver();
  device_clock = resources::clock();
  transceiver_direction = resources::transceiver_direction();
  frequency_select = resources::frequency_select();
  counter_reset = resources::counter_reset();
  accumulator_reset = resources::accumulator_reset();
  intensity = resources::intensity();

  hal::print<32>(*console, "Starting application...");
  application();
  // while (true) {
  //   try {
  //   } catch (std::bad_optional_access const& e) {
  //     hal::print<32>(
  //       *console,
  //       "A resource required by the application was not available!\n"
  //       "Calling terminate!\n");
  //   }  // Allow any other exceptions to terminate the application
  // }
  // // Terminate if the code reaches this point.
  // std::terminate();
}

void application()
{
  using namespace std::chrono_literals;

  transceiver_direction->level(true);
  bool high_frequency = false;
  bool camera_connected = false;
  try {
    i2c = resources::i2c();
    camera_connected = camera_init();
  } catch (...) {
    hal::print<32>(*console, "Camera not connected...\n");
  }

  while (true) {
    std::array<hal::byte, 13> return_bytes;
    hal::byte checksum = 0x00;
    // get low frequency strongest signal
    frequency_select->level(high_frequency);
    auto lf_results = get_strongest_signal(high_frequency);
    // get high frequency strongest signal
    high_frequency = !high_frequency;
    frequency_select->level(high_frequency);
    auto hf_results = get_strongest_signal(high_frequency);
    // reset frequency
    high_frequency = !high_frequency;
    frequency_select->level(high_frequency);
    // get camera data
    std::array<hal::byte, 8> cam_data;
    if (camera_connected) {
      try {
        cam_data = get_camera_data();
      } catch (...) {
        hal::print<32>(*console, "Camera not connected...\n");
      }
    } else {
      cam_data = std::array<hal::byte, 8>{ 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00 };
    }
    // combine data
    return_bytes[0] = lf_results[0];
    return_bytes[1] = lf_results[1];
    return_bytes[2] = hf_results[0];
    return_bytes[3] = hf_results[1];
    for (size_t i = 0; i < cam_data.size(); i++) {
      return_bytes[4 + i] = cam_data[i];
    }

    for (size_t i = 0; i < 12; i++) {
      checksum += return_bytes[i];
    }
    return_bytes[12] = checksum;

    hal::write(*rs485_transceiver, return_bytes, hal::never_timeout());
    hal::print(*console, "\n");

    // hal::delay(*device_clock, 20ms);
  }
}

std::array<float, 8> read_led_values()
{
  using namespace std::chrono_literals;
  std::array<float, 8> read_values;

  size_t led_count = 0;

  accumulator_reset->level(true);
  counter_reset->level(true);
  counter_reset->level(false);
  hal::delay(*device_clock, 5ms);
  accumulator_reset->level(false);
  hal::delay(*device_clock, 3ms);
  read_values[led_count] = intensity->read();

  while (led_count < 7) {
    hal::delay(*device_clock, 2ms);
    accumulator_reset->level(true);
    led_count++;
    hal::delay(*device_clock, 5ms);
    accumulator_reset->level(false);
    hal::delay(*device_clock, 3ms);
    read_values[led_count] = intensity->read();
  }
  return read_values;
}

std::array<hal::byte, 2> get_strongest_signal(bool high_frequency)
{
  using namespace std::chrono_literals;
  auto read_values = read_led_values();
  std::array<hal::byte, 2> return_bytes;
  auto strongest_value =
    std::max_element(read_values.begin(), read_values.end());
  auto position = std::distance(read_values.begin(), strongest_value);
  return_bytes[0] = position;

  // scale between 0 and 128 to only get 7 bits of data
  auto scaled_float = hal::map(*strongest_value, { 0.0, 1.0 }, { 0.0, 127.0 });
  auto scaled_int = static_cast<uint8_t>(scaled_float);
  // send 1 bit for freq + 7 bits of data to transceiver for each PD
  auto trimmed_int = (scaled_int & 0b01111111);
  hal::byte send_byte = (high_frequency << 7) | trimmed_int;

  if (high_frequency) {
    hal::print(*console, "(HF) ");
  }
  return_bytes[1] = send_byte;
  hal::print<32>(*console, "LED %u: %u    ", position, trimmed_int);

  return return_bytes;
}

bool camera_init()
{
  constexpr hal::byte camera_address = 0x32;
  constexpr hal::byte protocol_address = 0x11;
  constexpr hal::byte header1 = 0x55;
  constexpr hal::byte header2 = 0xAA;
  constexpr hal::byte knock_cmd = 0x2C;
  constexpr hal::byte knock_checksum = 0x3C;
  constexpr hal::byte change_algorithm_cmd = 0x2D;
  constexpr hal::byte change_algorithm_checksum = 0x40;
  constexpr hal::byte obj_tracking_algorithm1 = 0x01;
  constexpr hal::byte obj_tracking_algorithm2 = 0x00;
  constexpr hal::byte ok_cmd = 0x2E;

  constexpr std::array<hal::byte, 6> knock_bytes{
    header1, header2, protocol_address, 0x00, knock_cmd, knock_checksum
  };
  constexpr std::array<hal::byte, 8> algo_change_bytes{
    header1,
    header2,
    protocol_address,
    0x02,
    change_algorithm_cmd,
    obj_tracking_algorithm1,
    obj_tracking_algorithm2,
    change_algorithm_checksum
  };

  std::array<hal::byte, 6> ok_buffer;

  // knock then read OK
  hal::write_then_read(
    *i2c, camera_address, knock_bytes, ok_buffer, hal::never_timeout());
  // check read for ok
  if (ok_buffer[4] != ok_cmd) {
    return false;
  }
  ok_buffer = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  // set algorithm to object tracking
  hal::write_then_read(
    *i2c, camera_address, algo_change_bytes, ok_buffer, hal::never_timeout());
  // check read for ok
  if (ok_buffer[4] != ok_cmd) {
    return false;
  }
  return true;
}

std::array<hal::byte, 8> get_camera_data()
{
  std::array<hal::byte, 8> return_bytes{ 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00 };
  constexpr hal::byte camera_address = 0x32;
  constexpr hal::byte protocol_address = 0x11;
  constexpr hal::byte header1 = 0x55;
  constexpr hal::byte header2 = 0xAA;
  constexpr hal::byte request_blocks_cmd = 0x21;
  constexpr hal::byte request_blocks_checksum = 0x31;

  constexpr std::array<hal::byte, 6> request_blocks_bytes{
    header1,
    header2,
    protocol_address,
    0x00,
    request_blocks_cmd,
    request_blocks_checksum
  };
  std::array<hal::byte, 16> block_data_buffer;
  std::array<hal::byte, 16> info_buffer;

  // request and read back data
  info_buffer =
    hal::write_then_read<16>(*i2c, camera_address, request_blocks_bytes);
  // check command only (everything else is sorta useless)
  if (info_buffer[4] != 0x29) {
    hal::print(*console, "Failed reading response.\n");
    return return_bytes;
  }
  // get block data from read again
  block_data_buffer = hal::read<16>(*i2c, camera_address);
  // check checksum then send data
  hal::byte received_check_sum = 0;
  for (size_t i = 0; i < block_data_buffer.size() - 2; i++) {
    received_check_sum += block_data_buffer[i];
  }

  if (received_check_sum != block_data_buffer.back()) {
    // check sum mismatch
    hal::print<32>(*console,
                   "Checksum mismatch. %X != %X\n",
                   received_check_sum,
                   block_data_buffer.back());
    return return_bytes;
  }
  // only send bytes 5 - 12 (8 bytes)
  for (size_t i = 5; i <= 12; i++) {
    return_bytes[i - 5] = block_data_buffer[i];
  }
  // process data
  uint16_t x = (return_bytes[1] << 8) | return_bytes[0];
  uint16_t y = (return_bytes[3] << 8) | return_bytes[2];

  hal::print<20>(*console, "X: %u   Y: %u", x, y);
  return return_bytes;
}
