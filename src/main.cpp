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

#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

void application();
std::array<float, 8> read_led_values();
hal::byte send_strongest_signal(bool high_frequency);
bool camera_init();
hal::byte send_camera_data();

int main()
{
  using namespace std::literals;
  using namespace hal::literals;
  auto console = resources::console();

  hal::print<32>(*console, "Starting application...");

  application();

  // Terminate if the code reaches this point.
  std::terminate();
}

void application()
{
  using namespace std::chrono_literals;
  auto rs485_transceiver = resources::rs485_transceiver();
  auto transceiver_direction = resources::transceiver_direction();
  auto frequency_select = resources::frequency_select();
  auto clock = resources::clock();
  transceiver_direction->level(true);
  bool high_frequency = false;

  // wait a bit and then check for camera
  hal::delay(*clock, 1s);
  bool camera_connected = camera_init();

  while (true) {
    hal::byte checksum = 0x00;
    // get low frequency strongest signal
    frequency_select->level(high_frequency);
    checksum += send_strongest_signal(high_frequency);
    // get high frequency strongest signal
    high_frequency = !high_frequency;
    frequency_select->level(high_frequency);
    checksum += send_strongest_signal(high_frequency);
    // get camera data
    hal::byte cam_checksum = 0x00;
    if (camera_connected) {
      cam_checksum += send_camera_data();
    }
    // send all 0's if not connected or no data to read
    if (!camera_connected || cam_checksum == 0x00) {
      hal::write(*rs485_transceiver,
                 std::array<hal::byte, 9>{
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                 hal::never_timeout());
    }
    hal::delay(*clock, 100ms);
  }
}

std::array<float, 8> read_led_values()
{
  using namespace std::chrono_literals;
  auto clock = resources::clock();
  std::array<float, 8> read_values;
  auto counter_reset = resources::counter_reset();
  auto accumulator_reset = resources::accumulator_reset();
  auto intensity = resources::intensity();
  size_t led_count = 0;

  accumulator_reset->level(true);
  counter_reset->level(true);
  counter_reset->level(false);
  hal::delay(*clock, 5ms);
  accumulator_reset->level(false);
  hal::delay(*clock, 3ms);
  read_values[led_count] = intensity->read();

  while (led_count < 7) {
    hal::delay(*clock, 2ms);
    accumulator_reset->level(true);
    led_count++;
    hal::delay(*clock, 5ms);
    accumulator_reset->level(false);
    hal::delay(*clock, 3ms);
    read_values[led_count] = intensity->read();
  }
  return read_values;
}

hal::byte send_strongest_signal(bool high_frequency)
{
  using namespace std::chrono_literals;
  auto console = resources::console();
  auto rs485_transceiver = resources::rs485_transceiver();
  auto read_values = read_led_values();

  auto strongest_value =
    std::max_element(read_values.begin(), read_values.end());
  auto position = std::distance(read_values.begin(), strongest_value);
  hal::write(*rs485_transceiver,
             std::array<hal::byte, 1>{ hal::byte(position) },
             hal::never_timeout());
  hal::byte checksum = hal::byte(position);

  // scale between 0 and 128 to only get 7 bits of data
  auto scaled_float = hal::map(*strongest_value, { 0.0, 1.0 }, { 0.0, 128.0 });
  auto scaled_int = static_cast<uint8_t>(scaled_float);
  // send 1 bit for freq + 7 bits of data to transceiver for each PD
  auto trimmed_int = (scaled_int & 0b01111111);
  hal::byte send_byte = (high_frequency << 7) | trimmed_int;

  if (high_frequency) {
    hal::print(*console, "(HF) ");
  }

  hal::print<32>(*console, "LED %u: %u \n", position, trimmed_int);
  hal::write(*rs485_transceiver,
             std::array<hal::byte, 1>{ send_byte },
             hal::never_timeout());
  checksum += send_byte;
  return checksum;
}

bool camera_init()
{
  auto i2c = resources::i2c();
  auto console = resources::console();
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

hal::byte send_camera_data()
{
  auto i2c = resources::i2c();
  auto console = resources::console();
  auto rs485_transceiver = resources::rs485_transceiver();

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
    return 0x00;
  }
  // get block data from read again
  block_data_buffer = hal::read<16>(*i2c, camera_address);
  // check checksum then send data
  hal::byte received_check_sum = 0;
  for (auto& byte : block_data_buffer)
    received_check_sum += byte;

  if (received_check_sum != block_data_buffer.back()) {
    // check sum mismatch
    return 0x00;
  }
  // only send bytes 5 - 12
  hal::byte checksum = 0x00;
  for (size_t i = 5; i <= 12; i++) {
    hal::write(*rs485_transceiver,
               std::array<hal::byte, 1>{ block_data_buffer[i] },
               hal::never_timeout());
    checksum += block_data_buffer[i];
  }
  // process data
  uint16_t x = (block_data_buffer[6] << 4) | block_data_buffer[5];
  uint16_t y = (block_data_buffer[8] << 4) | block_data_buffer[7];

  hal::print<20>(*console, "X: %u   Y: %u \n", x, y);
  return checksum;
}
