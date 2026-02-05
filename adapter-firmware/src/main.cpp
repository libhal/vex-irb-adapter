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

std::array<hal::byte, 3> get_strongest_signal(bool high_frequency,
                                              hal::output_pin& p_counter_reset,
                                              hal::output_pin& p_acc_reset,
                                              hal::adc& p_intensity,
                                              hal::steady_clock& p_clock,
                                              hal::serial& p_console);

bool camera_init(std::span<hal::byte> p_all_data_buffer,
                 hal::i2c& p_i2c,
                 hal::serial& p_console,
                 hal::steady_clock& p_clock);
std::array<hal::byte, 9> get_camera_data(std::span<hal::byte> p_all_data_buffer,
                                         hal::i2c& p_i2c,
                                         hal::serial& p_console);
void flush_buffer(hal::i2c& p_i2c, hal::serial& p_console);
std::span<hal::byte> read_response_data(std::span<hal::byte> p_all_data_buffer,
                                        hal::i2c& p_i2c);

constexpr hal::byte camera_address = 0x50;
constexpr hal::byte header1 = 0x55;
constexpr hal::byte header2 = 0xAA;
constexpr hal::byte any_algo = 0x00;
// constexpr hal::byte obj_tracking_algorithm = 0x03;

constexpr hal::byte knock_cmd = 0x00;
constexpr hal::byte knock_length = 0x0A;
constexpr hal::byte request_blocks_cmd = 0x01;
// constexpr hal::byte change_algorithm_cmd = 0x0A;
// constexpr hal::byte change_algorithm_length = 0x0A;
constexpr hal::byte result_ok = 0x00;

int main()
{
  using namespace std::literals;
  using namespace hal::literals;
  application();
}

void application()
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 256> all_data_buffer;
  auto console = resources::console();
  auto rs485_transceiver = resources::rs485_transceiver();
  auto device_clock = resources::clock();
  auto transceiver_direction = resources::transceiver_direction();
  auto frequency_select = resources::frequency_select();
  auto counter_reset = resources::counter_reset();
  auto accumulator_reset = resources::accumulator_reset();
  auto intensity = resources::intensity();
  auto i2c = resources::i2c();

  hal::print<32>(*console, "Starting application...\n");
  bool camera_connected = false;
  try {
    i2c = resources::i2c();
    camera_connected =
      camera_init(all_data_buffer, *i2c, *console, *device_clock);
  } catch (...) {
    hal::print<32>(*console, "Camera not connected...\n");
  }

  while (true) {
    std::array<hal::byte, 1> read_bytes;
    transceiver_direction->level(false);  // read mode
    hal::read(*rs485_transceiver, read_bytes, hal::never_timeout());
    transceiver_direction->level(true);  // send mode

    // process data
    if (read_bytes[0] == 'l') {  // low freq request
      // set frequency to low
      frequency_select->level(false);
      auto lf_results = get_strongest_signal(false,
                                             *counter_reset,
                                             *accumulator_reset,
                                             *intensity,
                                             *device_clock,
                                             *console);

      hal::write(*rs485_transceiver, lf_results, hal::never_timeout());
      // delay some time to allow bytes to send
      hal::delay(*device_clock, 2ms);

    } else if (read_bytes[0] == 'h') {  // hi freq request
      // set frequency to high
      frequency_select->level(true);
      auto hf_results = get_strongest_signal(true,
                                             *counter_reset,
                                             *accumulator_reset,
                                             *intensity,
                                             *device_clock,
                                             *console);
      hal::write(*rs485_transceiver, hf_results, hal::never_timeout());
      // delay some time to allow bytes to send
      hal::delay(*device_clock, 2ms);

    } else if (read_bytes[0] == 'c') {  // cam request
      std::array<hal::byte, 9> cam_data{ 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00 };
      if (camera_connected) {
        cam_data = get_camera_data(all_data_buffer, *i2c, *console);
      } else {
        try {
          camera_connected =
            camera_init(all_data_buffer, *i2c, *console, *device_clock);
          cam_data = get_camera_data(all_data_buffer, *i2c, *console);
        } catch (...) {
          hal::print<32>(*console, "Camera not connected...\n");
        }
      }
      // send camera data to vex
      hal::write(*rs485_transceiver, cam_data, hal::never_timeout());
      // delay some time to allow bytes to send
      hal::delay(*device_clock, 10ms);
    } else {
      hal::print<32>(*console, "unkown read 0x%02X \n", read_bytes[0]);
    }
  }
}

std::array<hal::byte, 3> get_strongest_signal(bool high_frequency,
                                              hal::output_pin& p_counter_reset,
                                              hal::output_pin& p_acc_reset,
                                              hal::adc& p_intensity,
                                              hal::steady_clock& p_clock,
                                              hal::serial& p_console)
{
  using namespace std::chrono_literals;
  std::array<float, 8> read_values;

  // get data from IRB using current strategy
  size_t led_count = 0;
  // reset and get first photo diode data
  p_acc_reset.level(true);
  p_counter_reset.level(true);
  p_counter_reset.level(false);
  hal::delay(p_clock, 5ms);
  p_acc_reset.level(false);
  hal::delay(p_clock, 3ms);
  read_values[led_count] = p_intensity.read();

  while (led_count < 7) {
    // get PD 1 - 7 data
    hal::delay(p_clock, 2ms);
    p_acc_reset.level(true);
    led_count++;
    hal::delay(p_clock, 5ms);
    p_acc_reset.level(false);
    hal::delay(p_clock, 3ms);
    read_values[led_count] = p_intensity.read();
  }
  std::array<hal::byte, 3> return_bytes;
  // find strongest value and save it to return data
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
    hal::print(p_console, "(HF) ");
  }
  return_bytes[1] = send_byte;
  // add checksum to data
  return_bytes[2] = return_bytes[0] + return_bytes[1];
  hal::print<32>(p_console, "LED %u: %u \n", position, trimmed_int);

  return return_bytes;
}

bool camera_init(std::span<hal::byte> p_all_data_buffer,
                 hal::i2c& p_i2c,
                 hal::serial& p_console,
                 hal::steady_clock& p_clock)
{
  using namespace std::chrono_literals;

  constexpr std::array<hal::byte, 16> knock_bytes{
    header1,      header2,
    knock_cmd,    any_algo,
    knock_length, 0x00,
    0x00,         0x00,
    0x00,         0x00,
    0x00,         0x00,
    0x00,         0x00,
    0x00,         hal::byte(header1 + header2 + knock_length)
  };
  // get rid of any previous data hanging in buffer
  flush_buffer(p_i2c, p_console);
  // knock camera to handshake and get response
  hal::write(p_i2c, camera_address, knock_bytes);
  hal::delay(p_clock, 50ms);
  std::span<hal::byte> ok_buffer = read_response_data(p_all_data_buffer, p_i2c);

  hal::print(p_console, "Knock Response: ");
  for (hal::byte i : ok_buffer) {
    hal::print<20>(p_console, "0x%02X ", i);
  }
  hal::print(p_console, "\n");

  if (ok_buffer[1] != result_ok) {
    hal::print(p_console, "Camera knock not ok.\n");
    return false;
  }
  return true;
}

void flush_buffer(hal::i2c& p_i2c, hal::serial& p_console)
{
  // flush the camera i2c buffer 4 bytes at a time
  bool empty = false;
  std::array<hal::byte, 4> byte_empty = { false, false, false, false };

  while (!empty) {
    std::array<hal::byte, 4> data = hal::read<4>(p_i2c, camera_address);
    // pre emptively set empty to true, if 4 empty bytes in a row are found,
    // buffer is likely empty
    empty = true;
    hal::print(p_console, "Buffer Flush: ");
    for (size_t i = 0; i < data.size(); i++) {
      hal::print<20>(p_console, "0x%02X ", data[i]);
      if (data[i] == 0xFF) {
        // empty byte found
        byte_empty[i] = true;
      }
    }
    for (auto slot : byte_empty) {
      if (slot == false) {
        empty = false;
      }
    }
    hal::print(p_console, "\n");
  }
}

std::span<hal::byte> read_response_data(std::span<hal::byte> p_all_data_buffer,
                                        hal::i2c& p_i2c)
{
  using namespace std::literals;

  hal::byte header = 0x00;
  uint8_t read_attempts = 0;
  hal::byte cmd;
  hal::byte algo;

  // error bytes to be sent in case of error, last byte is a variable to
  // describe what error occured
  // 0x01 = header1 mismatch
  // 0x02 = header2 mismatch
  std::span<hal::byte> error_bytes = p_all_data_buffer.subspan(0, 3);
  error_bytes[0] = 0xDE;
  error_bytes[1] = 0xAD;

  while (header != header1 && read_attempts < 32) {
    header = hal::read<1>(p_i2c, camera_address).front();
    read_attempts++;
  }
  if (header != header1) {
    error_bytes[2] = 0x01;
    return error_bytes;
  }
  read_attempts = 0;
  while (header != header2 && read_attempts < 32) {
    header = hal::read<1>(p_i2c, camera_address).front();
    read_attempts++;
  }
  if (header != header2) {
    error_bytes[2] = 0x02;
    return error_bytes;
  }
  cmd = hal::read<1>(p_i2c, camera_address).front();
  algo = hal::read<1>(p_i2c, camera_address).front();
  uint8_t data_length = hal::read<1>(p_i2c, camera_address).front();

  std::span<hal::byte> data_buffer =
    p_all_data_buffer.subspan(0, data_length + 3);
  data_buffer[0] = cmd;
  // start calculating checksum
  hal::byte calculated_checksum = header1 + header2 + cmd + algo + data_length;

  for (uint8_t i = 0; i < data_length; i++) {
    hal::byte read_byte = hal::read<1>(p_i2c, camera_address).front();
    data_buffer[i + 1] = read_byte;
    calculated_checksum += read_byte;
  }

  // get checksum from camera
  hal::byte received_check_sum = hal::read<1>(p_i2c, camera_address).front();
  data_buffer[data_length + 1] = received_check_sum;
  data_buffer[data_length + 2] = calculated_checksum;

  if (received_check_sum != calculated_checksum) {
    // checksum mismatch, send anyways but change cmd
    data_buffer[0] += 0x10;
  }

  return data_buffer;
}

std::array<hal::byte, 9> get_camera_data(std::span<hal::byte> p_all_data_buffer,
                                         hal::i2c& p_i2c,
                                         hal::serial& p_console)
{
  using namespace std::chrono_literals;

  std::array<hal::byte, 9> return_bytes{ 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00 };

  constexpr std::array<hal::byte, 6> request_blocks_bytes{
    header1, header2, request_blocks_cmd, any_algo, 0x00, 0x00
  };

  try {
    // request and read back data
    hal::write(p_i2c, camera_address, request_blocks_bytes);
    bool flush_needed = false;

    std::span<hal::byte> read_buffer =
      read_response_data(p_all_data_buffer, p_i2c);
    // check command of return info
    if (read_buffer[0] == 0x1B || read_buffer[0] == 0x2B) {
      std::span<hal::byte> block_data_buffer =
        read_response_data(p_all_data_buffer, p_i2c);
      if (block_data_buffer[0] == 0x1C) {
        // process data
        uint16_t x = (block_data_buffer[4] << 8) | block_data_buffer[3];
        uint16_t y = (block_data_buffer[6] << 8) | block_data_buffer[5];
        hal::print<20>(p_console, "X: %u   Y: %u\n", x, y);
        hal::byte send_checksum = 0x00;
        for (size_t i = 3; i < 11; i++) {
          return_bytes[i - 3] = block_data_buffer[i];
          send_checksum += block_data_buffer[i];
        }
        return_bytes[8] = send_checksum;
      } else {
        hal::print(p_console, "Unknown Response: ");
        flush_needed = true;
        for (hal::byte i : block_data_buffer) {
          hal::print<20>(p_console, "0x%02X ", i);
        }
        hal::print(p_console, "\n");
      }
    } else {
      if (read_buffer[0] == 0x1C) {
        uint16_t x = (read_buffer[4] << 8) | read_buffer[3];
        uint16_t y = (read_buffer[6] << 8) | read_buffer[5];
        hal::print<20>(p_console, "X: %u   Y: %u\n", x, y);
        hal::byte send_checksum = 0x00;
        for (size_t i = 3; i < 11; i++) {
          return_bytes[i - 3] = read_buffer[i];
          send_checksum += read_buffer[i];
        }
        return_bytes[8] = send_checksum;
      } else {
        hal::print(p_console, "Unknown Response: ");
        flush_needed = true;
        for (hal::byte i : read_buffer) {
          hal::print<20>(p_console, "0x%02X ", i);
        }
        hal::print(p_console, "\n");
      }
    }
    if (flush_needed) {
      flush_buffer(p_i2c, p_console);
    }
  } catch (...) {
    hal::print(p_console, "Nacked\n");
    flush_buffer(p_i2c, p_console);
  }
  return return_bytes;
}
