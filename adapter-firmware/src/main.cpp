// Copyright 2025 Malia Labor
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

#include <cinttypes>
#include <cstddef>
#include <cstdint>

#include <algorithm>
#include <array>
#include <numeric>

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

struct sample_args
{
  hal::output_pin& counter_reset;
  hal::output_pin& counter_clock;
  hal::adc& intensity;
  hal::adc& reference;
  hal::steady_clock& clock;
};

enum class irb_freq : hal::u8
{
  low = 0,
  high = 1,
};

std::array<hal::byte, 8> sample_all_diodes(sample_args p_args);
std::array<hal::byte, 3> get_strongest_signal(
  irb_freq p_freq,
  std::array<hal::byte, 8> const& p_samples);

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
[[maybe_unused]] constexpr hal::byte obj_tracking_algorithm = 0x03;

constexpr hal::byte knock_cmd = 0x00;
constexpr hal::byte knock_length = 0x0A;
constexpr hal::byte request_blocks_cmd = 0x01;
[[maybe_unused]] constexpr hal::byte change_algorithm_cmd = 0x0A;
[[maybe_unused]] constexpr hal::byte change_algorithm_length = 0x0A;
constexpr hal::byte result_ok = 0x00;

int main()
{
  initialize_platform();
  application();
}

#if not defined(E10_ADAPTER_VERSION)
#define E10_ADAPTER_VERSION "undef"
#endif

constexpr std::string_view version = E10_ADAPTER_VERSION;

void application()
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 256> all_data_buffer{};
  auto console = resources::console();
  auto rs485_transceiver = resources::rs485_transceiver();
  auto device_clock = resources::clock();
  auto transceiver_direction = resources::transceiver_direction();
  auto frequency_select = resources::frequency_select();
  auto counter_reset = resources::counter_reset();
  auto counter_clock = resources::counter_clock();
  auto intensity = resources::intensity();
  auto adc_reference = resources::adc_reference();
  auto i2c = resources::i2c();

  hal::print<64>(*console, "Starting application...\n");
  bool camera_connected = false;

  try {
    camera_connected =
      camera_init(all_data_buffer, *i2c, *console, *device_clock);
  } catch (...) {
    hal::print<64>(*console, "Camera not connected...\n");
  }

  while (true) {
    std::array<hal::byte, 1> read_bytes{};
    // Put RS485 transceiver into read mode
    transceiver_direction->level(false);
    auto response = rs485_transceiver->read(read_bytes);

    // Used by commands like 'v' to only respond to the serial port that
    // requested the version.
    bool command_found = false;
    bool console_request = false;

    if (response.data.size() == read_bytes.size()) {
      // We received some data, put RS485 transceiver into send mode
      transceiver_direction->level(true);
      command_found = true;
    } else {
      response = console->read(read_bytes);
      if (response.data.size() == read_bytes.size()) {
        command_found = true;
        console_request = true;
      }
    }

    // If a command wasn't received by either serial port, skip the rest of the
    // loop.
    if (not command_found) {
      continue;
    }

    auto const start = device_clock->uptime();
    // process data
    switch (read_bytes[0]) {
      case 'v': {  // Version
        hal::write(*console, version, hal::never_timeout());
        hal::write(*rs485_transceiver, version, hal::never_timeout());
        break;
      }
      case 'a': {  // Both low and high frequency signals
        // Sample the voltage divider's voltage (max expected voltage from the
        // sensor)
        auto const reference_ratio = adc_reference->read();
        // set frequency to low
        frequency_select->level(false);
        auto const low_frequency_samples =
          sample_all_diodes({ .counter_reset = *counter_reset,
                              .counter_clock = *counter_clock,
                              .intensity = *intensity,
                              .reference = *adc_reference,
                              .clock = *device_clock });

        // set frequency to high
        frequency_select->level(true);
        auto const high_frequency_samples =
          sample_all_diodes({ .counter_reset = *counter_reset,
                              .counter_clock = *counter_clock,
                              .intensity = *intensity,
                              .reference = *adc_reference,
                              .clock = *device_clock });

        if (console_request) {
          hal::print<64>(*console, "Reference Ratio = %.6f\n", reference_ratio);
          hal::print(*console, " Low Samples: [");
          for (auto sample : low_frequency_samples) {
            hal::print<64>(*console, "%03u, ", sample);
          }
          hal::print(*console, "]\n");
          hal::print(*console, "High Samples: [");
          for (auto sample : high_frequency_samples) {
            hal::print<64>(*console, "%03u, ", sample);
          }
          hal::print(*console, "]\n");
        } else {
          // Calculate checksum
          std::array<hal::u8, 1> checksum{};
          checksum[0] = std::accumulate(
            low_frequency_samples.begin(), low_frequency_samples.end(), 0);
          checksum[0] = std::accumulate(high_frequency_samples.begin(),
                                        high_frequency_samples.end(),
                                        checksum[0]);

          // Send data over
          hal::write(
            *rs485_transceiver, low_frequency_samples, hal::never_timeout());
          hal::write(
            *rs485_transceiver, high_frequency_samples, hal::never_timeout());
          hal::write(*rs485_transceiver, checksum, hal::never_timeout());
        }
        break;
      }
      case 'l': {
        // set frequency to low
        frequency_select->level(false);
        auto const low_frequency_samples =
          sample_all_diodes({ .counter_reset = *counter_reset,
                              .counter_clock = *counter_clock,
                              .intensity = *intensity,
                              .reference = *adc_reference,
                              .clock = *device_clock });
        auto const payload =
          get_strongest_signal(irb_freq::low, low_frequency_samples);
        hal::write(*rs485_transceiver, payload, hal::never_timeout());
        break;
      }
      case 'h': {
        // set frequency to high
        frequency_select->level(true);
        auto const high_frequency_samples =
          sample_all_diodes({ .counter_reset = *counter_reset,
                              .counter_clock = *counter_clock,
                              .intensity = *intensity,
                              .reference = *adc_reference,
                              .clock = *device_clock });
        auto const payload =
          get_strongest_signal(irb_freq::high, high_frequency_samples);
        hal::write(*rs485_transceiver, payload, hal::never_timeout());
        break;
      }
      case 'c': {
        std::array<hal::byte, 9> cam_data{ 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00 };
        try {
          if (camera_connected) {
            cam_data = get_camera_data(all_data_buffer, *i2c, *console);
          } else {
            hal::print<64>(*console, "Reconnecting Camera\n");
            camera_connected =
              camera_init(all_data_buffer, *i2c, *console, *device_clock);
            cam_data = get_camera_data(all_data_buffer, *i2c, *console);
          }
        } catch (...) {
          camera_connected = false;
          hal::print<64>(*console, "Camera not connected...\n");
        }
        hal::write(*rs485_transceiver, cam_data, hal::never_timeout());
        break;
      }
      default:
        hal::print<64>(*console, "Unknown read 0x%02X \n", read_bytes[0]);
        break;
    }

    auto const end = device_clock->uptime();
    auto const delta = (end - start);
    hal::print<128>(
      *console, "t: %" PRIu64 ", f: %f\n", delta, device_clock->frequency());
    // Wait before setting the transceiver into read mode
    hal::delay(*device_clock, 100us);
  }
}

std::array<hal::byte, 8> sample_all_diodes(sample_args p_args)
{
  using namespace std::chrono_literals;

  hal::output_pin& counter_reset = p_args.counter_reset;
  hal::output_pin& counter_clock = p_args.counter_clock;
  hal::adc& intensity = p_args.intensity;
  hal::adc& reference = p_args.reference;
  hal::steady_clock& clock = p_args.clock;

  std::array<hal::u8, 8> samples{};

  // Sample the voltage divider's voltage (max expected voltage from the sensor)
  auto const reference_ratio = reference.read();

  // Reset IRB hardware counter used to multiplex/select the photo diode to
  // sample
  counter_reset.level(true);
  // Wait to allow reset to take hold
  hal::delay(clock, 10us);
  // Clear counter reset, photo-diode 0 should be accumulating charge
  counter_reset.level(false);

  for (auto& value : samples) {
    counter_clock.level(true);
    hal::delay(clock, 3ms);
    // Increment the counter to the next photo-diode
    counter_clock.level(false);
    hal::delay(clock, 5ms);
    // Sample the analog value
    auto const reading = intensity.read();
    // Map float to u8 relative to the reference ratio
    auto const mapped_reading =
      hal::map(reading, { 0.0f, reference_ratio }, { 0.0f, 255.0f });
    auto const clamped_value =
      std::clamp(static_cast<int>(mapped_reading), 0, 255);
    value = static_cast<hal::u8>(clamped_value);
  }

  counter_reset.level(true);
  counter_clock.level(true);

  return samples;
}

std::array<hal::byte, 3> get_strongest_signal(
  irb_freq p_freq,
  std::array<hal::u8, 8> const& p_samples)
{
  std::array<hal::byte, 3> return_bytes{};

  // Find strongest value of the samples
  auto const strongest_value = std::ranges::max_element(p_samples);
  auto const position = std::distance(p_samples.begin(), strongest_value);
  return_bytes[0] = position;

  // send 1 bit for freq + 7 bits of data to transceiver for each PD
  return_bytes[1] = static_cast<hal::u8>(*strongest_value >> 1);

  if (p_freq == irb_freq::high) {
    return_bytes[1] |= 1 << 7;  // set "HIGH FREQUENCY" 7th bit
  }
  // Calculate checksum
  return_bytes[2] = return_bytes[0] + return_bytes[1];

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
    hal::print<64>(p_console, "0x%02X ", unsigned{ i });
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
    // preemptively set empty to true, if 4 empty bytes in a row are found,
    // buffer is likely empty
    empty = true;
    hal::print(p_console, "Buffer Flush: ");
    for (size_t i = 0; i < data.size(); i++) {
      hal::print<64>(p_console, "0x%02X ", unsigned{ data[i] });
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
  hal::byte cmd = 0;
  hal::byte algo = 0;

  // error bytes to be sent in case of error, last byte is a variable to
  // describe what error occurred
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
        hal::print<64>(
          p_console, "X: %u   Y: %u\n", unsigned{ x }, unsigned{ y });
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
          hal::print<64>(p_console, "0x%02X ", i);
        }
        hal::print(p_console, "\n");
      }
    } else {
      if (read_buffer[0] == 0x1C) {
        uint16_t x = (read_buffer[4] << 8) | read_buffer[3];
        uint16_t y = (read_buffer[6] << 8) | read_buffer[5];
        hal::print<64>(
          p_console, "X: %u   Y: %u\n", unsigned{ x }, unsigned{ y });
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
          hal::print<64>(p_console, "0x%02X ", i);
        }
        hal::print(p_console, "\n");
      }
    }
    if (flush_needed) {
      flush_buffer(p_i2c, p_console);
    }
  } catch (...) {
    hal::print(p_console, "NACK\n");
    flush_buffer(p_i2c, p_console);
  }
  return return_bytes;
}
