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

#include <libhal-arm-mcu/system_control.hpp>
#include <libhal/adc.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

namespace custom {
/**
 * @brief A stand in interface until libhal supports an official watchdog
 * interface.
 *
 */
class watchdog
{
public:
  watchdog() = default;
  virtual void start() = 0;
  virtual void reset() = 0;
  virtual void set_countdown_time(hal::time_duration p_wait_time) = 0;
  virtual bool check_flag() = 0;
  virtual void clear_flag() = 0;
  virtual ~watchdog() = default;
};
}  // namespace custom

namespace resources {
// =======================================================
// Defined by each platform file
// =======================================================
/**
 * @brief Allocator for driver memory
 *
 * The expectation is that the implementation of this allocator is a
 * std::pmr::monotonic_buffer_resource with static memory storage, meaning the
 * memory is fixed in size and memory cannot be deallocated. This is fine for
 * the demos.
 *
 * @return std::pmr::polymorphic_allocator<>
 */
std::pmr::polymorphic_allocator<> driver_allocator();
/**
 * @brief Steady clock that provides the current uptime
 *
 * @return hal::v5::strong_ptr<hal::steady_clock>
 */
hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::serial> rs485_transceiver();
hal::v5::strong_ptr<hal::adc> intensity();
hal::v5::strong_ptr<hal::i2c> i2c();
hal::v5::strong_ptr<hal::output_pin> counter_reset();
hal::v5::strong_ptr<hal::output_pin> accumulator_reset();
hal::v5::strong_ptr<hal::output_pin> transceiver_direction();
hal::v5::strong_ptr<hal::output_pin> frequency_select();

inline void reset()
{
  hal::cortex_m::reset();
}
inline void sleep(hal::time_duration p_duration)
{
  auto delay_clock = resources::clock();
  hal::delay(*delay_clock, p_duration);
}
}  // namespace resources

// Application function is implemented by one of the .cpp files.
void initialize_platform();
void application();
