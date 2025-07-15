// // Copyright 2024 Khalil Estell
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //      http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// #include <libhal-arm-mcu/dwt_counter.hpp>
// #include <libhal-arm-mcu/stm32f1/adc.hpp>
// #include <libhal-arm-mcu/stm32f1/clock.hpp>
// #include <libhal-arm-mcu/stm32f1/constants.hpp>
// #include <libhal-arm-mcu/stm32f1/output_pin.hpp>
// #include <libhal-arm-mcu/stm32f1/uart.hpp>
// #include <libhal-arm-mcu/system_control.hpp>
// #include <libhal-util/atomic_spin_lock.hpp>

// #include <resource_list.hpp>

// void initialize_platform(resource_list& p_list)
// {
//   using namespace hal::literals;

//   p_list.reset = +[]() { hal::cortex_m::reset(); };
//   // Set the MCU to the maximum clock speed
//   hal::stm32f1::maximum_speed_using_internal_oscillator();

//   static hal::cortex_m::dwt_counter counter(
//     hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
//   p_list.clock = &counter;

//   static hal::stm32f1::uart console(
//     hal::port<1>,
//     hal::buffer<128>,
//     hal::serial::settings{ .baud_rate = 115200 });

//   p_list.console = &console;

//   static hal::stm32f1::uart uart2(hal::port<2>,
//                                   hal::buffer<128>,
//                                   hal::serial::settings{ .baud_rate = 115200
//                                   });
//   p_list.rs485_transceiver = &uart2;

//   static hal::stm32f1::output_pin pb14('B', 14);
//   static hal::stm32f1::output_pin pb13('B', 13);
//   static hal::stm32f1::output_pin pb12('B', 12);
//   static hal::stm32f1::output_pin pa0('A', 0);
//   static hal::atomic_spin_lock adc_lock;
//   static hal::stm32f1::adc<hal::stm32f1::peripheral::adc1> adc(adc_lock);
//   static auto a0 = adc.acquire_channel(hal::stm32f1::adc_pins::pb0);

//   p_list.counter_reset = &pb14;
//   p_list.accumulator_reset = &pb13;
//   p_list.frequency_select = &pb12;
//   p_list.transceiver_direction = &pa0;
//   p_list.intensity = &a0;
// }

// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f1/adc.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/spi.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/stm32f1/usart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/bit_bang_spi.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include <libhal/pointers.hpp>
#include <resource_list.hpp>

namespace resources {
using namespace hal::literals;
using st_peripheral = hal::stm32f1::peripheral;

std::pmr::polymorphic_allocator<> driver_allocator()
{
  static std::array<hal::byte, 1024> driver_memory{};
  static std::pmr::monotonic_buffer_resource resource(
    driver_memory.data(),
    driver_memory.size(),
    std::pmr::null_memory_resource());
  return &resource;
}

auto& gpio_a()
{
  static hal::stm32f1::gpio<st_peripheral::gpio_a> gpio;
  return gpio;
}
auto& gpio_b()
{
  static hal::stm32f1::gpio<st_peripheral::gpio_b> gpio;
  return gpio;
}

hal::v5::optional_ptr<hal::cortex_m::dwt_counter> clock_ptr;
hal::v5::strong_ptr<hal::steady_clock> clock()
{
  if (not clock_ptr) {
    auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
    clock_ptr = hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(
      driver_allocator(), cpu_frequency);
  }
  return clock_ptr;
}

hal::v5::strong_ptr<hal::serial> console()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::uart>(
    driver_allocator(), hal::port<1>, hal::buffer<128>);
}

hal::v5::strong_ptr<hal::serial> rs485_transceiver()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::uart>(
    driver_allocator(), hal::port<2>, hal::buffer<128>);
}

hal::v5::strong_ptr<hal::adc> intensity()
{
#if 0
  static hal::atomic_spin_lock adc_lock;
  static hal::stm32f1::adc<st_peripheral::adc1> adc(adc_lock);
  auto pb0 = adc.acquire_channel(hal::stm32f1::adc_pins::pb0);
  return hal::v5::make_strong_ptr<decltype(pb0)>(driver_allocator(),
                                                 std::move(pb0));
#endif
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::i2c> i2c()
{
  static auto sda_output_pin = gpio_b().acquire_output_pin(7);
  static auto scl_output_pin = gpio_b().acquire_output_pin(6);
  auto clock = resources::clock();
  return hal::v5::make_strong_ptr<hal::bit_bang_i2c>(driver_allocator(),
                                                     hal::bit_bang_i2c::pins{
                                                       .sda = &sda_output_pin,
                                                       .scl = &scl_output_pin,
                                                     },
                                                     *clock);
}

hal::v5::strong_ptr<hal::output_pin> counter_reset()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(14))>(
    driver_allocator(), gpio_b().acquire_output_pin(14));
}

hal::v5::strong_ptr<hal::output_pin> accumulator_reset()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(13))>(
    driver_allocator(), gpio_b().acquire_output_pin(13));
}

hal::v5::strong_ptr<hal::output_pin> frequency_select()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(12))>(
    driver_allocator(), gpio_b().acquire_output_pin(12));
}

hal::v5::strong_ptr<hal::output_pin> transceiver_direction()
{
  return hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(0))>(
    driver_allocator(), gpio_a().acquire_output_pin(0));
}

// Watchdog implementation using global function pattern from original
class stm32f103c8_watchdog : public custom::watchdog
{
public:
  void start() override
  {
    m_stm_watchdog.start();
  }

  void reset() override
  {
    m_stm_watchdog.reset();
  }

  void set_countdown_time(hal::time_duration p_wait_time) override
  {
    m_stm_watchdog.set_countdown_time(p_wait_time);
  }

  bool check_flag() override
  {
    return m_stm_watchdog.check_flag();
  }

  void clear_flag() override
  {
    m_stm_watchdog.clear_flag();
  }

private:
  hal::stm32f1::independent_watchdog m_stm_watchdog{};
};

hal::v5::strong_ptr<custom::watchdog> watchdog()
{
  return hal::v5::make_strong_ptr<stm32f103c8_watchdog>(driver_allocator());
}

[[noreturn]] void terminate_handler() noexcept
{
  if (not clock_ptr) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern
  auto status_led = resources::frequency_select();
  auto clock = resources::clock();

  while (true) {
    using namespace std::chrono_literals;
    status_led->level(false);
    hal::delay(*clock, 100ms);
    status_led->level(true);
    hal::delay(*clock, 100ms);
    status_led->level(false);
    hal::delay(*clock, 100ms);
    status_led->level(true);
    hal::delay(*clock, 1000ms);
  }
}

}  // namespace resources

void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(resources::terminate_handler);
  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();
  hal::stm32f1::release_jtag_pins();
}
