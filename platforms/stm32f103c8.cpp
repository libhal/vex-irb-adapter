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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/stm32f1/adc.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/atomic_spin_lock.hpp>

#include <resource_list.hpp>

void initialize_platform(resource_list& p_list)
{
  using namespace hal::literals;

  p_list.reset = +[]() { hal::cortex_m::reset(); };
  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();

  static hal::cortex_m::dwt_counter counter(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
  p_list.clock = &counter;

  static hal::stm32f1::uart console(
    hal::port<1>,
    hal::buffer<128>,
    hal::serial::settings{ .baud_rate = 115200 });

  p_list.console = &console;

  static hal::stm32f1::uart uart2(hal::port<2>,
                                  hal::buffer<128>,
                                  hal::serial::settings{ .baud_rate = 115200 });
  p_list.rs485_transceiver = &uart2;

  static hal::stm32f1::output_pin pb14('B', 14);
  static hal::stm32f1::output_pin pb13('B', 13);
  static hal::stm32f1::output_pin pb12('B', 12);
  static hal::stm32f1::output_pin pa0('A', 0);
  static hal::atomic_spin_lock adc_lock;
  static hal::stm32f1::adc<hal::stm32f1::peripheral::adc1> adc(adc_lock);
  static auto a0 = adc.acquire_channel(hal::stm32f1::adc_pins::pb0);

  p_list.counter_reset = &pb14;
  p_list.counter_clock = &pb13;
  p_list.frequency_select = &pb12;
  p_list.transceiver_direction = &pa0;
  p_list.intensity = &a0;
}
