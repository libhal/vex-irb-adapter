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

#include <libhal-micromod/micromod.hpp>

#include <resource_list.hpp>

void initialize_platform(resource_list& p_list)
{
  using namespace hal::literals;
  using namespace hal::micromod;

  p_list.reset = +[]() { hal::micromod::v1::reset(); },

  v1::initialize_platform();

  p_list.clock = &v1::uptime_clock();
  p_list.console = &v1::console(hal::buffer<128>);

  p_list.counter_reset = &v1::output_g6();
  p_list.counter_clock = &v1::output_g5();
  p_list.frequency_select = &v1::output_g4();
  p_list.transceiver_direction = &v1::output_g0();
  p_list.intensity = &v1::a0();
}
