#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS

// Robot configuration code.

// generating and setting random seed
void
initializeRandomSeed()
{
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}

void
vexcodeInit()
{

  // Initializing random seed.
  initializeRandomSeed();
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void
playVexcodeSound(const char* soundName)
{
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration
#pragma region IRB Adapter Code

#include <cstdbool>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#include <algorithm>
#include <array>
#include <iostream>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace e10 {
// Camera indicies
constexpr auto x_center_low = 0;
constexpr auto x_center_hi = 1;
constexpr auto y_center_low = 2;
constexpr auto y_center_hi = 3;
constexpr auto block_width_low = 4;
constexpr auto block_width_hi = 5;
constexpr auto block_height_low = 6;
constexpr auto block_height_hi = 7;

// Photo diode indicies
constexpr auto diode_number = 0;
constexpr auto intensity_value = 1;

class adapter
{
public:
  struct low_freq_data
  {
    uint8_t strongest_photo_diode = 0;
    uint8_t intensity = 0;
  };

  struct hi_freq_data
  {
    uint8_t strongest_photo_diode = 0;
    uint8_t intensity = 0;
  };

  struct camera_data
  {
    uint16_t x_center = 0;
    uint16_t y_center = 0;
    uint16_t width = 0;
    uint16_t height = 0;
  };

  adapter(uint8_t p_port)
  {
    // make thread to run reads here
    std::array<char, 12> port_buffer{};
    snprintf(port_buffer.data(), port_buffer.size(), "/dev/port%u", p_port);
    m_port_file = fopen(port_buffer.data(), "wb+");
    if (m_port_file == NULL) {
      printf("Failed opening port\n");
    }
  }

  low_freq_data get_low_freq_data()
  {
    std::array<uint8_t, 3> read_buffer{};

    if (request_data('l', read_buffer)) {
      return low_freq_data{
        .strongest_photo_diode =
          static_cast<uint8_t>(static_cast<int>(read_buffer[diode_number])),
        .intensity =
          static_cast<uint8_t>(static_cast<int>(read_buffer[intensity_value])),
      };
    }
    return {};
  }

  hi_freq_data get_hi_freq_data()
  {
    std::array<uint8_t, 3> read_buffer{};

    if (request_data('h', read_buffer)) {
      return hi_freq_data{
        .strongest_photo_diode =
          static_cast<uint8_t>(read_buffer[diode_number]),
        .intensity = static_cast<uint8_t>(read_buffer[intensity_value]),
      };
    }
    return {};
  }

  camera_data get_cam_data()
  {
    std::array<uint8_t, 9> read_buffer{};

    if (request_data('c', read_buffer)) {
      return camera_data{
        .x_center = static_cast<uint16_t>((read_buffer[x_center_hi] << 8) |
                                          read_buffer[x_center_low]),
        .y_center = static_cast<uint16_t>((read_buffer[y_center_hi] << 8) |
                                          read_buffer[y_center_low]),
        .width = static_cast<uint16_t>((read_buffer[block_width_hi] << 8) |
                                       read_buffer[block_width_low]),
        .height = static_cast<uint16_t>((read_buffer[block_height_hi] << 8) |
                                        read_buffer[block_height_low]),
      };
    }
    return {};
  }

private:
  bool request_data(char p_command, std::span<uint8_t> p_read_buffer)
  {
    bool success = true;
    if (m_port_file == NULL) {
      printf("Port not open...\n");
      return false;
    }

    auto bytes_written = fwrite(&p_command, sizeof(p_command), 1, m_port_file);
    if (bytes_written != sizeof(p_command)) {
      printf("Failed write to port, %zu bytes written.\n", bytes_written);
      return false;
    }

    // Wait 10ms for data
    vex::this_thread::sleep_for(10);

    // rudimentary "timeout"
    int attempts = 0;
    auto read_span = p_read_buffer;
    while (not read_span.empty()) {
      auto bytes_read = fread(read_span.data(),
                              sizeof(read_span[0]),
                              read_span.size_bytes(),
                              m_port_file);
      read_span.subspan(bytes_read);

      if (++attempts >= 15) {
        printf("Failed read from port, %zu bytes read:  ", bytes_read);
        // TODO(kammce): print out the bytes
        return false;
      }
    }

    auto const body = p_read_buffer.first(p_read_buffer.size() - 1);
    uint8_t const checksum = std::ranges::reduce(body, uint8_t{});

    if (checksum == static_cast<uint8_t>(p_read_buffer.back())) {
      printf("Checksum mismatch...\n");
      return false;
    }

    return true;
  }
  FILE* m_port_file;
};

} // namespace e10

#pragma endregion IRB Adapter Code

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// Devices added with codev5.vex.com GUI are defined above in the
// VEXcode Generated Robot Configuration region in the
// "Robot configuration code" section

// devices can also be defined manually as done here
vex::motor right_motor = motor(PORT20, ratio18_1, false);
vex::motor left_motor = motor(PORT10, ratio18_1, true);
vex::limit front_bumper = limit(Brain.ThreeWirePort.A);

enum class mission_state
{
  find_beacon,
  backup,
  find_object,
  mission_complete,
};

mission_state state = mission_state::find_beacon;

void
switch_pressed()
{
  // check state and advance to next step
  if (state == mission_state::find_beacon) {
    state = mission_state::backup;
  } else if (state == mission_state::find_object) {
    state = mission_state::mission_complete;
  }
}

int
main()
{
  Brain.Timer.clear();
  int const port_number = 1;
  e10::adapter board(port_number);
  uint8_t threshold = 5;
  uint8_t max_power = 40;

  front_bumper.pressed(switch_pressed);

  while (1) {
    switch (state) {
      case mission_state::find_beacon: {
        auto low_freq_data = board.get_low_freq_data();
        int direction = low_freq_data.strongest_photo_diode;

        // write to screen
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10,
                             20,
                             "Low Frequency Beacon: PD %u: %u ",
                             low_freq_data.strongest_photo_diode,
                             low_freq_data.intensity);
        Brain.Screen.printAt(10, 40, "Camera Data: Waiting...");

        // spin in place if nothing detected
        if (low_freq_data.intensity < threshold) {
          right_motor.spin(reverse, max_power / 2, rpm);
          left_motor.spin(forward, max_power / 2, rpm);
        }
        // Go forward
        else if (direction == 3 || direction == 4) {
          right_motor.spin(forward, max_power, rpm);
          left_motor.spin(forward, max_power, rpm);
        }
        // Go right
        else if (direction < 3) {
          right_motor.spin(forward, max_power * (direction / 3), rpm);
          left_motor.spin(forward, max_power, rpm);
        }
        // Go left
        else if (direction > 4) {
          right_motor.spin(forward, max_power, rpm);
          left_motor.spin(forward, max_power * ((7 - direction) / 3), rpm);
        }

        if (front_bumper.pressing()) {
          state = mission_state::backup;
        }
        break;
      }
      case mission_state::backup: {
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 20, "Low Frequency Beacon: Collected");
        Brain.Screen.printAt(10, 40, "Camera Data: waiting...");
        printf("Backing up...\n");

        // stop moving
        right_motor.stop();
        left_motor.stop();
        this_thread::sleep_for(500);
        // back up for 2 sec
        right_motor.spin(reverse, max_power, rpm);
        left_motor.spin(reverse, max_power, rpm);
        this_thread::sleep_for(2000);
        // stop again
        right_motor.stop();
        left_motor.stop();
        state = mission_state::find_object;
        max_power = 25;
        break;
      }
      case mission_state::find_object: {
        auto cam_data = board.get_cam_data();
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 20, "Low Frequency Beacon: Collected");
        Brain.Screen.printAt(10,
                             40,
                             "Camera Data: %d, %u (%u X %u)",
                             cam_data.x_center,
                             cam_data.y_center,
                             cam_data.width,
                             cam_data.height);
        printf("Camera Data: X:%u Y:%u    %ux%u\n",
               cam_data.x_center,
               cam_data.y_center,
               cam_data.width,
               cam_data.height);

        // spin if nothing detected
        if (cam_data.width == 0) {
          right_motor.spin(forward, 10, rpm);
          left_motor.spin(reverse, 10, rpm);
        }
        // go forward if relatively center (x center = 320, y center = 240)
        else if (310 <= cam_data.x_center && cam_data.x_center < 330) {
          right_motor.spin(forward, max_power, rpm);
          left_motor.spin(forward, max_power, rpm);
        }
        // turn left or right if not
        else {
          float turn_factor = (cam_data.x_center - 320.0f);
          turn_factor = turn_factor / 320.0f;
          // turn right
          if (turn_factor > 0) {
            auto adjusted_speed = turn_factor * max_power;
            left_motor.spin(forward, max_power * 0.97, rpm);
            right_motor.spin(forward, adjusted_speed, rpm);
          }
          // turn left
          if (turn_factor < 0) {
            auto adjusted_speed = -turn_factor * max_power;
            left_motor.spin(forward, adjusted_speed + 3, rpm);
            right_motor.spin(forward, max_power * 0.97, rpm);
          }
        }
        break;
      }
      case mission_state::mission_complete: {
        right_motor.stop();
        left_motor.stop();
        float timer_seconds = Brain.Timer.time(seconds);
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 20, "Low Frequency Beacon: Collected");
        Brain.Screen.printAt(10, 40, "Camera Data: Collected");
        Brain.Screen.printAt(
          10, 80, "Mission Complete in %.3f seconds!", timer_seconds);
        this_thread::sleep_for(15000);
        Brain.programStop();
      }
      default:
        break;
    }
  }
}
