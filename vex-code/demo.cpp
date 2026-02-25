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

#include <cstddef>
#include <cstdio>
#include <iostream>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <vector>

namespace e10 {
enum camera_index
{
  x_center_low,
  x_center_hi,
  y_center_low,
  y_center_hi,
  block_width_low,
  block_width_hi,
  block_height_low,
  block_height_hi
};

enum photo_diode_index
{
  diode_number,
  intensity_value
};
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
    : m_port(p_port)
  {
    // make thread to run reads here
    char port_buffer[12];
    sprintf(port_buffer, "/dev/port%u", m_port);
    m_port_file = fopen(port_buffer, "wb+");
    if (m_port_file == NULL) {
      printf("Failed opening port\n");
    }
  }

  low_freq_data get_low_freq_data()
  {
    std::vector<char> read_buffer(3);
    std::vector<char> write_buffer = { 'l' };
    low_freq_data return_data;

    if (request_data(write_buffer, read_buffer)) {
      // check checksum
      auto calculated_checksum =
        read_buffer[diode_number] + read_buffer[intensity_value];
      if (calculated_checksum != read_buffer.back()) {
        printf("Checksum mismatch...\n");
        return return_data;
      }
      return_data.strongest_photo_diode = (int)read_buffer[diode_number];
      return_data.intensity = (int)read_buffer[intensity_value] & 0b01111111;
    }
    return return_data;
  }

  hi_freq_data get_hi_freq_data()
  {
    std::vector<char> read_buffer(3);
    std::vector<char> write_buffer = { 'h' };
    hi_freq_data return_data;

    if (request_data(write_buffer, read_buffer)) {
      // check checksum
      auto calculated_checksum =
        read_buffer[diode_number] + read_buffer[intensity_value];
      if (calculated_checksum != read_buffer.back()) {
        printf("Checksum mismatch...\n");
        return return_data;
      }
      return_data.strongest_photo_diode = (int)read_buffer[diode_number];
      return_data.intensity = (int)read_buffer[intensity_value] & 0b01111111;
    }
    return return_data;
  }

  camera_data get_cam_data()
  {
    std::vector<char> read_buffer(9);
    std::vector<char> write_buffer = { 'c' };
    camera_data return_data;

    if (request_data(write_buffer, read_buffer)) {
      // check checksum
      uint8_t calculated_checksum = 0x00;
      for (size_t i = 0; i < read_buffer.size() - 1; i++) {
        calculated_checksum += read_buffer[i];
      }
      if (calculated_checksum != read_buffer.back()) {
        printf("Checksum mismatch...\n");
        return return_data;
      }
      return_data.x_center = ((uint8_t)read_buffer[x_center_hi] << 8) |
                             (uint8_t)read_buffer[x_center_low];
      return_data.y_center = ((uint8_t)read_buffer[y_center_hi] << 8) |
                             (uint8_t)read_buffer[y_center_low];
      return_data.width = ((uint8_t)read_buffer[block_width_hi] << 8) |
                          (uint8_t)read_buffer[block_width_low];
      return_data.height = ((uint8_t)read_buffer[block_height_hi] << 8) |
                           (uint8_t)read_buffer[block_height_low];
    }
    return return_data;
  }

private:
  bool request_data(std::vector<char>& write_buffer,
                    std::vector<char>& read_buffer)
  {
    bool success = true;
    if (m_port_file == NULL) {
      printf("Port not open...\n");
      return false;
    }

    auto bytes_written = fwrite(write_buffer.data(),
                                sizeof write_buffer[0],
                                write_buffer.size(),
                                m_port_file);
    if (bytes_written != write_buffer.size()) {
      printf("Failed write to port, %u bytes written.\n", bytes_written);
      success = false;
    }
    // wait for data
    vex::this_thread::sleep_for(90);

    // rudimentary "timeout"
    size_t bytes_read = 0;
    int attempts = 0;
    while (attempts < 15 && bytes_read != read_buffer.size()) {
      bytes_read += fread(&read_buffer[bytes_read],
                          sizeof read_buffer[0],
                          read_buffer.size() - bytes_read,
                          m_port_file);
      attempts++;
    }
    if (bytes_read != read_buffer.size()) {
      printf("Failed read from port, %u bytes read:  ", bytes_read);
      success = false;
      for (size_t i = 0; i < bytes_read; i++) {
        printf("%u   ", read_buffer[i]);
      }
      printf("\n");
    }
    return success;
  }
  uint8_t m_port;
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

enum class state_t
{
  find_beacon,
  backup,
  find_object,
  mission_complete,
};

state_t state = state_t::find_beacon;

void
switch_pressed()
{
  // check state and advance to next step
  if (state == state_t::find_beacon) {
    state = state_t::backup;
  } else if (state == state_t::find_object) {
    state = state_t::mission_complete;
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
      case state_t::find_beacon: {
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
        break;
      }
      case state_t::backup: {
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
        state = state_t::find_object;
        max_power = 25;
        break;
      }
      case state_t::find_object: {
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
      case state_t::mission_complete: {
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
