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

namespace e10 {
class adapter
{
public:
  struct low_freq_data
  {
    uint8_t strongest_photo_diode = 0;
    uint8_t intensity = 0;
  };

  struct high_freq_data
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
    : m_sampling_thread(sampling_thread, this)
    , m_port(p_port)
  {
  }

  low_freq_data get_low_freq_data() { return m_cached_low; }
  high_freq_data get_high_freq_data() { return m_cached_high; }
  camera_data get_cam_data() { return m_cached_camera; }

private:
  template<size_t Length>
  struct request_response
  {
    request_response(bool p_valid)
      : valid(p_valid)
    {
    }
    request_response() {}
    std::array<uint8_t, Length> data{};
    bool valid = false;
  };

  static int sampling_thread(void* p_args)
  {
    auto* self = static_cast<adapter*>(p_args);
    self->sampling_thread_impl();
    return 0;
  }

  int sampling_thread_impl()
  {
    FILE* port = nullptr;

    while (port == nullptr) {
      printf("Opening port %u\n", m_port);

      // Clear everything and wait
      m_cached_low = {};
      m_cached_high = {};
      m_cached_camera = {};

      // Attempt to open port ==================================================

      constexpr char const* path_template = "/dev/port%u";
      // Add 3 for the 3 letters of 256 (max 8-bit value) and + 1 for null
      // character.
      constexpr size_t buffer_length = sizeof(path_template) + 3 + 1;
      std::array<char, 13> port_path{};
      snprintf(port_path.data(), port_path.size(), path_template, m_port);
      port = fopen(port_path.data(), "wb+");

      if (port == nullptr) {
        printf("Opening port %u FAILED!\n", m_port);
        vex::wait(1, seconds);
      }
    }

    while (true) {
      // Camera indicies
      static constexpr auto x_center_low = 0;
      static constexpr auto x_center_hi = 1;
      static constexpr auto y_center_low = 2;
      static constexpr auto y_center_hi = 3;
      static constexpr auto block_width_low = 4;
      static constexpr auto block_width_hi = 5;
      static constexpr auto block_height_low = 6;
      static constexpr auto block_height_hi = 7;

      // Photo diode indicies
      static constexpr auto diode_number = 0;
      static constexpr auto intensity_value = 1;

      // Max response size needed is 9 bytes
      auto const low_buffer = request_data<3>(port, 'l');
      if (low_buffer.valid) {
        m_cached_low.strongest_photo_diode = low_buffer.data[diode_number];
        m_cached_low.intensity = low_buffer.data[intensity_value];
      }

      auto const high_buffer = request_data<3>(port, 'h');
      if (high_buffer.valid) {
        m_cached_high.strongest_photo_diode = high_buffer.data[diode_number];
        m_cached_high.intensity = high_buffer.data[intensity_value];
      };

      auto const cam = request_data<9>(port, 'c');
      if (cam.valid) {
        auto const x = (cam.data[x_center_hi] << 8) | cam.data[x_center_low];
        m_cached_camera.x_center = static_cast<uint16_t>(x);

        auto const y = (cam.data[y_center_hi] << 8) | cam.data[y_center_low];
        m_cached_camera.y_center = static_cast<uint16_t>(y);

        auto const width =
          (cam.data[block_width_hi] << 8) | cam.data[block_width_low];
        m_cached_camera.width = static_cast<uint16_t>(width);

        auto const height =
          (cam.data[block_height_hi] << 8) | cam.data[block_height_low];
        m_cached_camera.height = static_cast<uint16_t>(height);
      }
      this_thread::sleep_for(25); // wait 10ms
    }
    return 0;
  }

  template<size_t ArraySize>
  static request_response<ArraySize> request_data(FILE* p_port, char p_command)
  {
    static_assert(ArraySize >= 3,
                  "ArraySize must be equal to or greater than 3.");
    request_response<ArraySize> response{};

    if (p_port == NULL) {
      printf("Port not open...\n");
      return false;
    }

    auto const bytes_written = fwrite(&p_command, sizeof(p_command), 1, p_port);
    if (bytes_written != sizeof(p_command)) {
      printf("Failed write to port, %zu bytes written.\n", bytes_written);
      return false;
    }

    auto iterator = response.data.begin();
    auto end = response.data.end();

    for (int attempts = 0; attempts < 50 and iterator != end; attempts++) {
      this_thread::sleep_for(1);
      auto const read_length = std::distance(iterator, end);
      auto const bytes_read = fread(iterator, 1, read_length, p_port);
      iterator += bytes_read;
    }

    if (iterator != end) {
      auto start = response.data.begin();
      auto const bytes_remaining = std::distance(start, iterator);
      printf("Failed read from port! %u bytes read. Contents: [",
             bytes_remaining);
      for (; start != iterator + 1; start++) {
        printf("0x%02X, ", *start);
      }
      printf("]\n");
      return false;
    }

    // Calculate Checksum: Use `.end() - 2` so we can be two characters from the
    // last
    uint8_t checksum = 0;
    for (auto start = response.data.begin(); start != (response.data.end() - 2);
         start++) {
      checksum += *start;
    }

    if (checksum != response.data.back()) {
      printf("Checksum mismatch...\n");
      return false;
    }

    return response;
  }

  vex::thread m_sampling_thread;
  low_freq_data m_cached_low;
  high_freq_data m_cached_high;
  camera_data m_cached_camera;
  uint8_t m_port;
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

  // Change this number to the port number you are using on your VEX controller
  static constexpr int port_number = 1;
  e10::adapter board(port_number);

  // TODO(kammce): replace with calls to `pressing()`
  front_bumper.pressed(switch_pressed);

  static constexpr uint8_t threshold = 5;
  static constexpr uint8_t max_power = 40;
  static constexpr size_t print_skip_count = 200;
  int print_skip = 0;

  while (true) {
    switch (state) {
      case mission_state::find_beacon: {
        auto low_freq_data = board.get_low_freq_data();
        int direction = low_freq_data.strongest_photo_diode;

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
        break;
      }
      case mission_state::find_object: {
        auto cam_data = board.get_cam_data();
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
    this_thread::sleep_for(1);

    if (print_skip % print_skip_count == 0) {
      // write to screen
      Brain.Screen.clearScreen();

      auto low_freq_data = board.get_low_freq_data();
      auto cam_data = board.get_cam_data();
      Brain.Screen.printAt(
        10, 20, "Print count: %d", print_skip / print_skip_count);
      Brain.Screen.printAt(10,
                           40,
                           "Low Frequency Beacon: PD %u: %u %d",
                           low_freq_data.strongest_photo_diode,
                           low_freq_data.intensity);
      Brain.Screen.printAt(10,
                           60,
                           "Camera Data: %d, %u (%u X %u)",
                           cam_data.x_center,
                           cam_data.y_center,
                           cam_data.width,
                           cam_data.height);
    }
    print_skip++;
  }
}