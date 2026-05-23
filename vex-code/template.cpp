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
    uint8_t direction = 0;
    uint8_t intensity = 0;
  };

  struct high_freq_data
  {
    uint8_t direction = 0;
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
    , m_sampling_thread(sampling_thread, this)
  {
  }

  low_freq_data get_low_freq_data() { return m_cached_low; }
  high_freq_data get_high_freq_data() { return m_cached_high; }
  camera_data get_detected_object() { return m_cached_camera; }

  ~adapter() { fclose(m_port_file); }

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
    while (m_port_file == nullptr) {
      printf("Opening port %u\n", m_port);

      // Clear everything and wait
      m_cached_low = {};
      m_cached_high = {};
      m_cached_camera = {};

      // Attempt to open port ==================================================

      constexpr char const path_template[] = "/dev/port%u";
      // Add 3 for the 3 letters of 256 (max 8-bit value) and + 1 for null
      // character.
      constexpr size_t buffer_length = sizeof(path_template) + 3 + 1;
      std::array<char, buffer_length> port_path{};
      snprintf(port_path.data(), port_path.size(), path_template, m_port);
      m_port_file = fopen(port_path.data(), "wb+");

      if (m_port_file == nullptr) {
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

      auto const high_buffer = request_data<3>('h');
      if (high_buffer.valid) {
        m_cached_high.direction = high_buffer.data[diode_number];
        m_cached_high.intensity = high_buffer.data[intensity_value];
      };

      this_thread::sleep_for(10);

      auto const low_buffer = request_data<3>('l');
      if (low_buffer.valid) {
        m_cached_low.direction = low_buffer.data[diode_number];
        m_cached_low.intensity = low_buffer.data[intensity_value];
      }

      this_thread::sleep_for(10);

      auto const cam = request_data<9>('c');
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
      this_thread::sleep_for(10);
    }
    return 0;
  }

  template<size_t ArraySize>
  request_response<ArraySize> request_data(char p_command)
  {
    static_assert(ArraySize >= 3,
                  "ArraySize must be equal to or greater than 3.");
    request_response<ArraySize> response{};

    if (m_port_file == NULL) {
      printf("Port not open...\n");
      return false;
    }

    auto const bytes_written =
      fwrite(&p_command, sizeof(p_command), 1, m_port_file);
    if (bytes_written != sizeof(p_command)) {
      printf("Failed write to port, %zu bytes written.\n", bytes_written);
      return false;
    }

    auto iterator = response.data.begin();
    auto end = response.data.end();
    auto back = response.data.end() - 1;

    for (int attempts = 0; attempts < 100 and iterator != end; attempts++) {
      this_thread::sleep_for(1);
      // auto const read_length = std::distance(iterator, end);
      auto const bytes_read = fread(iterator, 1, 1, m_port_file);
      iterator += bytes_read;
    }

    if (std::distance(iterator, end) != 0) {
      auto start = response.data.begin();
      auto const bytes_read = std::distance(start, iterator - 1);
      auto const bytes_remaining = std::distance(iterator, end);
      printf(
        "Command '%c' response failed! %u bytes read. %u bytes remaining \n",
        p_command,
        bytes_read,
        bytes_remaining);
      printf("    Contents: [");
      for (; start != iterator + 1; start++) {
        printf("0x%02X, ", *start);
      }
      printf("]\n");
      return false;
    }

    uint8_t checksum = 0;
    for (auto start = response.data.begin(); start != back; start++) {
      checksum += *start;
    }

    if (checksum != *back) {
      auto start = response.data.begin();
      auto const bytes_read = std::distance(start, iterator);
      auto const bytes_remaining = std::distance(iterator, end);
      printf("Checksum mismatch! Calculated = (%u), Received = (%u) ...\n",
             checksum,
             *back);
      printf(
        "Command '%c' response failed! %u bytes read. %u bytes remaining \n",
        p_command,
        bytes_read,
        bytes_remaining);
      printf("    Contents: [");
      for (; start != iterator + 1; start++) {
        printf("0x%02X, ", *start);
      }
      printf("]\n");
      return false;
    }

    printf("Command '%c' worked\n", p_command);
    printf("    Contents: [");
    for (auto start = response.data.begin(); start != response.data.end();
         start++) {
      printf("0x%02X, ", *start);
    }
    printf("]\n");
    response.valid = true;
    return response;
  }

  uint8_t m_port;
  FILE* m_port_file = nullptr;
  vex::thread m_sampling_thread;
  low_freq_data m_cached_low;
  high_freq_data m_cached_high;
  camera_data m_cached_camera;
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
  turn_off_beacon,
  backup,
  find_object,
  escape_arena,
  mission_complete,
};

mission_state state = mission_state::find_beacon;

int
main()
{
  Brain.Timer.clear();

  // Change this number to the port number you are using on your VEX controller
  static constexpr int port_number = 1;
  e10::adapter board(port_number);

  printf("Starting GOTO Beacon!\n");

  static constexpr uint8_t threshold = 5;
  static constexpr uint8_t max_power = 40;
  static constexpr size_t print_skip_count = 400;
  int print_skip = 0;

  while (true) {
    switch (state) {
      case mission_state::find_beacon: {
        auto const infrared = board.get_low_freq_data();
        int const direction = infrared.direction;

        // spin in place if nothing detected
        if (infrared.intensity < threshold) {
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

        // Check if the front bumper has been pressed. If so, then we must have
        // reached the beacon.
        if (front_bumper.pressing()) {
          state = mission_state::turn_off_beacon;
        }
        break;
      }
      case mission_state::turn_off_beacon: {
        // STUDENT NOTE: Replace line below with your own code...
        state = mission_state::backup;
        break;
      }
      case mission_state::backup: {
        // STUDENT NOTE: Replace line below with your own code...
        state = mission_state::find_object;
        break;
      }
      case mission_state::find_object: {
        auto const detected_object = board.get_detected_object();
        // If width (or height) is 0, then the object has not been detected,
        // then spin around to scan for the object.
        if (detected_object.width == 0) {
          right_motor.spin(forward, 10, rpm);
          left_motor.spin(reverse, 10, rpm);
        }

        // STUDENT NOTE: Add the rest of the code...

        // Check if the front bumper has been pressed. If so, then we must have
        // reached the object.
        if (front_bumper.pressing()) {
          state = mission_state::turn_off_beacon;
        }
        break;
      }
      case mission_state::escape_arena: {
        // STUDENT NOTE: Replace line below with your own code...
        state = mission_state::mission_complete;
        break;
      }
      case mission_state::mission_complete: {
        right_motor.stop();
        left_motor.stop();
        float const timer_seconds = Brain.Timer.time(seconds);
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(
          10, 80, "Mission Complete in %.3f seconds!", timer_seconds);
        this_thread::sleep_for(15000);
        Brain.programStop();
      }
      default:
        break;
    }

    if (print_skip % print_skip_count == 0) {
      // write to screen
      Brain.Screen.clearScreen();

      auto low_freq_data = board.get_low_freq_data();
      auto detected_object = board.get_detected_object();
      Brain.Screen.printAt(
        10, 20, "Print count: %d", print_skip / print_skip_count);
      Brain.Screen.printAt(10,
                           40,
                           "Low Frequency Beacon: PD %u: %u %d",
                           low_freq_data.direction,
                           low_freq_data.intensity);
      Brain.Screen.printAt(10,
                           60,
                           "Camera Data: %d, %u (%u X %u)",
                           detected_object.x_center,
                           detected_object.y_center,
                           detected_object.width,
                           detected_object.height);
    }
    print_skip++;

    // Sleep for 1 millisecond before looping again
    this_thread::sleep_for(1);
  }
}