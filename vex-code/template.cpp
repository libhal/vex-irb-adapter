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

#pragma region IRB Adapter Code

#include <cstdbool>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#include <array>
#include <iterator>

namespace e10 {
class adapter
{
public:
  struct ir_measurement
  {
    uint8_t direction = 0;
    uint8_t intensity = 0;
  };

  struct detected_object
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

  ir_measurement get_low_ir() { return m_cached_low; }
  ir_measurement get_high_ir() { return m_cached_high; }
  detected_object get_detected_object() { return m_cached_camera; }

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
      // Object recognition byte indicies
      static constexpr auto x_center_low = 0;
      static constexpr auto x_center_hi = 1;
      static constexpr auto y_center_low = 2;
      static constexpr auto y_center_hi = 3;
      static constexpr auto block_width_low = 4;
      static constexpr auto block_width_hi = 5;
      static constexpr auto block_height_low = 6;
      static constexpr auto block_height_hi = 7;

      // Photo diode command byte indicies
      static constexpr auto diode_number = 0;
      static constexpr auto intensity_value = 1;

      // --- High Buffer Processing ---
      auto const high_buffer = request_data<sizeof(ir_measurement)>('h');
      if (high_buffer.valid) {
        ir_measurement tmp_high{};
        tmp_high.direction = high_buffer.data[diode_number];
        tmp_high.intensity = high_buffer.data[intensity_value];
        m_cached_high = tmp_high;
      }
      this_thread::sleep_for(10);

      // --- Camera Buffer Processing ---
      auto const cam = request_data<sizeof(detected_object)>('c');
      if (cam.valid) {
        detected_object tmp_cam{};
        tmp_cam.x_center =
          (cam.data[x_center_hi] << 8) | cam.data[x_center_low];
        tmp_cam.y_center =
          (cam.data[y_center_hi] << 8) | cam.data[y_center_low];
        tmp_cam.width =
          (cam.data[block_width_hi] << 8) | cam.data[block_width_low];
        tmp_cam.height =
          (cam.data[block_height_hi] << 8) | cam.data[block_height_low];
        m_cached_camera = tmp_cam;
      }
      this_thread::sleep_for(10);

      // --- Low Buffer Processing ---
      auto const low_buffer = request_data<sizeof(ir_measurement)>('l');
      if (low_buffer.valid) {
        ir_measurement tmp_low;
        tmp_low.direction = low_buffer.data[diode_number];
        tmp_low.intensity = low_buffer.data[intensity_value];
        m_cached_low = tmp_low;
      }
      this_thread::sleep_for(10);
    }
    return 0;
  }

  template<typename Iterator>
  void print_failed_response(char p_command,
                             Iterator p_begin,
                             Iterator p_cursor,
                             Iterator p_end)
  {
    auto const bytes_read = std::distance(p_begin, p_cursor - 1);
    auto const bytes_remaining = std::distance(p_cursor, p_end);
    printf("Command '%c' response failed! %u bytes read. %u bytes remaining \n",
           p_command,
           bytes_read,
           bytes_remaining);
    printf("    Contents: [");
    for (auto index = p_begin; index != p_cursor + 1; index++) {
      printf("0x%02X, ", *index);
    }
    printf("]\n");
  }

  template<size_t ObjectSize>
  request_response<ObjectSize + 1> request_data(char p_command)
  {
    static_assert(ObjectSize >= 1,
                  "ObjectSize must be equal to or greater than 1.");
    request_response<ObjectSize + 1> response{};

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
    auto const end = response.data.end();
    // The last byte of the response is the checksum
    auto const response_checksum = response.data.end() - 1;

    // Attempts cap is set to 100 after measuring (printing) the number of
    // attempts needed to capture all bytes which came out to be between 81
    // and 86.
    for (int attempts = 0; attempts < 100 and iterator != end; attempts++) {
      this_thread::sleep_for(1);
      auto const read_length = std::distance(iterator, end);
      auto const bytes_read = fread(/* address = */ iterator,
                                    /* element_size = */ sizeof(*iterator),
                                    /* element_count = */ read_length,
                                    /* file_stream = */ m_port_file);
      // Advance the iterator `bytes_read` number of elements (bytes)
      iterator += bytes_read;
    }

    // Didn't reach the end of the response buffer
    if (std::distance(iterator, end) != 0) {
      auto const begin = response.data.begin();
      print_failed_response(p_command, begin, iterator, end);
      return false;
    }

    // Calculate the checksum
    uint8_t calculated_checksum = 0;
    for (auto index = response.data.begin(); index != response_checksum;
         index++) {
      calculated_checksum += *index;
    }

    if (calculated_checksum != *response_checksum) {
      auto const begin = response.data.begin();
      printf("Bad Checksum! Calculated: 0x%02X, Received: 0x%02X\n",
             calculated_checksum,
             *response_checksum);
      print_failed_response(p_command, begin, iterator, end);
      return false;
    }

    response.valid = true;
    return response;
  }

  FILE* m_port_file = nullptr;
  vex::thread m_sampling_thread;
  detected_object m_cached_camera{};
  ir_measurement m_cached_high{};
  ir_measurement m_cached_low{};
  uint8_t m_port{};
};
// A simple helper function to keep values within a range
template<typename T>
T
clamp(T value, T min_val, T max_val)
{
  if (value < min_val)
    return min_val;
  if (value > max_val)
    return max_val;
  return value;
}
} // namespace e10

#pragma endregion IRB Adapter Code

#pragma endregion VEXcode Generated Robot Configuration

// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// Counters to reduce how fast the screen is updated
static constexpr size_t print_skip_count = 400;
int print_skip = 0;

// Devices added with codev5.vex.com GUI are defined above in the
// VEXcode Generated Robot Configuration region in the
// "Robot configuration code" section. Devices can also be defined
// manually as seen below.
vex::motor right_motor = motor(PORT20, ratio18_1, false);
vex::motor left_motor = motor(PORT10, ratio18_1, true);
vex::limit front_bumper = limit(Brain.ThreeWirePort.A);

enum class mission_state
{
  goto_beacon,
  turn_off_beacon,
  backup,
  goto_object,
  escape_arena,
  mission_complete,
};

int
main()
{
  Brain.Timer.clear();

  // NOTE: Change port_number to the port on the VEX Brain the E10 Adapter is
  // connected to.
  static constexpr int port_number = 1;
  e10::adapter board(port_number);

  printf("Starting GOTO Beacon!\n");

  static constexpr uint8_t threshold = 5;
  static constexpr float forward_rpm = 20;

  mission_state state = mission_state::goto_beacon;

  while (true) {
    switch (state) {
      case mission_state::goto_beacon: {
        auto const infrared = board.get_low_ir();
        float const direction = infrared.direction;

        // 1. Check bumper
        if (front_bumper.pressing()) {
          state = mission_state::turn_off_beacon;
          break;
        }

        // 2. Spin in place if nothing detected
        if (infrared.intensity < threshold) {
          right_motor.spin(reverse, forward_rpm, rpm);
          left_motor.spin(forward, forward_rpm, rpm);
          break;
        }

        // 3. Calculate error

        // Half way between 3 and 4
        constexpr float center_target = 3.5f;
        // turn_sensitivity controls how aggressively the robot turns when it
        // detects a strong side diode
        constexpr float turn_sensitivity = 0.5f;
        // We map the discrete direction to an error relative to the center of 3
        // & 4. If direction is 3 or 4, error will be -0.5 or +0.5, very turn.
        // If direction is 1, error will be -2.5, large turn.
        float const error = direction - center_target;

        // 4. Tank steering calculation (differential power)
        // We calculate a "steering offset" that shifts power between wheels.
        float const steer_offset = error * (turn_sensitivity * forward_rpm);

        // Left motor gets the base power PLUS the offset
        // Right motor gets the base power MINUS the offset
        float left_rpm = forward_rpm + steer_offset;
        float right_rpm = forward_rpm - steer_offset;

        // 5. Safety clamping
        // Ensure power stays within [0, forward_rpm] to prevent reverse or
        // over-voltage
        left_rpm = e10::clamp(left_rpm, 0.0f, forward_rpm);
        right_rpm = e10::clamp(right_rpm, 0.0f, forward_rpm);

        // 6. Execute
        right_motor.spin(reverse, right_rpm, rpm);
        left_motor.spin(forward, left_rpm, rpm);
        break;
      }
      case mission_state::turn_off_beacon: {
        // STUDENT NOTE: Replace line below with your own code...
        state = mission_state::backup;
        break;
      }
      case mission_state::backup: {
        // STUDENT NOTE: Replace line below with your own code...
        state = mission_state::goto_object;
        break;
      }
      case mission_state::goto_object: {
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
          state = mission_state::escape_arena;
        }
        break;
      }
      case mission_state::escape_arena: {
        // STUDENT NOTE: Suggestion: replace code below put a button on the
        // robot so when it leaves the arena, you can press it to stop it and
        // move the state to "mission_complete"
        state = mission_state::mission_complete;
        break;
      }
      case mission_state::mission_complete: {
        right_motor.stop();
        left_motor.stop();
        float const timer_seconds = Brain.Timer.time(seconds);
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(
          10, 100, "Mission Complete in %.3f seconds!", timer_seconds);
        this_thread::sleep_for(15000);
        Brain.programStop();
      }
      default:
        break;
    }

    if (print_skip % print_skip_count == 0) {
      auto const low_ir = board.get_low_ir();
      auto const high_ir = board.get_high_ir();
      auto const detected_object = board.get_detected_object();
      auto const total_print_count = print_skip / print_skip_count;
      Brain.Screen.printAt(10, 20, " Print count | %d", total_print_count);
      Brain.Screen.printAt(10,
                           40,
                           " 1kHz Beacon | dir: %u, intensity: %03u",
                           low_ir.direction,
                           low_ir.intensity);
      Brain.Screen.printAt(10,
                           60,
                           "10kHz Beacon | dir: %u, intensity: %03u",
                           high_ir.direction,
                           high_ir.intensity);
      Brain.Screen.printAt(10,
                           80,
                           " Camera Data | (x: %03u, y: %03u) (%03u X %03u)  ",
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