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

#pragma region E10 IRB Adapter Code

#include <cstdbool>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#include <array>
#include <iterator>

namespace e10 {
/**
 * @brief Hardware abstraction adapter for the E10 IRB sensor board.
 *
 * Communicates with the E10 adapter over a VEX smart port serial connection.
 * A background sampling thread continuously polls the board for IR beacon
 * measurements and camera object detections, caching the latest values so
 * callers can read them without blocking on I/O.
 *
 * Construct one instance per physical adapter and call `get_low_ir()`,
 * `get_high_ir()`, or `get_detected_object()` from the main loop.
 */
class adapter
{
public:
  /**
   * @brief A single infrared beacon measurement from one of the IR receivers.
   *
   * Each measurement contains the direction of the strongest detected IR signal
   * (which photo diode fired) and the intensity of that signal. The raw byte
   * array is populated by the adapter's background thread via a serial request
   * to the E10 board.
   */
  struct ir_measurement
  {
    // Photo diode command byte indicies
    static constexpr auto diode_number = 0;
    static constexpr auto intensity_value = 1;
    static constexpr auto max_intensity_mask = static_cast<uint8_t>(~(1U << 7));

    /**
     * @brief Minimum valid direction index (leftmost photo diode).
     * @return uint8_t - always 0
     */
    constexpr uint8_t min_direction() const noexcept { return 0; }

    /**
     * @brief Maximum valid direction index (rightmost photo diode).
     * @return uint8_t - always 7, giving 8 discrete directions (0–7)
     */
    constexpr uint8_t max_direction() const noexcept { return 7; }

    /**
     * @brief Minimum detectable signal intensity.
     * @return uint8_t - always 0 (no signal)
     */
    constexpr uint8_t min_intensity() const noexcept { return 0; }

    /**
     * @brief Maximum detectable signal intensity.
     * @return uint8_t - always 127
     */
    constexpr uint8_t max_intensity() const noexcept
    {
      return max_intensity_mask;
    }

    /**
     * @brief Index of the photo diode that received the strongest IR signal.
     *
     * The value is in the range [min_direction(), max_direction()] where 0 is
     * the leftmost diode and 7 is the rightmost. A value near the center (3–4)
     * means the beacon is approximately straight ahead.
     *
     * @return uint8_t - direction index in [0, 7]
     */
    uint8_t direction() const noexcept { return raw[diode_number]; }

    /**
     * @brief Strength of the detected IR signal.
     *
     * The value is in the range [min_intensity(), max_intensity()]. Values
     * below a small threshold (e.g. 10) typically indicate ambient IR noise
     * rather than a real beacon signal.
     *
     * @return uint8_t - signal intensity in [0, 127]
     */
    uint8_t intensity() const noexcept
    {
      return raw[intensity_value] & max_intensity_mask;
    }

    /**
     * @brief Equality comparison against another measurement.
     * @param other - measurement to compare against
     * @return true if every byte of the raw payload matches
     */
    bool operator==(const ir_measurement& other) const noexcept
    {
      return raw == other.raw;
    }

    std::array<uint8_t, 3> raw{};
  };

  /**
   * @brief Bounding-box data for a single object detected by the camera.
   *
   * The camera frame is 640 × 480 pixels. All coordinate and size values are
   * in pixels, assembled from two-byte little-endian pairs in the raw payload.
   * A `width()` of zero means no object was detected in the current frame.
   */
  struct detected_object
  {
    // Object recognition byte indicies
    static constexpr auto x_center_low = 0;
    static constexpr auto x_center_hi = 1;
    static constexpr auto y_center_low = 2;
    static constexpr auto y_center_hi = 3;
    static constexpr auto block_width_low = 4;
    static constexpr auto block_width_hi = 5;
    static constexpr auto block_height_low = 6;
    static constexpr auto block_height_hi = 7;

    /**
     * @brief Horizontal resolution of the camera sensor in pixels.
     * @return float - always 640
     */
    constexpr float camera_width() const noexcept { return 640; }

    /**
     * @brief Vertical resolution of the camera sensor in pixels.
     * @return float - always 480
     */
    constexpr float camera_height() const noexcept { return 480; }

    /**
     * @brief Horizontal center of the detected object's bounding box.
     * @return uint16_t - x coordinate in pixels, 0 to 639
     */
    uint16_t x_center() const noexcept
    {
      return (raw[x_center_hi] << 8) | raw[x_center_low];
    }

    /**
     * @brief Vertical center of the detected object's bounding box.
     * @return uint16_t - y coordinate in pixels, 0 to 479
     */
    uint16_t y_center() const noexcept
    {
      return (raw[y_center_hi] << 8) | raw[y_center_low];
    }

    /**
     * @brief Width of the detected object's bounding box.
     *
     * A value of 0 indicates that no object was detected in the current frame.
     *
     * @return uint16_t - bounding box width in pixels
     */
    constexpr uint16_t width() const noexcept
    {
      return (raw[block_width_hi] << 8) | raw[block_width_low];
    }

    /**
     * @brief Height of the detected object's bounding box.
     * @return uint16_t - bounding box height in pixels
     */
    constexpr uint16_t height() const noexcept
    {
      return (raw[block_height_hi] << 8) | raw[block_height_low];
    }

    /**
     * @brief Equality comparison against another detected_object.
     * @param other - object to compare against
     * @return true if every byte of the raw payload matches
     */
    bool operator==(const detected_object& other) const
    {
      return raw == other.raw;
    }

    using data_array = std::array<uint8_t, 9>;
    data_array raw{};
  };

  /**
   * @brief Construct an adapter and begin background sampling.
   *
   * Launches an internal VEX thread that repeatedly polls the E10 board over
   * the specified smart port. The thread retries the port open automatically
   * if the initial open fails.
   *
   * @param p_port - VEX smart port number (1–21) the E10 adapter is plugged
   * into
   */
  adapter(uint8_t p_port)
    : m_sampling_thread(sampling_thread, this)
    , m_port(p_port)
  {
  }

  /**
   * @brief Return the latest measurement from the 1 kHz IR receiver.
   *
   * Reads from a cache that is updated by the background sampling thread.
   * The read is performed with a torn-write guard: the value is sampled twice
   * and retried until both reads agree, ensuring a consistent snapshot even
   * though the cache update is not atomic.
   *
   * @return ir_measurement - most recent low-frequency IR beacon measurement
   */
  ir_measurement get_low_ir()
  {
    ir_measurement result1, result2;
    do {
      result1 = m_cached_low;
      result2 = m_cached_low;
    } while (not(result1 == result2));

    return result1;
  }

  /**
   * @brief Return the latest measurement from the 10 kHz IR receiver.
   *
   * Reads from a cache that is updated by the background sampling thread.
   * Uses the same torn-write guard as `get_low_ir()`.
   *
   * @return ir_measurement - most recent high-frequency IR beacon measurement
   */
  ir_measurement get_high_ir()
  {
    ir_measurement result1, result2;
    do {
      result1 = m_cached_high;
      result2 = m_cached_high;
    } while (not(result1 == result2));

    return result1;
  }

  /**
   * @brief Return the latest object detection result from the camera.
   *
   * Reads from a cache that is updated by the background sampling thread.
   * Uses the same torn-write guard as `get_low_ir()`. Check
   * `detected_object::width() == 0` to determine whether any object is
   * currently visible.
   *
   * @return detected_object - most recent camera object detection
   */
  detected_object get_detected_object()
  {
    detected_object result1, result2;
    do {
      result1 = m_cached_camera;
      result2 = m_cached_camera;
    } while (not(result1 == result2));

    return result1;
  }

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
      // =======================================================================
      // --- High Buffer Processing ---
      // =======================================================================
      {
        auto const buffer = request_data<sizeof(ir_measurement)>('h');
        if (buffer.valid) {
          // NOTE: this 3-byte assignment is not atomic. get_high_ir() must
          // double check the result for information tearing before returning
          // the value.
          m_cached_high.raw = buffer.data;
        }
        this_thread::sleep_for(10);
      }

      // =======================================================================
      // --- Camera Buffer Processing ---
      // =======================================================================
      {
        auto const buffer = request_data<sizeof(detected_object)>('c');
        if (buffer.valid) {
          // NOTE: this 9-byte assignment is not atomic. get_detected_object()
          // must double check the result for information tearing before
          // returning the value.
          m_cached_camera.raw = buffer.data;
        }
        this_thread::sleep_for(10);
      }
      // =======================================================================
      // --- Low Buffer Processing ---
      // =======================================================================
      {
        auto const buffer = request_data<sizeof(ir_measurement)>('l');
        if (buffer.valid) {
          // NOTE: this 3-byte assignment is not atomic. get_low_ir() must
          // double check the result for information tearing before returning
          // the value.
          m_cached_low.raw = buffer.data;
        }
        this_thread::sleep_for(10);
      }
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
  request_response<ObjectSize> request_data(char p_command)
  {
    static_assert(ObjectSize >= 2,
                  "ObjectSize must be equal to or greater than 1.");
    request_response<ObjectSize> response{};

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
/**
 * @brief Constrain a value to the closed interval [min_val, max_val].
 *
 * If `value` is less than `min_val`, returns `min_val`. If `value` is greater
 * than `max_val`, returns `max_val`. Otherwise returns `value` unchanged.
 *
 * @tparam T - any type that supports `<` comparison
 * @param value - the value to clamp
 * @param min_val - lower bound of the allowed range (inclusive)
 * @param max_val - upper bound of the allowed range (inclusive)
 * @return T - the clamped value in [min_val, max_val]
 */
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

#pragma endregion E10 IRB Adapter Code

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
motor right_motor = motor(PORT20, ratio18_1, false);
motor left_motor = motor(PORT10, ratio18_1, true);
limit front_bumper = limit(Brain.ThreeWirePort.A);

/**
 * @brief Ordered phases of the robot's autonomous mission.
 *
 * The robot progresses through these states sequentially. Each state is
 * handled by a case in the main control loop's switch statement.
 */
enum class mission_state
{
  /// Navigate toward the IR beacon using the low-frequency IR receiver.
  goto_beacon,
  /// Deactivate the beacon once the front bumper confirms contact.
  turn_off_beacon,
  /// Reverse away from the beacon before searching for the target object.
  backup,
  /// Locate and drive toward the colored object using the camera.
  goto_object,
  /// Drive out of the arena after retrieving or reaching the object.
  escape_arena,
  /// Stop all motors and display the a message such as elapsed time on the
  /// brain screen.
  mission_complete,
};

int
main()
{
  Brain.Timer.clear();
  printf("Starting rescue mission!\n");

  // Start the mission state with goto_beacon
  mission_state state = mission_state::goto_beacon;

  // Make `forward_rpm` higher for higher overall speed and lower for lower
  // speed.
  constexpr float forward_rpm = 30;
  // Change `port_number` to the port on the VEX Brain the E10 Adapter is
  // connected to.
  constexpr int port_number = 1;
  e10::adapter sensor(port_number);

  // Loop the code below forever
  while (true) {
    switch (state) {
      case mission_state::goto_beacon: {
        // =====================================================================
        // 1. Check bumper to determine if the beacon has been reached
        // =====================================================================
        if (front_bumper.pressing()) {
          state = mission_state::turn_off_beacon;
          break;
        }

        // =====================================================================
        // 2. Collect data from sensor
        // =====================================================================
        auto const infrared = sensor.get_low_ir();
        float const direction = infrared.direction();

        // =====================================================================
        // 3. Check if the beacon has been spotted
        // =====================================================================
        // The intensity_threshold is how much of a signal to use to determine
        // if the infrared beacon has been spotted. We need a threshold because
        // the environment contains infrared light of various frequencies that
        // can be picked up by the sensors.
        constexpr uint8_t intensity_threshold = 10;
        if (infrared.intensity() < intensity_threshold) {
          right_motor.spin(reverse, forward_rpm, rpm);
          left_motor.spin(forward, forward_rpm, rpm);
          break;
        }

        // =====================================================================
        // 4. Calculate error
        // =====================================================================
        // IR sensor has 8 photo sensors, numbered 0 to 7. max_direction()
        // return 7. To get the middle we divide the max direction value by 2.
        constexpr float center_target = infrared.max_direction() / 2.0f;
        // turn_sensitivity controls how aggressively the robot turns when it
        // detects a strong side diode. Increase amount to turn harder. Reduce
        // to turn less.
        constexpr float turn_sensitivity = 0.5f;
        // We map the discrete direction to an error relative to the center.
        float const error = direction - center_target;

        // =====================================================================
        // 5. Tank steering calculation (differential power)
        // =====================================================================
        // We calculate a "steering offset" that shifts power between wheels.
        float const steer_offset = (error * turn_sensitivity) * forward_rpm;

        // Left motor gets the base power MINUS the offset
        // Right motor gets the base power PLUS the offset
        float left_rpm = forward_rpm - steer_offset;
        float right_rpm = forward_rpm + steer_offset;

        // =====================================================================
        // 6. Safety clamping
        // =====================================================================
        // Ensure power stays within [0, forward_rpm] to prevent reverse
        left_rpm = e10::clamp(left_rpm, 0.0f, forward_rpm);
        right_rpm = e10::clamp(right_rpm, 0.0f, forward_rpm);

        // =====================================================================
        // 7. Send command to motors
        // =====================================================================
        right_motor.spin(forward, right_rpm, rpm);
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
        this_thread::sleep_for(1000);
        break;
      }
      case mission_state::goto_object: {
        // =====================================================================
        // 1. Check bumper to determine if the object has been reached
        // =====================================================================
        if (front_bumper.pressing()) {
          state = mission_state::escape_arena;
        }

        // =====================================================================
        // 2. Collect data from sensor
        // =====================================================================
        auto const detected_object = sensor.get_detected_object();

        // =====================================================================
        // 3. Spin in place if nothing detected.
        // =====================================================================
        // Simply checking only width is sufficient determine that no object is
        // has been found.
        if (detected_object.width() == 0) {
          // Reducing rotation speed because camera is slower to detect objects.
          right_motor.spin(reverse, forward_rpm / 4, rpm);
          left_motor.spin(forward, forward_rpm / 4, rpm);
          break;
        }

        // STUDENT NOTE: Fill in the rest here.

        break;
      }
      case mission_state::escape_arena: {
        // STUDENT NOTE: Suggestion, put a button on the robot so when it leaves
        // the arena, you can press it to stop it and move the state to
        // "mission_complete". The code to detect that button can be put here.
        state = mission_state::mission_complete;
        break;
      }
      case mission_state::mission_complete: {
        right_motor.stop();
        left_motor.stop();
        float const timer_seconds = Brain.Timer.time(seconds);
        Brain.Screen.printAt(
          10, 100, "Mission Complete in %.3f seconds!", timer_seconds);
        this_thread::sleep_for(15000);
        Brain.programStop();
      }
      default:
        break;
    }

    if (print_skip % print_skip_count == 0) {
      auto const low_ir = sensor.get_low_ir();
      auto const high_ir = sensor.get_high_ir();
      auto const detected_object = sensor.get_detected_object();
      auto const total_print_count = print_skip / print_skip_count;
      Brain.Screen.printAt(10, 20, " Print count | %d", total_print_count);
      Brain.Screen.printAt(10,
                           40,
                           " 1kHz Beacon | dir: %u, intensity: %03u",
                           low_ir.direction(),
                           low_ir.intensity());
      Brain.Screen.printAt(10,
                           60,
                           "10kHz Beacon | dir: %u, intensity: %03u",
                           high_ir.direction(),
                           high_ir.intensity());
      Brain.Screen.printAt(10,
                           80,
                           " Camera Data | (x: %03u, y: %03u) (%03u X %03u)  ",
                           detected_object.x_center(),
                           detected_object.y_center(),
                           detected_object.width(),
                           detected_object.height());
    }
    print_skip++;

    // Sleep for 1 millisecond before looping again
    this_thread::sleep_for(1);
  }
}