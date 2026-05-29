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
 * Construct one instance per physical adapter and call `measure_1kHz()`,
 * `measure_10kHz()`, or `get_detected_object()` from the main loop.
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
     * @return int - always 0
     */
    constexpr int min_direction() const noexcept { return 0; }

    /**
     * @brief Maximum valid direction index (rightmost photo diode).
     * @return int - always 7, giving 8 discrete directions (0–7)
     */
    constexpr int max_direction() const noexcept { return 7; }

    /**
     * @brief Minimum detectable signal intensity.
     * @return int - always 0 (no signal)
     */
    constexpr int min_intensity() const noexcept { return 0; }

    /**
     * @brief Maximum detectable signal intensity.
     * @return int - always 127
     */
    constexpr int max_intensity() const noexcept { return max_intensity_mask; }

    /**
     * @brief Index of the photo diode that received the strongest IR signal.
     *
     * The value is in the range [min_direction(), max_direction()] where 0 is
     * the leftmost diode and 7 is the rightmost. A value near the center (3–4)
     * means the beacon is approximately straight ahead.
     *
     * @return int - direction index in [0, 7]
     */
    int direction() const noexcept { return raw[diode_number]; }

    /**
     * @brief Strength of the detected IR signal.
     *
     * The value is in the range [min_intensity(), max_intensity()]. Values
     * below a small threshold (e.g. 10) typically indicate ambient IR noise
     * rather than a real beacon signal.
     *
     * @return int - signal intensity in [0, 127]
     */
    int intensity() const noexcept
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
     * @return int - x coordinate in pixels, 0 to 639
     */
    int x_center() const noexcept
    {
      return (raw[x_center_hi] << 8) | raw[x_center_low];
    }

    /**
     * @brief Vertical center of the detected object's bounding box.
     * @return int - y coordinate in pixels, 0 to 479
     */
    int y_center() const noexcept
    {
      return (raw[y_center_hi] << 8) | raw[y_center_low];
    }

    /**
     * @brief Width of the detected object's bounding box.
     *
     * A value of 0 indicates that no object was detected in the current frame.
     *
     * @return int - bounding box width in pixels
     */
    constexpr int width() const noexcept
    {
      return (raw[block_width_hi] << 8) | raw[block_width_low];
    }

    /**
     * @brief Height of the detected object's bounding box.
     * @return int - bounding box height in pixels
     */
    constexpr int height() const noexcept
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
  ir_measurement measure_1kHz()
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
   * Uses the same torn-write guard as `measure_1kHz()`.
   *
   * @return ir_measurement - most recent high-frequency IR beacon measurement
   */
  ir_measurement measure_10kHz()
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
   * Uses the same torn-write guard as `measure_1kHz()`. Check
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
          // NOTE: this 3-byte assignment is not atomic. measure_10kHz() must
          // double check the result for information tearing before returning
          // the value.
          m_cached_high.raw = buffer.data;
        }
        vex::wait(10, msec);
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
        vex::wait(10, msec);
      }
      // =======================================================================
      // --- Low Buffer Processing ---
      // =======================================================================
      {
        auto const buffer = request_data<sizeof(ir_measurement)>('l');
        if (buffer.valid) {
          // NOTE: this 3-byte assignment is not atomic. measure_1kHz() must
          // double check the result for information tearing before returning
          // the value.
          m_cached_low.raw = buffer.data;
        }
        vex::wait(10, msec);
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
      vex::wait(1, msec);
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

/**
 * @brief Ordered phases of the robot's autonomous mission.
 *
 * The robot progresses through these states sequentially. Each state is
 * handled by a case in the main control loop's switch statement.
 */
enum class mission_state
{
  /// Locate and drive toward to the object using the camera.
  goto_object,
  /// Reverse away from the object before searching for the infrared beacon
  backup,
  /// Navigate toward the infrared beacon
  goto_beacon,
  /// Deactivate the beacon once contact with it is confirmed.
  turn_off_beacon,
  /// Drive out of the arena after retrieving or reaching the object.
  escape_arena,
};

/**
 * @brief Get a string presenting the mission state
 *
 * @param p_state - state to get the string of
 * @return char const* - string representing the mission state.
 */
char const*
to_string(mission_state p_state)
{
  switch (p_state) {
    case mission_state::goto_object:
      return "goto_object";
    case mission_state::backup:
      return "backup";
    case mission_state::goto_beacon:
      return "goto_beacon";
    case mission_state::turn_off_beacon:
      return "turn_off_beacon";
    case mission_state::escape_arena:
      return "escape_arena";
  }
}

/**
 * @brief Print out data pertaining to the e10::adapter
 *
 * This function prints out a message if it is called, if at least 100ms has
 * passed since it was last called OR the mission state that is provided has
 * changed from the last state that was passed
 *
 * @param p_sensor - e10 adapter to get sensor information from
 * @param p_current_state - mission state to print to the console
 */
void
print_sensor_data(e10::adapter& p_sensor, mission_state p_current_state)
{

  // Previous mission control state to be compared to the current state that is
  // passed to print_sensor_data
  static mission_state previous_state = e10::mission_state::goto_object;

  // Amount of time to wait before printing (in milliseconds)
  static unsigned constexpr print_period_ms = 100;

  // This timer is used to determine when the screen should be updated again.
  static vex::timer print_timer;

  // Holds the number of times a print has occurred. This is a helpful
  // diagnostic which can be used from the programmer to determine if their code
  // is stuck in an infinite loop somewhere in the state machine. If that is the
  // case, then the print function won't happen and the print counter on the
  // screen will be locked to the previous value that was printed. When this
  // occurs, thats a hint to check the code for places where it may be stuck.
  static unsigned print_counter = 0;

  if (print_timer.time(msec) > print_period_ms or
      p_current_state != previous_state) {
    // Extract measurements from IRB and camera
    auto const low_ir = p_sensor.measure_1kHz();
    auto const high_ir = p_sensor.measure_10kHz();
    auto const detected_object = p_sensor.get_detected_object();
    // NOTE: The text, when rendered on the screen, has a height of 20 pixels.
    // This means that the x-position of the text should be in multiples of 20.
    Brain.Screen.printAt(10, 20, " Print count | %u", print_counter);
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
    Brain.Screen.printAt(10,
                         100,
                         "       State | (%d) %-15s",
                         static_cast<int>(p_current_state),
                         to_string(p_current_state));

    // Reset the print timer back to 0 and wait for it to climb above 100ms
    // before we print again.
    print_timer.clear();
    // Increment the print timer to indicate that a print has occurred.
    print_counter++;
    // Record the current state to match against the next call of this function
    previous_state = p_current_state;
  }
}
} // namespace e10

#pragma endregion E10 IRB Adapter Code

#pragma endregion VEXcode Generated Robot Configuration

// ‼️‼️‼️ WARNING: low level driver code above, DO NOT TOUCH ☠️☠️☠️

int
main()
{
  // 💡 TIP: This will print the message below within the V5 print console.
  // See the "speed gauge" picture below the "start" and "stop"
  // buttons on the upper right hand size of the interface. You can
  // use these print messages to log information at different points
  // in the program.
  printf("Starting mission!\n");

  // Create motor objects to for the right and left motor
  //    1st input parameter is the port number
  //    2nd parameter is the gear ratio of the motor
  //    3rd parameter determines if the motor should move in reverse
  motor right_motor = motor(PORT10, ratio18_1, false);
  motor left_motor = motor(PORT20, ratio18_1, true);

  // See VEX API docs here for more details:
  // https://api.vex.com/v5/home/cpp/Motors_and_MotorControllers/motor_and_motor_group.html

  // Create a limit switch connected to legacy port A, which is on the side of
  // the brain 5 computer.
  limit front_limit_switch = limit(Brain.ThreeWirePort.A);

  // The mission state controls which phase of the mission the robot is
  // currently executing. The robot progresses through these states in order:
  //
  //   e10::mission_state::goto_object
  //     - Locate and drive toward the object using the camera.
  //   e10::mission_state::backup
  //     - Reverse away from the object before finding the beacon.
  //   e10::mission_state::goto_beacon
  //     - Navigate toward the infrared beacon.
  //   e10::mission_state::turn_off_beacon
  //     - Deactivate the beacon once contact is confirmed.
  //   e10::mission_state::escape_arena
  //     - Drive out of the arena.
  //
  // For testing, modify the starting state here to begin the mission at a
  // different phase.
  e10::mission_state state = e10::mission_state::goto_object;

  // Change `port_number` to the port on the VEX Brain the E10 Adapter is
  // connected to.
  constexpr int port_number = 1;
  e10::adapter sensor(port_number);

  // Loop the code below forever
  while (true) {
    // Prints sensor information from the e10 sensor to the touch screen.
    // 💡 TIP: this can be removed if students
    print_sensor_data(sensor, state);

    switch (state) {
      case e10::mission_state::goto_object: {
        // =====================================================================
        // 1. Check limit switch to determine if the object has been reached
        // =====================================================================
        if (front_limit_switch.pressing()) {
          state = e10::mission_state::backup;
          break;
        }

        // =====================================================================
        // 2. Define speed parameters
        // =====================================================================
        // These define the speed of rotation and the speed of forward approach.
        // Feel free to modify these values to get the robot to spin/move slower
        // or faster. `spin_rpm` is slow because the camera is slower to detect
        // objects than it is to keep track of objects.
        float const spin_rpm = 10.0f;
        float const forward_rpm = 35.0f;

        // =====================================================================
        // 3. Collect data from sensor
        // =====================================================================
        auto const detected_object = sensor.get_detected_object();

        // =====================================================================
        // 4. Spin robot in place if nothing detected
        // =====================================================================
        // Simply checking only width() is sufficient determine that no object
        // has been found.
        if (detected_object.width() == 0) {
          // Here we set the directions of the motors in opposite directions to
          // get our robot chassis to rotate/spin about its center.
          right_motor.spin(reverse, spin_rpm, rpm);
          left_motor.spin(forward, spin_rpm, rpm);
          break;
        }

        // =====================================================================
        // 5. Calculate error
        // =====================================================================
        // The camera frame is 640 pixels wide. Dividing by 2 gives us the
        // horizontal center (320) that we steer toward.
        float const center_target = detected_object.camera_width() / 2.0f;
        // turn_sensitivity controls how aggressively the robot turns when the
        // object is off-center. Increase to turn harder, reduce to turn less.
        float const turn_sensitivity = 0.005f;
        // Calculate how far the object is from the camera center in pixels.
        float const error = detected_object.x_center() - center_target;

        // =====================================================================
        // 6. Tank steering calculation (differential power)
        // =====================================================================
        // We calculate a "steering offset" that shifts power between wheels.
        float const steer_offset = (error * turn_sensitivity) * forward_rpm;

        // Left motor gets the base power PLUS the offset
        // Right motor gets the base power MINUS the offset
        float left_rpm = forward_rpm + steer_offset;
        float right_rpm = forward_rpm - steer_offset;

        // =====================================================================
        // 7. Safety clamping
        // =====================================================================
        // Ensure RPM stays within [0, forward_rpm]
        left_rpm = e10::clamp(left_rpm, 0.0f, forward_rpm);
        right_rpm = e10::clamp(right_rpm, 0.0f, forward_rpm);

        // =====================================================================
        // 8. Send command to motors
        // =====================================================================
        left_motor.spin(forward, left_rpm, rpm);
        right_motor.spin(forward, right_rpm, rpm);
        break;
      }
      case e10::mission_state::backup: {
        // Stop the motors for a 1 second
        right_motor.stop();
        left_motor.stop();
        wait(1, seconds);

        // Backup for 2 seconds
        auto const backup_rpm = 40;
        left_motor.spin(reverse, backup_rpm, rpm);
        right_motor.spin(reverse, backup_rpm, rpm);
        wait(2, seconds);

        state = e10::mission_state::goto_beacon;
        break;
      }
      case e10::mission_state::goto_beacon: {
        // =====================================================================
        // 1. Check limit switch to determine if the beacon has been reached
        // =====================================================================
        if (front_limit_switch.pressing()) {
          state = e10::mission_state::turn_off_beacon;
          break;
        }

        // =====================================================================
        // 2. Define speed parameters
        // =====================================================================
        // These define the speed of rotation and the speed of forward approach.
        // Feel free to modify these values to get the robot to spin/move slower
        // or faster. `spin_rpm` is faster than for `goto_object` because the IR
        // sensor can detect strong infrared signals very quickly.
        float const spin_rpm = 30.0f;
        float const forward_rpm = 25.0f;

        // =====================================================================
        // 3. Collect data from sensor
        // =====================================================================
        // Get a measurement of infrared sensor for the 10kHz signal
        auto const infrared = sensor.measure_10kHz();

        // =====================================================================
        // 4. Check if the beacon has been spotted
        // =====================================================================
        // The beacon_detection_level is how much of a signal to use to
        // determine if the infrared beacon has been spotted. We need a
        // threshold because the environment contains infrared light pulses of
        // various frequencies that can be picked up by the sensors.
        //
        // 💡 TIP TO STUDENTS: In general, this value should work, but it may be
        // adjusted up or down to help with navigation to the object. If it's
        // too low, then environmental noise will be picked up by the sensor and
        // the robot may navigate to something that's emitting infrared light
        // but isn't the beacon. If this value is too large, the robot will not
        // detect the beacon from further away and will continue to rotate.
        uint8_t const beacon_detection_level = 10;
        if (infrared.intensity() < beacon_detection_level) {
          // Here we set the directions of the motors in opposite directions to
          // get our robot chassis to rotate/spin about its center.
          right_motor.spin(reverse, spin_rpm, rpm);
          left_motor.spin(forward, spin_rpm, rpm);
          break;
        }

        // =====================================================================
        // 5. Calculate error
        // =====================================================================
        // Use infrared diode max_direction to determine center line of which
        // should be 3.5, between 3 and 4
        float const center_target = infrared.max_direction() / 2.0f;
        // Since the directions are coarse (only 8 distinct directions), the
        // turn sensitivity needs to be dialed up by a factor of 100.
        float const turn_sensitivity = 0.5f;
        // Calculate how far the object is from the diode center.
        float const error = infrared.direction() - center_target;

        // =====================================================================
        // 6. Tank steering calculation (differential power)
        // =====================================================================
        // We calculate a "steering offset" that shifts power between wheels.
        float const steer_offset = (error * turn_sensitivity) * forward_rpm;

        // Compared to the camera, the left and right rpm values have their
        // offset sign changed. This is needed because the 0 value is on the
        // left with the camera and on the right for the diodes.
        float left_rpm = forward_rpm - steer_offset;
        float right_rpm = forward_rpm + steer_offset;

        // =====================================================================
        // 7. Safety clamping
        // =====================================================================
        // Ensure power stays within [0, forward_rpm] to prevent reverse
        left_rpm = e10::clamp(left_rpm, 0.0f, forward_rpm);
        right_rpm = e10::clamp(right_rpm, 0.0f, forward_rpm);

        // =====================================================================
        // 8. Send command to motors
        // =====================================================================
        left_motor.spin(forward, left_rpm, rpm);
        right_motor.spin(forward, right_rpm, rpm);
        break;
      }
      case e10::mission_state::turn_off_beacon: {
        // =====================================================================
        // 1. Stop motors
        // =====================================================================
        right_motor.stop();
        left_motor.stop();
        // Wait 2 seconds before using the arm to turn off the beacon
        wait(2, seconds);

        // =====================================================================
        // 2. Turn off beacon with robot arm
        // =====================================================================
        // ✏️ TO STUDENTS: add your code here to turn off beacon with robot arm

        // =====================================================================
        // 3. Determine if the beacon has been turned off
        // =====================================================================
        auto const infrared = sensor.measure_10kHz();

        // 💡 TIP for STUDENTS: In general, this value should work, but it may
        // be adjusted up or down to help with determining if the beacon has
        // been turned off.
        //
        // If this value is set too low, then environmental noise may be picked
        // up by the sensor and the robot may navigate to something that's
        // emitting infrared light but isn't the beacon. This would result in a
        // false negative, where the robot thinks the beacon is still on, due to
        // the environmental noise, but its actually off.
        //
        // If this value is too large, the robot may think that the beacon is
        // off when in reality, the beacon is still on but its signal strength
        // is below the detection level.
        uint8_t const beacon_detection_level = 10;
        // If the intensity is above the ambient infrared light amount in the
        // room, then the beacon must still be on.
        if (infrared.intensity() <= beacon_detection_level) {
          // Beacon has been turned off, now its time to capture and escape
          // from the arena with the beacon.
          state = e10::mission_state::escape_arena;
          break;
        }

        // Go back to the backup state to backup from the beacon, then proceed
        // to the goto_beacon state.
        state = e10::mission_state::backup;
        break;
      }
      case e10::mission_state::escape_arena: {
        // ✏️ TO STUDENTS: Here you will the code needed to escape the arena.
        break;
      }
    }

    // Wait for 1 millisecond before looping again. This gives time for the
    // motors to move and hardware to update before we command them again.
    wait(1, msec);
  }
}
