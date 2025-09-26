/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       malia                                                     */
/*    Created:      6/7/2025, 10:47:12 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <cstdio>

#include "irb_adapter.h"
#include "vex.h"

using namespace vex;

vex::brain Brain;
vex::motor right_motor = motor(PORT20, ratio18_1, true);
vex::motor left_motor = motor(PORT10, ratio18_1, false);
vex::limit front_bumper = limit(Brain.ThreeWirePort.A);
bool beacon_pressed = false;
bool object_collected = false;

void switch_pressed()
{
  if (!beacon_pressed) {
    beacon_pressed = true;
  } else {
    object_collected = true;
  }
  // stop moving
  right_motor.stop();
  left_motor.stop();
  this_thread::sleep_for(1000);
  // back up for 1 sec
  right_motor.spin(reverse, 40, rpm);
  left_motor.spin(reverse, 40, rpm);
  this_thread::sleep_for(1000);
  // stop again
  right_motor.stop();
  left_motor.stop();
  this_thread::sleep_for(1000);
}

void spin(bool clockwise, uint8_t power)
{
  if (clockwise) {
    right_motor.spin(reverse, power, rpm);
    left_motor.spin(forward, power, rpm);
  } else {
    right_motor.spin(forward, power, rpm);
    left_motor.spin(reverse, power, rpm);
  }
}

int main()
{
  irb_adapter::adapter board(1);
  uint8_t threshold = 5;
  uint8_t power = 40;
  bool spin_clockwise = false;

  front_bumper.pressed(switch_pressed);

  while (1) {
    Brain.Screen.clearScreen();
    // check if bumper pressed
    if (!beacon_pressed) {
      auto low_freq_data = board.get_low_freq_data();
      Brain.Screen.printAt(10,
                           20,
                           "Low Frequency Beacon: PD %u: %u",
                           low_freq_data.low_freq_photo_diode,
                           low_freq_data.low_freq_value);
      Brain.Screen.printAt(10, 40, "Camera Data: waiting...");
      printf("LF %u:%u;\n",
             low_freq_data.low_freq_photo_diode,
             low_freq_data.low_freq_value);

      // spin in place if nothing detected
      if (low_freq_data.low_freq_value < threshold) {
        spin(spin_clockwise, power);
      }
      // turn clockwise if less than 3
      else if (low_freq_data.low_freq_photo_diode < 3) {
        spin_clockwise = true;
        spin(spin_clockwise, power);
      }
      // turn counter clockwise if greater than 4
      else if (low_freq_data.low_freq_photo_diode > 4) {
        spin_clockwise = false;
        spin(spin_clockwise, power);
      }
      // go straight if in middle
      else {
        right_motor.spin(forward, power, rpm);
        left_motor.spin(forward, power, rpm);
      }
    } else if (beacon_pressed && !object_collected) {
      auto cam_data = board.get_cam_data();
      Brain.Screen.printAt(10, 20, "Low Frequency Beacon: Collected");
      Brain.Screen.printAt(10,
                           40,
                           "Camera Data: %d, %u (%u X %u)",
                           cam_data.x_center,
                           cam_data.y_center,
                           cam_data.block_width,
                           cam_data.block_height);
      printf("Camera Data: X:%u Y:%u    %ux%u\n",
             cam_data.x_center,
             cam_data.y_center,
             cam_data.block_width,
             cam_data.block_height);

      // spin in place if nothing detected
      if (cam_data.block_width == 0) {
        spin(spin_clockwise, power);
      }
      // turn clockwise if x more than 155
      else if (cam_data.x_center < 155) {
        spin_clockwise = true;
        spin(spin_clockwise, power);
      }
      // turn counter clockwise if x less than 165
      else if (cam_data.x_center > 165) {
        spin_clockwise = false;
        spin(spin_clockwise, power);
      }
      // go straight if in middle
      else {
        right_motor.spin(forward, power, rpm);
        left_motor.spin(forward, power, rpm);
      }
    } else {
      Brain.Screen.printAt(10, 20, "Low Frequency Beacon: Collected");
      Brain.Screen.printAt(10, 40, "Camera Data: Collected");
    }
  }
}
