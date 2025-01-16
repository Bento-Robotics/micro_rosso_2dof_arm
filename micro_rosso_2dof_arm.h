#ifndef __2DOF_arm_h
#define __2DOF_arm_h
#include <Servo.h>
#include "micro_rosso.h"

const uint32_t linkage_bottom_length_mm = 140;
const uint32_t linkage_top_length_mm = 250;
const uint32_t home_pos_servo_bottom_ms = 2550;
const uint32_t home_pos_servo_top_ms = 2600;


/* servo_top
 *    ↓
 *    O----C
 *    |  ↑linkage_top
 *    | ←linkage_bottom
 *   ̲̲_O̲_ ←servo_bottom
 */
class Two_DOF_Arm
{
public:
  Two_DOF_Arm();
  static bool setup(Servo *servo_bottom, Servo *servo_top,
                    const char *topic_name = "/arm_control",
                    timer_descriptor &timer = micro_rosso::timer_control);

  // errors go to ros2 log
  static void move_arm_absolute(uint32_t x, uint32_t y);
  //static void move_arm_relative(uint32_t x, uint32_t y);

  static void home_arm();
  //static void attach_emergency_stop(); //NOTE ineffective unless we can power down the servos somehow
};

#endif // __2DOF_arm_h
