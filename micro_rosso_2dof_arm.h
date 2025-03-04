#ifndef __2DOF_arm_h
#define __2DOF_arm_h
#include <Servo.h>
#include "micro_rosso.h"
#include "geometry_msgs/msg/point.h"

enum servo_type {
  PRECISE_SERVO_180deg,
  PRECISE_SERVO_270deg
};

// arduino Servo class won't let you read out limits - so here's a wrapper
class Precise_Servo : public Servo {
public:
  // attach the given pin to the next free channel, sets pinMode.
  // returns channel number or 0 if failure.
  int begin(pin_size_t pin, servo_type type = PRECISE_SERVO_180deg);
  // attach the given pin to the next free channel, sets pinMode, min, and max values for write().
  // returns channel number or 0 if failure.
  // angle if <=270, pulse duration if >270
  int begin(pin_size_t pin, int min, int max, servo_type type = PRECISE_SERVO_180deg);
  void write(float value);             // if value is <=270 its treated as an angle, otherwise as pulse width in microseconds
private:
  int _minUs;
  int _maxUs;
  servo_type _type;
};

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
                    uint linkage_bottom_length, uint linkage_top_length,
                    const char *ros_namespace = "/");

  //TODO errors go to ros2 log
  static void move_arm_to(float x, float y);

  static void home_arm();
  //static void attach_emergency_stop(); //NOTE ineffective unless we can power down the servos somehow

private:
  static void report_cb(int64_t last_call_time);
  static void absolute_control_cb(const void* msgin);
  static void relative_control_cb(const void* msgin);
  static void control_loop_cb(int64_t last_call_time);
  static void home_srv_cb(const void* req, void* res);
};

#endif // __2DOF_arm_h
