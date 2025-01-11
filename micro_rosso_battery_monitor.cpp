#include "micro_rosso.h"
#include "micro_rosso_battery_monitor.h"
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/battery_state.h>


static sensor_msgs__msg__BatteryState msg_battery_state;

static publisher_descriptor pdescriptor_battery;


#define RCCHECK(fn)            \
{                              \
  rcl_ret_t temp_rc = fn;      \
  if ((temp_rc != RCL_RET_OK)) \
  {                            \
    return false;              \
  }                            \
}
#define RCNOCHECK(fn)     \
{                         \
  rcl_ret_t temp_rc = fn; \
  (void)temp_rc;          \
}

Basic_Battery::Basic_Battery()
{
  msg_battery_state.header.frame_id = micro_ros_string_utilities_set(msg_battery_state.header.frame_id, "base_link");
  msg_battery_state.voltage = 1.f;
  //TODO ...
};

static void report_cb(int64_t last_call_time)
{
  if (pdescriptor_battery.topic_name != nullptr)
  {
    micro_rosso::set_timestamp(msg_battery_state.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_battery.publisher,
      &msg_battery_state,
      nullptr));
  }
}

bool Basic_Battery::setup(const int analog_pin,
                            const char *topic_name,
                            timer_descriptor &timer) 
{
  if (topic_name != NULL)
  {
    pdescriptor_battery.qos = QOS_BEST_EFFORT;
    pdescriptor_battery.type_support =
      (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(
        sensor_msgs, msg, BatteryState);
    pdescriptor_battery.topic_name = topic_name;
    micro_rosso::publishers.push_back(&pdescriptor_battery);
  }

  D_println("done.");
  return true;
}

