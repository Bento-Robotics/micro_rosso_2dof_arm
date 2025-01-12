#include "micro_rosso.h"
#include "sensor_msgs/msg/battery_state.h"
#include "micro_rosso_battery_monitor.h"

#include <cmath>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/battery_state.h>


static sensor_msgs__msg__BatteryState msg_battery_state;

static publisher_descriptor pdescriptor_battery;

static pin_size_t analog_voltage_pin;

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

// battery setup - initialize data
Basic_Battery::Basic_Battery()
{
  // messages come pre-initialized, no need to initialize everything else
  msg_battery_state.header.frame_id = micro_ros_string_utilities_set(msg_battery_state.header.frame_id, "base_link");
  msg_battery_state.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
  msg_battery_state.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  msg_battery_state.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  msg_battery_state.present = true;
  msg_battery_state.location = micro_ros_string_utilities_set(msg_battery_state.location, "body");
  msg_battery_state.serial_number = micro_ros_string_utilities_set(msg_battery_state.serial_number, "custom");
};

// battery topic callback - read battery state
static void report_cb(int64_t last_call_time)
{
  if (pdescriptor_battery.topic_name != 0 ) //TODO && last_reading != reading
  {
    uint analog_value = analogRead(analog_voltage_pin);
    float mapped_value = analog_value / 310.f; // map to 0...3.3v
    msg_battery_state.voltage = mapped_value;

    micro_rosso::set_timestamp(msg_battery_state.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_battery.publisher,
      &msg_battery_state,
      NULL));
  }
}

// set up micro_rosso and analog pin
bool Basic_Battery::setup(const int analog_pin,
                            const char *topic_name,
                            timer_descriptor &timer)
{
  if (topic_name != NULL)
  {
    pdescriptor_battery.qos = QOS_DEFAULT;
    pdescriptor_battery.type_support =
      (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(
        sensor_msgs, msg, BatteryState);
    pdescriptor_battery.topic_name = topic_name;
    micro_rosso::publishers.push_back(&pdescriptor_battery);
  }
  timer.callbacks.push_back(&report_cb);

  analog_voltage_pin = analog_pin;
  pinMode(analog_voltage_pin, INPUT);
  D_println("done.");
  return true;
}

