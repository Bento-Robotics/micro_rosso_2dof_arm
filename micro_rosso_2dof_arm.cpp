#include "micro_rosso_2dof_arm.h"
#include "geometry_msgs/msg/point.h"
#include "micro_rosso.h"
#include "rcl_interfaces/msg/log.h"

#include <geometry_msgs/msg/point.h>
#include <std_srvs/srv/trigger.h>

static subscriber_descriptor sdescriptor_status_msg;
static geometry_msgs__msg__Point msg_arm_position;

static service_descriptor srvdescriptor_home_srv;
std_srvs__srv__Trigger_Request home_srv_request;
std_srvs__srv__Trigger_Response home_srv_response;

static Servo *servo_top;
static Servo *servo_bottom;

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      return false;                                                            \
    }                                                                          \
  }
#define RCNOCHECK(fn)                                                          \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    (void)temp_rc;                                                             \
  }

// arm setup - initialize data and home arm
Two_DOF_Arm::Two_DOF_Arm() {
  // messages come pre-initialized, no need to initialize everything else
  home_arm(); // this may be suboptimal, but servos are unidirectional
              // communicators so we have no idea where they are.
};

// battery topic callback - read battery state
static void control_cb(const void *msgin) {
  if (sdescriptor_status_msg.topic_name != NULL) {
    Two_DOF_Arm::move_arm_absolute(((geometry_msgs__msg__Point *)msgin)->x,
                                   ((geometry_msgs__msg__Point *)msgin)->y);
  }
}

static void home_srv_cb(const void *req, void *res) {
  std_srvs__srv__Trigger_Response* response = (std_srvs__srv__Trigger_Response*) res;
  response->success = true;
  Two_DOF_Arm::home_arm();
}

// set up micro_rosso and analog pin
bool Two_DOF_Arm::setup(Servo *servo_bottom, Servo *servo_top,
                        const char *topic_name, timer_descriptor &timer) {
  if (topic_name != NULL) {
    sdescriptor_status_msg.type_support =
        (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(
            geometry_msgs, msg, Point);
    sdescriptor_status_msg.topic_name = topic_name;
    sdescriptor_status_msg.msg = &msg_arm_position;
    sdescriptor_status_msg.callback = &control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_status_msg);

    srvdescriptor_home_srv.qos = QOS_DEFAULT;
    srvdescriptor_home_srv.type_support =
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
    srvdescriptor_home_srv.service_name = "/home_arm";
    srvdescriptor_home_srv.request = &home_srv_request;
    srvdescriptor_home_srv.response = &home_srv_response;
    srvdescriptor_home_srv.callback = &home_srv_cb;
    micro_rosso::services.push_back(&srvdescriptor_home_srv);
  }

  D_println("done.");
  return true;
}

void Two_DOF_Arm::move_arm_absolute(uint32_t x, uint32_t y) {

  const float c2 = sq(x) + sq(y);

  const float angle_top =
      atan2(y, x) +
      acos((sq(linkage_top_length_mm) + c2 - sq(linkage_bottom_length_mm)) /
           (2 * linkage_top_length_mm * sqrt(c2)));
  const float angle_bottom =
      acos((sq(linkage_top_length_mm) + sq(linkage_bottom_length_mm) - c2) /
           (2 * linkage_top_length_mm * linkage_bottom_length_mm));
  float angle_top_deg = (RAD_TO_DEG * angle_top);
  if (std::isnormal(angle_top_deg)) {
    servo_top->writeMicroseconds(1150 + (1350 / PI) * angle_top);
    char massage[100];
    sprintf(massage, "top DEG: %f, RAD: %f", angle_top_deg, angle_top);
    Logger::log(massage, "micro_rosso_2dof_arm", "move_arm_absolute", __LINE__,
                rcl_interfaces__msg__Log__INFO);
  } else {
    Logger::log("requested arm position out-of-bounds, ignoring",
                "micro_rosso_2dof_arm", "move_arm_absolute", __LINE__,
                rcl_interfaces__msg__Log__WARN);
    return;
  }

  float angle_bottom_deg = (RAD_TO_DEG * angle_bottom);
  if (std::isnormal(angle_bottom_deg)) {
    servo_bottom->writeMicroseconds(2450 - (1350 / PI) * angle_bottom);
    char massage[100];
    sprintf(massage, "top DEG: %f, RAD: %f", angle_top_deg, angle_top);
    Logger::log(massage, "micro_rosso_2dof_arm", "move_arm_absolute", __LINE__,
                rcl_interfaces__msg__Log__INFO);
  } else {
    Logger::log("requested arm position out-of-bounds, ignoring",
                "micro_rosso_2dof_arm", "move_arm_absolute", __LINE__,
                rcl_interfaces__msg__Log__WARN);
    return;
  }
}

void Two_DOF_Arm::home_arm() {
  servo_top->writeMicroseconds(home_pos_servo_top_ms);
  // delay(1); NOTE there was a bug here with the last revision, maybe this will
  // help
  servo_bottom->writeMicroseconds(home_pos_servo_bottom_ms);
}
