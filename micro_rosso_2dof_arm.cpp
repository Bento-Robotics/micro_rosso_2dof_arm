#include "micro_rosso_2dof_arm.h"
#include "micro_rosso.h"
#include "rcl/types.h"
#include "rcl_interfaces/msg/log.h"
#include "rmw/types.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include <micro_ros_utilities/string_utilities.h>

#include <geometry_msgs/msg/point.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_srvs/srv/trigger.h>
#include <rcl_interfaces/msg/log.h>

static subscriber_descriptor sdescriptor_arm_absolute_control;
static subscriber_descriptor sdescriptor_arm_relative_control;
static geometry_msgs__msg__Point msg_arm_position;

static publisher_descriptor pdescriptor_joint_states;
static sensor_msgs__msg__JointState msg_joint_states;

static service_descriptor srvdescriptor_home_srv;
std_srvs__srv__Trigger_Request home_srv_request;
std_srvs__srv__Trigger_Response home_srv_response;

static publisher_descriptor pdescriptor_log;
static rcl_interfaces__msg__Log msg_log;


static Servo* _servo_top;
static Servo* _servo_bottom;

static geometry_msgs__msg__Point _goal_pos;
static geometry_msgs__msg__Point _actual_pos;
static double _actual_angles[2] = {0.0, 0.0};

static uint _linkage_bottom_length;
static uint _linkage_top_length;

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

void ros_log(char* string, uint log_level = rcl_interfaces__msg__Log__DEBUG) {
  if (pdescriptor_log.topic_name != 0 && sizeof(string) != 0) //TODO && last_reading != reading
  {
    micro_rosso::set_timestamp(msg_log.stamp);
    msg_log.level = log_level;
    msg_log.name = micro_ros_string_utilities_set(msg_log.name, "micro_rosso_2dof_arm");
    msg_log.msg = micro_ros_string_utilities_set(msg_log.msg, string);

    RCNOCHECK(rcl_publish(
      &pdescriptor_log.publisher,
      &msg_log,
      NULL));

    // msg_log = ; // empty message
  }

}

// arm state topic callback
void Two_DOF_Arm::report_cb(int64_t last_call_time)
{
  if (pdescriptor_joint_states.topic_name != 0 ) //TODO && last_reading != reading
  {
    msg_joint_states.position.data[0] = _actual_angles[0];
    msg_joint_states.position.data[1] = _actual_angles[1];

    micro_rosso::set_timestamp(msg_joint_states.header.stamp);
    digitalWrite(LED_BUILTIN, RCL_RET_OK == rcl_publish(
      &pdescriptor_joint_states.publisher,
      &msg_joint_states,
      NULL));

  char string[50];
  sprintf(string, "joint state: 0: %d, 1: %d", msg_joint_states.position.data[0], msg_joint_states.position.data[1]);
  ros_log(string);
  }
}

void Two_DOF_Arm::absolute_control_cb(const void* msgin) {
  // static float oldx, oldy;
  // if (sdescriptor_arm_absolute_control.topic_name != NULL && (((geometry_msgs__msg__Point *)msgin)->x != oldx || ((geometry_msgs__msg__Point *)msgin)->y != oldy)) {
  if (sdescriptor_arm_absolute_control.topic_name != NULL) {
    _goal_pos.x = ((geometry_msgs__msg__Point *)msgin)->x;
    _goal_pos.y = ((geometry_msgs__msg__Point *)msgin)->y;
  }
  char string[50];
  sprintf(string, "absolute_control: x: %f, y: %f", _goal_pos.x, _goal_pos.y);
  ros_log(string);
}

void Two_DOF_Arm::relative_control_cb(const void* msgin) {
  // static float oldx, oldy;
  // if (sdescriptor_arm_absolute_control.topic_name != NULL && (((geometry_msgs__msg__Point *)msgin)->x != oldx || ((geometry_msgs__msg__Point *)msgin)->y != oldy)) {
  if (sdescriptor_arm_absolute_control.topic_name != NULL) {
    _goal_pos.x += ((geometry_msgs__msg__Point *)msgin)->x;
    _goal_pos.y += ((geometry_msgs__msg__Point *)msgin)->y;
  }
  char string[50];
  sprintf(string, "relative_control: x: %f, y: %f", _goal_pos.x, _goal_pos.y);
  ros_log(string);
}

void Two_DOF_Arm::control_loop_cb(int64_t last_call_time) {
  // TODO PID
  // TODO motion smoothing
  const float kp = 0.25;
  auto controller = [kp](double* actual_pos, double goal_pos) -> void {
    (*actual_pos) += (goal_pos - (*actual_pos)) * kp;
  };

  if (_goal_pos.x != _actual_pos.x) {
    controller(&_actual_pos.x, _goal_pos.x);
  }
  if (_goal_pos.y != _actual_pos.y) {
    controller(&_actual_pos.y, _goal_pos.y);
  }
  // 2dof arm has no z axis

  move_arm_to(_actual_pos.x, _actual_pos.y);
}

void Two_DOF_Arm::home_srv_cb(const void *req, void *res) {
  std_srvs__srv__Trigger_Response* response = (std_srvs__srv__Trigger_Response*) res;
  home_arm();
  response->success = true;
}


// arm setup - initialize data and home arm
Two_DOF_Arm::Two_DOF_Arm() {
  msg_joint_states.name.size = 2;
  msg_joint_states.name.capacity= 2;
  msg_joint_states.position.data = (double *) malloc(2 * sizeof(double));
  msg_joint_states.position.size = 2;
  msg_joint_states.position.capacity = 2;

  msg_joint_states.name.data[0] = micro_ros_string_utilities_set(msg_joint_states.name.data[0], "arm_base_joint");
  msg_joint_states.name.data[1] = micro_ros_string_utilities_set(msg_joint_states.name.data[1], "arm_elbow_joint");
  // messages come pre-initialized, no need to initialize everything else
};

// set up micro_rosso and analog pin
bool Two_DOF_Arm::setup(Servo *servo_bottom, Servo *servo_top,
                    uint linkage_bottom_length, uint linkage_top_length,
                    const char *ros_namespace){
  //TODO namespace functionality
  if (ros_namespace != NULL) {
    sdescriptor_arm_absolute_control.type_support =
      (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(
        geometry_msgs, msg, Point);
    sdescriptor_arm_absolute_control.topic_name = "/arm_control_absolute";
    sdescriptor_arm_absolute_control.msg = &msg_arm_position;
    sdescriptor_arm_absolute_control.callback = &absolute_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_arm_absolute_control);

    //NOTE both share the same message storage, don't know if that's gonna become an issue
    sdescriptor_arm_relative_control.type_support =
      (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(
        geometry_msgs, msg, Point);
    sdescriptor_arm_relative_control.topic_name = "/arm_control_relative";
    sdescriptor_arm_relative_control.msg = &msg_arm_position;
    sdescriptor_arm_relative_control.callback = &relative_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_arm_relative_control);

    srvdescriptor_home_srv.qos = QOS_DEFAULT;
    srvdescriptor_home_srv.type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
    srvdescriptor_home_srv.service_name = "/home_arm";
    srvdescriptor_home_srv.request = &home_srv_request;
    srvdescriptor_home_srv.response = &home_srv_response;
    srvdescriptor_home_srv.callback = &home_srv_cb;
    micro_rosso::services.push_back(&srvdescriptor_home_srv);

    pdescriptor_joint_states.qos = QOS_DEFAULT;
    pdescriptor_joint_states.type_support =
      (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
    pdescriptor_joint_states.topic_name = "/joint_states";
    micro_rosso::publishers.push_back(&pdescriptor_joint_states);

    pdescriptor_log.qos = QOS_DEFAULT; //TODO qos mismatch
    pdescriptor_log.type_support =
      (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log);
    pdescriptor_log.topic_name = "/rosout";
    micro_rosso::publishers.push_back(&pdescriptor_log);

    micro_rosso::timer_control.callbacks.push_back(control_loop_cb);
    micro_rosso::timer_report.callbacks.push_back(report_cb);
  }
  _servo_bottom = servo_bottom;
  _servo_top = servo_top;
  _linkage_bottom_length = linkage_bottom_length;
  _linkage_top_length = linkage_top_length;

  home_arm(); // the sudden movement is suboptimal, but servos are unidirectional communicators so we have no idea where they are.

  D_println("done.");
  return true;
}

void Two_DOF_Arm::move_arm_to(float x, float y) {  
  const float c2 = sq(x) + sq(y);

  const float angle_top = atan2(y,  x) + acos(
    (sq(_linkage_bottom_length) + c2 - sq(_linkage_top_length)) /
    (2 * _linkage_bottom_length * sqrt(c2))
  );
  const float angle_bottom = acos(
    (sq(_linkage_bottom_length) + sq(_linkage_top_length) - c2) / 
    (2 * _linkage_bottom_length * _linkage_top_length)
  );

  const float angle_top_deg = angle_top * RAD_TO_DEG;
  const float angle_bottom_deg = angle_bottom * RAD_TO_DEG;

  if (std::isnormal(angle_top_deg) && std::isnormal(angle_bottom_deg)) {
    _servo_top->write(angle_top_deg); // _pos_goal_x = scorcery1;}
    _servo_bottom->write(180.f - angle_bottom_deg); // _pos_goal_y = scorcery2;}
  }

  _actual_angles[0] = angle_bottom;
  _actual_angles[1] = angle_top;

  char string[50];
  sprintf(string, "control: ϑ_top: %f, ϑ_bottom: %f", angle_top_deg, angle_bottom_deg);
  ros_log(string);
}

void Two_DOF_Arm::home_arm() {
  // joint angles are both 0deg => y=0
  _goal_pos.y = 0.1; // misbehaves with 0.0
  move_arm_to(_goal_pos.x, _goal_pos.y);

  // wait for arm to move, else it could hit the bottom servo/whatever it's mounted to
  delay(200);

  // x position ≙ size of linkages
  _goal_pos.x = (_linkage_top_length - _linkage_bottom_length) * 1.f;
  move_arm_to(_goal_pos.x, _goal_pos.y);
}
