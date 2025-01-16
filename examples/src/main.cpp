#include <Arduino.h>
#include <Servo.h>

#include "micro_rosso.h"

#include "ticker.h"
Ticker ticker;

#include "sync_time.h"
SyncTime sync_time;

#include "ros_status.h"
RosStatus ros_status;

#include "micro_rosso_2dof_arm.h"
Two_DOF_Arm two_DOF_arm;

void led_callback(int64_t last_call_time) {
  static bool status;
  digitalWrite(LED_BUILTIN, status);
  status = !status;
}

void setup() {
  D_println("Booting...");

  Servo servo1;
  Servo servo2;
  // DS3240 limits:  min: 400 , max: 2600
  // Values constrained by monting position
  servo1.attach(0, 900, 2600);
  servo2.attach(1, 850, 2550);
  //home arm
  servo1.writeMicroseconds(2600);
  servo2.writeMicroseconds(2550);


  pinMode(LED_BUILTIN, OUTPUT);
  D_print("Setting up transport... ");
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  if (!micro_rosso::setup("my_node_name"))
    D_println("FAIL micro_rosso.setup()");

  if (!ticker.setup())
    D_println("FAIL ticker.setup()");
  ticker.timer_tick.callbacks.push_back(&led_callback);

  if (!sync_time.setup())
    D_println("FAIL sync_time.setup()");

  if (!ros_status.setup())
    D_println("FAIL ros_status.setup()");

  //if (!two_DOF_arm.setup(servo1, servo2))
  //  D_println("FAIL two_DOF_arm.setup()");

  D_println("Boot completed.");
}

void loop() { micro_rosso::loop(); }
