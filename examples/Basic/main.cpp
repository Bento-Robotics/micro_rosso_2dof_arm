
#include <Arduino.h>
#include "micro_rosso.h"

#define BATTERY_VOLTAGE_PIN A1

#include "micro_rosso_basic_battery.h"
Basic_Battery bat;

#include "ticker.h"
Ticker ticker;

#include "ros_status.h"
RosStatus ros_status;

void setup()
{
  D_println("Booting...");

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  D_print("Setting up transport... ");
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  // if Wifi.config not called, will use DHCP
  // WiFi.config(MICRO_ROS_TRANSPORT_WIFI_STATIC_IP,MICRO_ROS_TRANSPORT_WIFI_STATIC_GATEWAY, MICRO_ROS_TRANSPORT_WIFI_STATIC_SUBNET);
  set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
#endif
  D_println("Done.");

  if (!micro_rosso::setup("batdemo_rclc"))
  {
    D_println("FAIL micro_rosso.setup()");
  }

  if (!ticker.setup())
  {
    D_println("FAIL ticker.setup()");
  };

  if (!ros_status.setup())
  {
    D_println("FAIL ros_status.setup()");
  };

  if (!bat.setup("/battery", ticker.timer_tick))
  {
    D_println("FAIL bat.setup()");
  };

  D_println("Boot completed.");
}

void loop()
{
  micro_rosso::loop();
}
