# micro rosso 2dof arm

This a module for the [micro_rosso](https://github.com/xopxe/micro_rosso_platformio) system.

It provides support for a 2dof robotic arm using servos and ROS2.
Currently very experimental stuff.  
TODO: use precise_servo

## Loading and starting

First, import the module into your project's `platformio.ini`:

```ini
lib_deps =
    ...
        "Bento-Robotics/micro_rosso_2dof_arm": "^0.1.0"
```

Then, in your `main.cpp`:

```cpp
...
#include "micro_rosso_2dof_arm.h"
Two_DOF_Arm two_DOF_arm;

Servo servo_top;
Servo servo_bottom;
...
void setup() {
  servo_top.attach(3, 850, 2530);
  servo_bottom.attach(2, 900, 2520);

  const uint32_t linkage_bottom_length_mm = 140;
  const uint32_t linkage_top_length_mm = 250;
  two_DOF_arm.setup(&servo_top, &servo_bottom, linkage_bottom_length_mm, linkage_top_length_mm)
  ...
}
```

The setup method allows passing an optional topic namespace. It is declared as follows:

```h
  static bool setup(Servo *servo_bottom, Servo *servo_top,
                    uint linkage_bottom_length, uint linkage_top_length,
                    const char *ros_namespace = "/");
```


## Using the module

The module uses the following topics and services:

* **sub** arm_control_absolute: [geometry_msgs/msg/Point](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/Point.html). Moves the arm to the given point;
* **sub** arm_control_relative: [geometry_msgs/msg/Point](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/Point.html). Moves the arm by the given amount;
* **pub** joint_states: [sensor_msgs/msg/JointState](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/JointState.html). Joint states for simulation;
* **pub** rosout: [rcl_interfaces/msg/Log](https://docs.ros.org/en/jazzy/p/rcl_interfaces/msg/Log.html). Logging (mostly debug)
* **srv** home_arm: [std_srvs/srv/Trigger](https://docs.ros.org/en/jazzy/p/std_srvs/srv/Trigger.html). Moves arm into home position (retracted).

## Authors and acknowledgment

xxopxe@gmail.com - creator of [micro_rosso](https://github.com/xopxe/micro_rosso_platformio)
jvisca@fing.edu.uy - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2024

## License

MIT
