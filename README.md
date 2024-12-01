# micro rosso battery monitor

This a module for the [micro_rosso](https://github.com/xopxe/micro_rosso_platformio) system.

It provides support for publishing battery voltage by publishing ROS2 topics.

## Loading and starting

First, import the module into your project's `platformio.ini`:

```ini
lib_deps =
    ...
        "Bento-Robotics/micro_rosso_battery_monitor": "^0.1.0"
```

Then, in your `main.cpp`:

```cpp
...
#include "micro_rosso_battery_monitor.h"
Basic_Battery basic_battery;

...
void setup() {
  basic_battery.setup(ANALOG_PIN);
  ...
}
```

The setup method allows passing optional topic names and a different micro_rosso timer to change the publication rate (by default, it uses the 1Hz timer). It is declared as follows:

```h
  static bool setup(int analog_pin,
                    const char *topic_temp = "/battery",
                    timer_descriptor &timer = micro_rosso::timer_report);
```


## Using the module

The module emits the following topic:

* battery_state: [sensor_msgs/msg/battery_state](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/BatteryState.html). Inserts ADC reading into voltage variable;

## Authors and acknowledgment

xxopxe@gmail.com - creator of [micro_rosso](https://github.com/xopxe/micro_rosso_platformio)
jvisca@fing.edu.uy - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2024

## License

MIT
