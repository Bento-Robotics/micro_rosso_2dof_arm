#ifndef __battery_monitor_h
#define __battery_monitor_h


class Basic_Battery
{
public:
  Basic_Battery();
  static bool setup(const int analog_pin,
                    const char *topic_name = "/battery",
                    timer_descriptor &timer = micro_rosso::timer_report);
};

#endif // __battery_monitor_h
