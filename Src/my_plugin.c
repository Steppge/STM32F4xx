/*
  my_plugin.c - starts the machine specific extensions (BTT Octopus Pro)

  grblHAL calls my_plugin_init() once at startup when built with -D ADD_MY_PLUGIN=1.
*/

extern void joystick_init (void);
extern void tft_display_init (void);
extern void controller_fan_init (void);

void my_plugin_init (void)
{
    joystick_init();        // analog joystick jogging, see joystick_plugin.c
    tft_display_init();     // MKS TS35 status display, see tft_display.c
    controller_fan_init();  // stepper driver cooling fan on FAN2, see controller_fan.c
}
