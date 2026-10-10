/*
  my_plugin.c - starts the machine specific extensions (BTT Octopus Pro)

  grblHAL calls my_plugin_init() once at startup when built with -D ADD_MY_PLUGIN=1.
  Jogging device: select JOYSTICK_ANALOG_ENABLE or GAMEPAD_ENABLE in my_machine.h.
*/

#include "driver.h"

#if JOYSTICK_ANALOG_ENABLE && GAMEPAD_ENABLE
#error "Enable only one of JOYSTICK_ANALOG_ENABLE and GAMEPAD_ENABLE in my_machine.h!"
#endif

extern void joystick_init (void);
extern void gamepad_init (void);
extern void tft_display_init (void);
extern void controller_fan_init (void);

void my_plugin_init (void)
{
#if JOYSTICK_ANALOG_ENABLE
    joystick_init();        // analog joystick jogging, see joystick_plugin.c
#endif
#if GAMEPAD_ENABLE
    gamepad_init();         // Bluetooth gamepad via ESP32, see gamepad_plugin.c
#endif
    tft_display_init();     // MKS TS35 status display, see tft_display.c
    controller_fan_init();  // stepper driver cooling fan on FAN2, see controller_fan.c
}
