/*
  controller_fan.c - stepper driver cooling fan for grblHAL (STM32F4xx, BTT Octopus Pro)

  The fan on FAN2 (PD12) runs while the stepper drivers are enabled and stops when
  grblHAL disables them, like Marlin's USE_CONTROLLER_FAN.

  Driver timing is set with grblHAL settings:
    $1=60000   drivers stay enabled for 60 s after the last move, then they are disabled
               (and with them the fan). The TMC2209 drops to the hold current
               ($210-$212, %) by itself after about 0.4 s standstill.
    $37=0      no axis is kept enabled permanently.

  The output is claimed from the aux outputs, so it shows as "Controller fan" in $pins
  and can not be switched with M62-M65.

  Started from my_plugin_init() in my_plugin.c.
*/

#include "driver.h"

#include "grbl/hal.h"
#include "grbl/ioports.h"

#define CTRL_FAN_PORT         GPIOD
#define CTRL_FAN_PIN          12
#define CTRL_FAN_ACTIVE_HIGH  1     // 1: fan runs when the output is high

static stepper_enable_ptr stepper_enable = NULL;
static bool fan_ok = false;

static void fan_set (bool on)
{
    if(fan_ok)
        DIGITAL_OUT(CTRL_FAN_PORT, CTRL_FAN_PIN, on == CTRL_FAN_ACTIVE_HIGH);
}

// Called from interrupt context, keep it short.
static void fan_stepper_enable (axes_signals_t enable, bool hold)
{
    stepper_enable(enable, hold);

    fan_set(enable.mask != 0);
}

void controller_fan_init (void)
{
    // Find the aux output port that is mapped to PD12 and claim it.
    uint8_t n_ports = ioports_available(Port_Digital, Port_Output);

    for(uint8_t port = 0; port < n_ports; port++) {

        xbar_t *info = ioport_get_info(Port_Digital, Port_Output, port);

        if(info && (GPIO_TypeDef *)info->port == CTRL_FAN_PORT && info->pin == CTRL_FAN_PIN) {
            uint8_t p = port;
            fan_ok = ioport_claim(Port_Digital, Port_Output, &p, "Controller fan") != NULL;
            break;
        }
    }

    if(fan_ok) {
        fan_set(false);
        stepper_enable = hal.stepper.enable;
        hal.stepper.enable = fan_stepper_enable;
    }
}
