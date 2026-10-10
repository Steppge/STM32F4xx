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

// Called for every digital output, remembers the port number of the fan pin.
static bool find_fan_port (xbar_t *pin, uint8_t port, void *data)
{
    if((GPIO_TypeDef *)pin->port == CTRL_FAN_PORT && pin->pin == CTRL_FAN_PIN) {
        *(uint8_t *)data = port;
        return true;
    }

    return false;
}

void controller_fan_init (void)
{
    uint8_t port = IOPORT_UNASSIGNED;

    // Find the output port number of the fan pin and claim exactly that port.
    // ioport_get_info() counts in driver order, ioport_claim() expects the port number (P0, P1..),
    // ioports_enumerate() delivers the port number for each pin.
    if(ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){0}, find_fan_port, &port)) {
        xbar_t *pin = ioport_claim(Port_Digital, Port_Output, &port, "Controller fan");
        fan_ok = pin && (GPIO_TypeDef *)pin->port == CTRL_FAN_PORT && pin->pin == CTRL_FAN_PIN;
    }

    if(fan_ok) {
        fan_set(false);
        stepper_enable = hal.stepper.enable;
        hal.stepper.enable = fan_stepper_enable;
    }
}
