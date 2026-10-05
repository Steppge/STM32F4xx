/*
  joystick_plugin.c - analog joystick jogging for grblHAL (STM32F4xx, BTT Octopus Pro)

  PROTOTYPE v0.2 - tested only in short bench runs. Keep a hand on the power switch.

  Reads three analog inputs (X, Y, Z stick) plus one digital enable input (PG15).
  While the enable switch is held and a stick is out of its dead zone, the plugin
  streams short jog segments. Speed follows the stick deflection and changes take
  effect with the next segment, so the motion stays smooth. Releasing the stick
  or the enable switch, or reversing a direction, cancels the jog.
  Any implausible ADC value (broken wire, short) stops motion.

  Place this file in Src\ (replace joystick_plugin.c) and keep -D ADD_MY_PLUGIN=1.

  Requirements: soft limits on ($20=1), jog soft-limit clipping if available ($40=1).

  Changes in v0.2:
   - jog is sent as short segments, speed changes no longer stop and restart the motion
*/

#include "driver.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "grbl/plugins.h"
#include "grbl/report.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/ioports.h"

#define JOY_VERSION "0.2"

// ------------------------------------------------------------------------
// Configuration - adjust to your sticks
// ------------------------------------------------------------------------

// Analog port numbers (order of AUXINPUTn_ANALOG in the board map).
#define JOY_PORT_X 0
#define JOY_PORT_Y 1
#define JOY_PORT_Z 2

typedef struct {
    int32_t min;        // ADC value, stick fully in one direction
    int32_t center;     // ADC value, stick released
    int32_t max;        // ADC value, stick fully in the other direction
    int32_t deadzone;   // +/- around center that counts as neutral
    bool invert;        // reverse direction of this axis
} joy_axis_cfg_t;

// 12 bit ADC readings (0..4095), measured on this machine.
// Direction: raw above center = positive jog.
static const joy_axis_cfg_t joy_cfg[3] = {
    {  590, 1484, 1957, 60, false },    // X  (right 590, rest ~1484, left 1957)
    {  687, 1445, 1942, 60, false },    // Y
    {  693, 1534, 2035, 60, false }     // Z
};

// Readings outside this window are treated as a fault (broken wire reads ~4095, short ~0).
#define JOY_ADC_VALID_MIN   250
#define JOY_ADC_VALID_MAX  2600

#define JOY_FEED_XY       1500.0f   // mm/min at full deflection (start low!)
#define JOY_FEED_Z         500.0f   // mm/min at full deflection

#define JOY_LEVELS            20    // speed is quantised to 1/JOY_LEVELS steps
#define JOY_POLL_MS           20    // stick sampling interval
#define JOY_SEG_TIME_S      0.25f   // travel time covered by one jog segment (seconds)
#define JOY_SEG_PERIOD_MS    120    // a new segment is queued this often (must be < JOY_SEG_TIME_S)
#define JOY_ENABLE_ACTIVE_LOW  1    // 1: enable switch pulls the input low

// Enable switch on PG15 (Stop7), read directly from the GPIO register.
// $pins shows it as: [PIN:PG15,Aux in 6,P3]
#define JOY_EN_PORT   GPIOG
#define JOY_EN_PIN    15
#define JOY_ENABLE_PORT 3

// Debug: append the raw stick values and the enable input to every status report, e.g.
//   <Idle|MPos:...|Joy:1437,1437,1534,1>   (X, Y, Z ADC value, enable input level)
// Set to 0 when everything is calibrated.
#define JOY_DEBUG 1

// ------------------------------------------------------------------------

static on_execute_realtime_ptr on_execute_realtime;
static on_report_options_ptr on_report_options;
#if JOY_DEBUG
static on_realtime_report_ptr on_realtime_report;
#endif

static bool ready = false;
static uint8_t enable_port = IOPORT_UNASSIGNED;
static uint8_t dbg_analog = 0, dbg_port = IOPORT_UNASSIGNED;

static bool jogging = false;        // a jog started by this plugin is active
static bool cancel_sent = false;
static int8_t sent_sign[3] = {0};   // direction pattern of the running jog
static uint32_t last_poll = 0, last_send = 0;

static float normalize (int32_t raw, const joy_axis_cfg_t *c)
{
    float v = 0.0f;
    int32_t lo = c->center - c->deadzone, hi = c->center + c->deadzone;

    if(raw < lo)
        v = -(float)(lo - raw) / (float)(lo - c->min);
    else if(raw > hi)
        v = (float)(raw - hi) / (float)(c->max - hi);

    if(v > 1.0f)
        v = 1.0f;
    else if(v < -1.0f)
        v = -1.0f;

    v = v * fabsf(v); // quadratic response for finer control near center

    return c->invert ? -v : v;
}

// Returns false on a fault (implausible reading). q[] gets the quantised deflection per axis.
static bool read_sticks (int8_t q[3])
{
    static const uint8_t port[3] = { JOY_PORT_X, JOY_PORT_Y, JOY_PORT_Z };

    for(uint_fast8_t i = 0; i < 3; i++) {

        int32_t raw = ioport_wait_on_input(Port_Analog, port[i], WaitMode_Immediate, 0.0f);

        if(raw < JOY_ADC_VALID_MIN || raw > JOY_ADC_VALID_MAX)
            return false;

        q[i] = (int8_t)lroundf(normalize(raw, &joy_cfg[i]) * JOY_LEVELS);
    }

    return true;
}

static bool enable_active (void)
{
    bool level = DIGITAL_IN(JOY_EN_PORT, JOY_EN_PIN);

#if JOY_ENABLE_ACTIVE_LOW
    return !level;
#else
    return level;
#endif
}

static int8_t sgn (int8_t v)
{
    return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

static bool same_direction (const int8_t q[3])
{
    for(uint_fast8_t i = 0; i < 3; i++) {
        if(sgn(q[i]) != sent_sign[i])
            return false;
    }

    return true;
}

// Queue one short jog segment in the current stick direction at the current stick speed.
static void send_segment (const int8_t q[3])
{
    static const char axis[3] = { 'X', 'Y', 'Z' };

    float s[3] = {
        (float)q[0] * JOY_FEED_XY / JOY_LEVELS,
        (float)q[1] * JOY_FEED_XY / JOY_LEVELS,
        (float)q[2] * JOY_FEED_Z / JOY_LEVELS
    };
    float feed = sqrtf(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);

    if(feed < 1.0f)
        return;

    float dist = feed / 60.0f * JOY_SEG_TIME_S; // total path length of the segment, mm

    char cmd[80] = "$J=G91G21";
    char *p = cmd + strlen(cmd);

    for(uint_fast8_t i = 0; i < 3; i++) {
        if(q[i])
            p += snprintf(p, sizeof(cmd) - (size_t)(p - cmd), "%c%.3f", axis[i], dist * s[i] / feed);
    }

    snprintf(p, sizeof(cmd) - (size_t)(p - cmd), "F%.0f", feed);

    if(grbl.enqueue_gcode(cmd)) {
        for(uint_fast8_t i = 0; i < 3; i++)
            sent_sign[i] = sgn(q[i]);
        jogging = true;
        last_send = hal.get_elapsed_ticks();
    }
}

static void joy_realtime (sys_state_t state)
{
    if(on_execute_realtime)
        on_execute_realtime(state);

    uint32_t now = hal.get_elapsed_ticks();

    if(!ready || (now - last_poll) < JOY_POLL_MS)
        return;

    last_poll = now;

    // Only act when idle or while a jog is running. Alarm, hold, cycle etc. are left alone.
    if(state != STATE_IDLE && !(state & STATE_JOG)) {
        jogging = cancel_sent = false;
        memset(sent_sign, 0, sizeof(sent_sign));
        return;
    }

    int8_t q[3] = {0};
    bool ok = enable_active() && read_sticks(q);
    bool want = ok && (q[0] || q[1] || q[2]);

    if(!want) {
        // Released, disabled or fault: stop our own jog.
        if(jogging && (state & STATE_JOG) && !cancel_sent) {
            grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
            cancel_sent = true;
        }
        if(state == STATE_IDLE) {
            jogging = cancel_sent = false;
            memset(sent_sign, 0, sizeof(sent_sign));
        }
        return;
    }

    // A cancel is pending: wait until the machine has stopped.
    if(cancel_sent) {
        if(state == STATE_IDLE) {
            jogging = cancel_sent = false;
            memset(sent_sign, 0, sizeof(sent_sign));
        } else
            return;
    }

    // Direction changed (axis added, removed or reversed): stop first, restart when idle.
    if(jogging && !same_direction(q)) {
        if(state & STATE_JOG) {
            grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
            cancel_sent = true;
        } else {
            jogging = false;
            memset(sent_sign, 0, sizeof(sent_sign));
        }
        return;
    }

    // Same direction (or idle): keep feeding segments. Speed changes apply from the next segment on.
    if((now - last_send) >= JOY_SEG_PERIOD_MS)
        send_segment(q);
}

#if JOY_DEBUG
static void joy_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(ready) {

        char buf[48];

        snprintf(buf, sizeof(buf), "|Joy:%ld,%ld,%ld,%ld",
                  (long)ioport_wait_on_input(Port_Analog, JOY_PORT_X, WaitMode_Immediate, 0.0f),
                   (long)ioport_wait_on_input(Port_Analog, JOY_PORT_Y, WaitMode_Immediate, 0.0f),
                    (long)ioport_wait_on_input(Port_Analog, JOY_PORT_Z, WaitMode_Immediate, 0.0f),
                     (long)DIGITAL_IN(JOY_EN_PORT, JOY_EN_PIN));

        stream_write(buf);
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}
#endif

static void joy_report_options (bool newopt)
{
    static char info[64];

    on_report_options(newopt);

    if(!newopt) {
        if(ready)
            snprintf(info, sizeof(info), JOY_VERSION " (enable port %u)", enable_port);
        else
            snprintf(info, sizeof(info), JOY_VERSION " (inactive: analog=%u enable=%u)", dbg_analog, dbg_port);
        report_plugin("Joystick", info);
    }
}

void my_plugin_init (void)
{
    dbg_analog = ioports_available(Port_Analog, Port_Input);
    dbg_port = IOPORT_UNASSIGNED - 1;       // 254: enable port not found

    // The enable input must be exactly PG15, otherwise the plugin stays inactive.
    xbar_t *info = ioport_get_info(Port_Digital, Port_Input, JOY_ENABLE_PORT);

    if(info) {
        if((GPIO_TypeDef *)info->port == JOY_EN_PORT && info->pin == JOY_EN_PIN) {

            uint8_t port = JOY_ENABLE_PORT;
            ioport_claim(Port_Digital, Port_Input, &port, "Joystick enable"); // result ignored, reading is direct

            // Internal pull-up so the input idles high when the switch is open (same PUPDR method the driver uses).
            JOY_EN_PORT->PUPDR = (JOY_EN_PORT->PUPDR & ~(3u << (JOY_EN_PIN * 2))) | (1u << (JOY_EN_PIN * 2));

            dbg_port = JOY_ENABLE_PORT;
        } else
            dbg_port = IOPORT_UNASSIGNED - 2; // 253: port number is not PG15
    }

    if(dbg_analog >= 3 && dbg_port == JOY_ENABLE_PORT) {
        enable_port = JOY_ENABLE_PORT;
        ready = true;
    }

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = joy_realtime;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = joy_report_options;

#if JOY_DEBUG
    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = joy_realtime_report;
#endif
}
