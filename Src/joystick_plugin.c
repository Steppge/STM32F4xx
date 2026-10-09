/*
  joystick_plugin.c - analog joystick jogging for grblHAL (STM32F4xx, BTT Octopus Pro)

  PROTOTYPE v0.5 - tested only in short bench runs. Keep a hand on the power switch.

  Reads three analog inputs (X, Y, Z stick) plus one digital enable input (PG15).
  While the enable switch is held and a stick is out of its dead zone, the plugin
  streams short jog segments. Speed follows the stick deflection and changes take
  effect with the next segment, so the motion stays smooth. Releasing the stick
  or the enable switch, or reversing a direction, cancels the jog.
  Any implausible ADC value (broken wire, short) stops motion.

  Started from my_plugin_init() in my_plugin.c (needs -D ADD_MY_PLUGIN=1).

  Requirements: soft limits on ($20=1), jog soft-limit clipping if available ($40=1).

  Changes in v0.2:
   - jog is sent as short segments, speed changes no longer stop and restart the motion

  Changes in v0.3:
   - a new segment is only queued when the planner holds less than JOY_MAX_QUEUED blocks.
     v0.2 queued segments faster than they were executed, so the planner filled up with
     full speed motion and slowing down the stick had (almost) no effect.
   - segment length is at least the braking distance (from $120-$122), so the motion
     stays smooth with the short queue.

  Changes in v0.4:
   - start lock: after power-up the joystick stays inactive until all sticks were centered once.

  Changes in v0.5:
   - JOY_STEPS speed steps per direction with hysteresis, readings are smoothed, smaller dead zone.
   - step 1 has a fixed slow speed (JOY_FEED_MIN_*) for touching off, steps 2..JOY_STEPS
     rise up to JOY_MAX_RATE_PCT of the axis max rate ($110-$112).
*/

#include "driver.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "grbl/plugins.h"
#include "grbl/report.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/ioports.h"
#include "grbl/planner.h"
#include "grbl/settings.h"

#define JOY_VERSION "0.5"

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
    {  590, 1484, 1957, 30, false },    // X  (right 590, rest ~1484, left 1957)
    {  687, 1445, 1942, 30, false },    // Y
    {  693, 1534, 2035, 30, false }     // Z
};

// Readings outside this window are treated as a fault (broken wire reads ~4095, short ~0).
#define JOY_ADC_VALID_MIN   250
#define JOY_ADC_VALID_MAX  2600

#define JOY_FEED_MIN_XY      5.0f   // mm/min at step 1 (fixed, for touching off)
#define JOY_FEED_MIN_Z       2.0f   // mm/min at step 1 (fixed, for touching off)
#define JOY_MAX_RATE_PCT    35.0f   // top step in % of the axis max rate ($110-$112)

#define JOY_STEPS             10    // speed steps per direction
#define JOY_HYST            0.03f   // hysteresis between steps, fraction of the full stick travel
#define JOY_DZ_HYST           10    // ADC counts: a running stick stops only this far inside the dead zone
#define JOY_FILTER             4    // smoothing, new reading weight 1/JOY_FILTER
#define JOY_POLL_MS           20    // stick sampling interval
#define JOY_SEG_TIME_S      0.10f   // travel time covered by one jog segment (seconds), lower = faster response
#define JOY_MAX_QUEUED         2    // max. planner blocks (incl. the running one) before a new segment is sent
#define JOY_BRAKE_MARGIN    1.2f    // segment is at least this times the braking distance
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
static bool armed = false;          // start lock released, all sticks were seen centered after power-up
static uint8_t enable_port = IOPORT_UNASSIGNED;
static uint8_t dbg_analog = 0, dbg_port = IOPORT_UNASSIGNED;

static bool jogging = false;        // a jog started by this plugin is active
static bool cancel_sent = false;
static int8_t sent_sign[3] = {0};   // direction pattern of the running jog
static uint32_t last_poll = 0;

static int8_t step[3] = {0};         // current speed step per axis, signed
static int32_t filt[3];             // smoothed readings, ADC counts * 16
static bool filt_init = false;

// Stick deflection -1..1 outside the dead zone, 0 inside.
static float normalize (int32_t raw, const joy_axis_cfg_t *c, int32_t deadzone)
{
    float v = 0.0f;
    int32_t lo = c->center - deadzone, hi = c->center + deadzone;

    if(raw < lo)
        v = -(float)(lo - raw) / (float)(lo - c->min);
    else if(raw > hi)
        v = (float)(raw - hi) / (float)(c->max - hi);

    if(v > 1.0f)
        v = 1.0f;
    else if(v < -1.0f)
        v = -1.0f;

    return c->invert ? -v : v;
}

// New speed step from the deflection, a step only changes when the stick is moved
// JOY_HYST beyond the step boundary, so ADC noise does not toggle the speed.
static int8_t update_step (int8_t cur, float d)
{
    if(d == 0.0f)
        return 0;

    int8_t target = (int8_t)ceilf(fabsf(d) * JOY_STEPS);
    if(target > JOY_STEPS)
        target = JOY_STEPS;
    if(d < 0.0f)
        target = -target;

    if(cur == 0 || (cur > 0) != (target > 0) || target == cur)
        return target;

    float a = fabsf(d), n = (float)abs(cur);

    if(abs(target) > abs(cur))
        return a > n / JOY_STEPS + JOY_HYST ? target : cur;

    return a <= (n - 1.0f) / JOY_STEPS - JOY_HYST ? target : cur;
}

// Returns false on a fault (implausible reading). q[] gets the speed step per axis.
static bool read_sticks (int8_t q[3])
{
    static const uint8_t port[3] = { JOY_PORT_X, JOY_PORT_Y, JOY_PORT_Z };
    int32_t raw[3];

    for(uint_fast8_t i = 0; i < 3; i++) {

        raw[i] = ioport_wait_on_input(Port_Analog, port[i], WaitMode_Immediate, 0.0f);

        if(raw[i] < JOY_ADC_VALID_MIN || raw[i] > JOY_ADC_VALID_MAX) {
            filt_init = false;
            memset(step, 0, sizeof(step));
            return false;
        }
    }

    for(uint_fast8_t i = 0; i < 3; i++) {

        if(!filt_init)
            filt[i] = raw[i] * 16;
        else
            filt[i] += (raw[i] * 16 - filt[i]) / JOY_FILTER;

        // A running axis keeps moving until the stick is JOY_DZ_HYST counts inside the dead zone.
        int32_t dz = joy_cfg[i].deadzone - (step[i] ? JOY_DZ_HYST : 0);

        step[i] = update_step(step[i], normalize((filt[i] + 8) / 16, &joy_cfg[i], dz));
        q[i] = step[i];
    }

    filt_init = true;

    return true;
}

// Axis speed in mm/min for a speed step: step 1 is fixed, steps 2..JOY_STEPS rise
// quadratically from there to JOY_MAX_RATE_PCT of the axis max rate.
static float step_feed (uint_fast8_t axis, int8_t q)
{
    uint_fast8_t n = (uint_fast8_t)abs(q);

    if(n == 0)
        return 0.0f;

    float min = axis == Z_AXIS ? JOY_FEED_MIN_Z : JOY_FEED_MIN_XY;
    float max = settings.axis[axis].max_rate * JOY_MAX_RATE_PCT / 100.0f;

    if(max < min)
        max = min;

    float f = (float)(n - 1) / (float)(JOY_STEPS - 1);

    return min + (max - min) * f * f;
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

    float s[3];
    for(uint_fast8_t i = 0; i < 3; i++)
        s[i] = q[i] < 0 ? -step_feed(i, q[i]) : step_feed(i, q[i]);
    float feed = sqrtf(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);

    if(feed < 0.5f)
        return;

    float dist = feed / 60.0f * JOY_SEG_TIME_S; // total path length of the segment, mm

    // The planner must be able to stop at the end of the queued motion, a segment shorter than
    // the braking distance would make it slow down before the next one arrives (stutter).
    // Use the lowest acceleration of the moving axes, settings are in mm/min^2.
    float accel = 0.0f;
    for(uint_fast8_t i = 0; i < 3; i++) {
        if(q[i] && (accel == 0.0f || settings.axis[i].acceleration < accel))
            accel = settings.axis[i].acceleration;
    }
    if(accel > 0.0f) {
        float brake = feed * feed / (2.0f * accel) * JOY_BRAKE_MARGIN;
        if(dist < brake)
            dist = brake;
    }

    char cmd[80] = "$J=G91G21";
    char *p = cmd + strlen(cmd);

    for(uint_fast8_t i = 0; i < 3; i++) {
        if(q[i])
            p += snprintf(p, sizeof(cmd) - (size_t)(p - cmd), "%c%.4f", axis[i], dist * s[i] / feed);
    }

    snprintf(p, sizeof(cmd) - (size_t)(p - cmd), "F%.1f", feed);

    if(grbl.enqueue_gcode(cmd)) {
        for(uint_fast8_t i = 0; i < 3; i++)
            sent_sign[i] = sgn(q[i]);
        jogging = true;
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
    bool valid = read_sticks(q);

    // Start lock: after power-up the joystick stays inactive until all sticks have been seen centered once.
    if(!armed) {
        if(!(valid && !q[0] && !q[1] && !q[2]))
            return;
        armed = true;
    }

    bool ok = valid && enable_active();
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

    // Same direction (or idle): keep feeding segments, but keep the planner queue short so
    // speed changes take effect after at most JOY_MAX_QUEUED segments.
    if((plan_get_buffer_size() - plan_get_block_buffer_available()) < JOY_MAX_QUEUED)
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
            snprintf(info, sizeof(info), JOY_VERSION " (enable port %u%s)", enable_port, armed ? "" : ", start lock");
        else
            snprintf(info, sizeof(info), JOY_VERSION " (inactive: analog=%u enable=%u)", dbg_analog, dbg_port);
        report_plugin("Joystick", info);
    }
}

void joystick_init (void)
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
