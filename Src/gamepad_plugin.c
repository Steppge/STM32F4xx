/*
  gamepad_plugin.c - jogging with a Bluetooth gamepad for grblHAL (STM32F4xx, BTT Octopus Pro)

  PROTOTYPE v0.4 - jogging, jog enable on Options, buttons, connection watchdog. Keep a hand on the power switch.

  An ESP32 (see esp32_gamepad/) connects a PS4 controller (DualShock 4; DualSense and Xbox
  Series X|S work the same way) and sends its state every 20 ms over UART to the TFT header
  (USART1, PA9/PA10, stream instance 0):

    $GP,<seq>,<conn>,<lx>,<ly>,<rx>,<ry>,<l2>,<r2>,<buttons>,<dpad>,<misc>,<battery>*<hh>

  and gets the machine state back for the light bar and rumble (see esp32_gamepad/sketch.cpp):

    $ST,<state>,<event>,<enabled>*<hh>     event: 1 alarm, 2 job finished, 3 acknowledge

  Jogging works like the analog joystick plugin (joystick_plugin.c): JOY_STEPS speed steps per
  direction with hysteresis, step 1 has a fixed slow speed, segments are streamed as $J= jogs.
  Every axis has its own control, so a slightly diagonal stick never moves a second axis:

    left stick up/down      Y
    right stick left/right  X
    L2 / R2 (analog)        Z up / Z down (both pressed: Z stops)
    Options                 jog enable on/off (light bar red / blue)

  Buttons ("hold" = GP_HOLD_MS):

    Circle      tap: feed hold (job or jog), after the hold has completed: hold = STOP (abort the job)
    Cross       tap: resume (cycle start) after a hold or tool change
    Triangle    hold: home ($H)
    Square      hold: X0 Y0 (G10 L20 P0)
    R1          hold: Z0
    PS          tap: reset after a critical alarm (hard limit), else unlock ($X)
    Share       tap: next jog step 0.01 / 0.05 / 0.1 / 1 / 10 mm (shared with the display Move screen)
    D-pad       idle: one jog step X-/X+/Y-/Y+ per press, needs jog enable
                job:  up/down feed override +/-10 %, right/left power (spindle) override +/-10 %

  Safety: releasing a stick or trigger stops the axis, no valid packet for GP_TIMEOUT_MS stops
  everything. Jog enable switches off by itself after GP_ENABLE_IDLE_MS without motion, when the
  controller disconnects, on an alarm, when a job starts and at power-up. It can only be switched
  on while all sticks and triggers are at rest and the machine is idle.

  Enabled with GAMEPAD_ENABLE in my_machine.h, started from my_plugin_init() in my_plugin.c.
*/

#include "driver.h"

#if GAMEPAD_ENABLE

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "grbl/plugins.h"
#include "grbl/report.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/planner.h"
#include "grbl/settings.h"
#include "grbl/stream.h"

#define GP_VERSION "0.4"

// ------------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------------

#define GP_STREAM           0       // serial stream instance: 0 = USART1 on the TFT header
#define GP_BAUD        115200
#define GP_TIMEOUT_MS     200       // no valid packet for this long = connection lost

#define GP_AXIS_MAX       512       // Bluepad32 stick range -512..511
#define GP_DEADZONE        50       // +/- around center that counts as neutral (about 10 %)
#define GP_DZ_HYST         15       // a running axis stops only this far inside the dead zone
#define GP_TRIGGER_MAX   1023       // Bluepad32 trigger range 0..1023
#define GP_TRIGGER_DZ      50       // trigger travel that counts as released (about 5 %)
#define GP_ENABLE_IDLE_MS 60000     // jog enable switches off after this long without motion

#define GP_HOLD_MS        600       // press time for "hold" buttons
#define GP_STEP_RATE_PCT  30.0f     // D-pad step jog feed in % of the axis max rate
#define GP_JOB_MIN_MS    10000      // "job finished" rumble only after jobs running at least this long

// Bluepad32 button bits (PS4 names)
#define GP_BTN_CROSS     0x0001     // BUTTON_A
#define GP_BTN_CIRCLE    0x0002     // BUTTON_B
#define GP_BTN_SQUARE    0x0004     // BUTTON_X
#define GP_BTN_TRIANGLE  0x0008     // BUTTON_Y
#define GP_BTN_R1        0x0020     // BUTTON_SHOULDER_R
#define GP_MISC_PS       0x01       // MISC_BUTTON_SYSTEM
#define GP_MISC_SHARE    0x02       // MISC_BUTTON_SELECT
#define GP_MISC_OPTIONS  0x04       // MISC_BUTTON_START (Options)
#define GP_DPAD_UP       0x01
#define GP_DPAD_DOWN     0x02
#define GP_DPAD_RIGHT    0x04
#define GP_DPAD_LEFT     0x08

extern float tft_move_step_mm (void);
extern float tft_move_step_next (void);

#define GP_INVERT_X         1       // 1: reverse the jog direction of an axis
#define GP_INVERT_Y         1
#define GP_INVERT_Z         0

#define JOY_FEED_MIN_XY      5.0f   // mm/min at step 1 (fixed, for touching off)
#define JOY_FEED_MIN_Z       2.0f   // mm/min at step 1 (fixed, for touching off)
#define JOY_MAX_RATE_PCT    35.0f   // top step in % of the axis max rate ($110-$112)
#define JOY_STEPS             10    // speed steps per direction
#define JOY_HYST            0.03f   // hysteresis between steps, fraction of the full stick travel
#define JOY_POLL_MS           20    // control loop interval
#define JOY_SEG_TIME_S      0.10f   // travel time covered by one jog segment (seconds)
#define JOY_MAX_QUEUED         2    // max. planner blocks (incl. the running one) before a new segment is sent
#define JOY_BRAKE_MARGIN    1.2f    // segment is at least this times the braking distance

// Debug: append the received values to every status report, e.g.
//   <Idle|MPos:...|Pad:1,0,-3,12,0,0>   (connected, jog enabled, rx, ly, l2, r2)
#define GP_DEBUG 1

// ------------------------------------------------------------------------

typedef struct {
    bool connected;         // controller connected to the ESP32
    int16_t lx, ly, rx, ry; // sticks -512..511, up = negative
    uint16_t l2, r2;        // triggers 0..1023
    uint16_t buttons;
    uint8_t dpad, misc, battery;
} gp_data_t;

static io_stream_t const *gp_stream = NULL;
static gp_data_t pad = {0};
static uint32_t last_packet = 0;    // ms, time of the last valid packet
static bool alive = false;          // valid packets arrive

static on_execute_realtime_ptr on_execute_realtime;
static on_report_options_ptr on_report_options;
#if GP_DEBUG
static on_realtime_report_ptr on_realtime_report;
#endif

static bool enabled = false;        // jog enable (Options)
static uint32_t enabled_since = 0;  // ms, last motion or switch-on
static bool jogging = false;        // a jog started by this plugin is active
static bool cancel_sent = false;
static int8_t sent_sign[3] = {0};   // direction pattern of the running jog
static int8_t step[3] = {0};        // current speed step per axis, signed
static uint32_t last_poll = 0;

// ------------------------------------------------------------------------
// Receiving
// ------------------------------------------------------------------------

// All received characters go into the input buffer, none is a realtime command.
static bool gp_rx_char (const uint8_t c)
{
    return false;
}

static bool parse_packet (char *line)
{
    char *star = strchr(line, '*');
    uint8_t sum = 0;

    if(star == NULL)
        return false;

    *star = '\0';
    for(char *p = line; *p; p++)
        sum ^= (uint8_t)*p;

    if(strtoul(star + 1, NULL, 16) != sum)
        return false;

    int seq, conn, lx, ly, rx, ry, l2, r2, bat;
    unsigned int buttons, dpad, misc;

    if(sscanf(line, "GP,%d,%d,%d,%d,%d,%d,%d,%d,%x,%x,%x,%d", &seq, &conn, &lx, &ly, &rx, &ry, &l2, &r2, &buttons, &dpad, &misc, &bat) != 12)
        return false;

    pad.connected = conn == 1;
    pad.lx = lx; pad.ly = ly; pad.rx = rx; pad.ry = ry;
    pad.l2 = l2; pad.r2 = r2;
    pad.buttons = buttons; pad.dpad = dpad; pad.misc = misc; pad.battery = bat;

    return true;
}

static void receive (uint32_t now)
{
    static char line[96];
    static uint8_t len = 0;
    int32_t c;

    while((c = gp_stream->read()) != SERIAL_NO_DATA) {
        if(c == '$')
            len = 0;
        else if(c == '\n' || c == '\r') {
            line[len] = '\0';
            if(len > 4 && parse_packet(line))
                last_packet = now;
            len = 0;
        } else if(len < sizeof(line) - 1)
            line[len++] = (char)c;
        else
            len = 0;    // too long: discard
    }

    alive = (now - last_packet) < GP_TIMEOUT_MS;
}

// ------------------------------------------------------------------------
// Jogging (same logic as joystick_plugin.c)
// ------------------------------------------------------------------------

// Stick deflection -1..1 outside the dead zone, 0 inside.
static float normalize (int32_t raw, int32_t deadzone)
{
    float v = 0.0f;

    if(raw > deadzone)
        v = (float)(raw - deadzone) / (float)(GP_AXIS_MAX - deadzone);
    else if(raw < -deadzone)
        v = (float)(raw + deadzone) / (float)(GP_AXIS_MAX - deadzone);

    return v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
}

// New speed step from the deflection, a step only changes when the stick is moved
// JOY_HYST beyond the step boundary.
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

// Trigger travel 0..1 outside the dead zone, 0 inside.
static float normalize_trigger (int32_t raw)
{
    if(raw <= GP_TRIGGER_DZ)
        return 0.0f;

    float v = (float)(raw - GP_TRIGGER_DZ) / (float)(GP_TRIGGER_MAX - GP_TRIGGER_DZ);

    return v > 1.0f ? 1.0f : v;
}

// q[] gets the speed step per axis X, Y, Z.
static void read_sticks (int8_t q[3])
{
    // X: right stick left/right, Y: left stick up/down (up is negative), Z: L2 up, R2 down
    float d[3];
    int32_t x = GP_INVERT_X ? -pad.rx : pad.rx, y = GP_INVERT_Y ? pad.ly : -pad.ly;

    d[0] = normalize(x, GP_DEADZONE - (step[0] ? GP_DZ_HYST : 0));
    d[1] = normalize(y, GP_DEADZONE - (step[1] ? GP_DZ_HYST : 0));

    float up = normalize_trigger(pad.l2), down = normalize_trigger(pad.r2);
    d[2] = (up > 0.0f && down > 0.0f) ? 0.0f : up - down;
    if(GP_INVERT_Z)
        d[2] = -d[2];

    for(uint_fast8_t i = 0; i < 3; i++) {
        step[i] = update_step(step[i], d[i]);
        q[i] = step[i];
    }
}

static bool at_rest (void)
{
    return abs(pad.rx) <= GP_DEADZONE && abs(pad.ly) <= GP_DEADZONE && pad.l2 <= GP_TRIGGER_DZ && pad.r2 <= GP_TRIGGER_DZ;
}

// Axis speed in mm/min for a speed step: step 1 is fixed, steps 2..JOY_STEPS rise
// quadratically to JOY_MAX_RATE_PCT of the axis max rate. R2 slows everything down.
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

    float dist = feed / 60.0f * JOY_SEG_TIME_S;

    // The segment must be at least the braking distance, otherwise the motion stutters.
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

static void jog_stop (sys_state_t state)
{
    if(jogging && (state & STATE_JOG) && !cancel_sent) {
        grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
        cancel_sent = true;
    }
    if(state == STATE_IDLE) {
        jogging = cancel_sent = false;
        memset(sent_sign, 0, sizeof(sent_sign));
    }
}

// Tell the ESP32 the machine state and the jog enable (light bar, rumble).
static void send_state (sys_state_t state, uint8_t event)
{
    static int8_t last_state = -1;
    static bool last_enabled = false;
    static uint32_t last_sent = 0;

    int8_t st = state == STATE_IDLE ? 0 : (state & STATE_CYCLE) ? 1 : (state & (STATE_HOLD|STATE_TOOL_CHANGE)) ? 2 :
                 (state & (STATE_ALARM|STATE_ESTOP)) ? 3 : (state & STATE_JOG) ? 4 : (state & STATE_HOMING) ? 5 : 6;
    uint32_t now = hal.get_elapsed_ticks();

    // on change, on an event and once a second so a reconnected ESP32 is up to date
    if(st == last_state && enabled == last_enabled && !event && now - last_sent < 1000)
        return;

    char body[24], line[32];
    uint8_t sum = 0;

    snprintf(body, sizeof(body), "ST,%d,%u,%d", st, event, enabled ? 1 : 0);
    for(char *p = body; *p; p++)
        sum ^= (uint8_t)*p;
    snprintf(line, sizeof(line), "$%s*%02X\n", body, sum);
    gp_stream->write(line);

    last_state = st;
    last_enabled = enabled;
    last_sent = now;
}

static void set_enabled (bool on, sys_state_t state)
{
    if(on != enabled) {
        enabled = on;
        enabled_since = hal.get_elapsed_ticks();
        send_state(state, 3);   // short click on the controller
    }
}

// ------------------------------------------------------------------------
// Buttons
// ------------------------------------------------------------------------

typedef enum {
    Btn_Circle = 0, Btn_Cross, Btn_Triangle, Btn_Square, Btn_R1, Btn_PS, Btn_Share,
    Btn_Up, Btn_Down, Btn_Right, Btn_Left,
    Btn_Count
} gp_button_t;

typedef struct {
    bool down;
    bool fired;         // hold action done for this press
    uint32_t since;     // ms, press start
} btn_state_t;

static btn_state_t btn[Btn_Count];

static bool is_down (gp_button_t b)
{
    switch(b) {
        case Btn_Circle:   return !!(pad.buttons & GP_BTN_CIRCLE);
        case Btn_Cross:    return !!(pad.buttons & GP_BTN_CROSS);
        case Btn_Triangle: return !!(pad.buttons & GP_BTN_TRIANGLE);
        case Btn_Square:   return !!(pad.buttons & GP_BTN_SQUARE);
        case Btn_R1:       return !!(pad.buttons & GP_BTN_R1);
        case Btn_PS:       return !!(pad.misc & GP_MISC_PS);
        case Btn_Share:    return !!(pad.misc & GP_MISC_SHARE);
        case Btn_Up:       return !!(pad.dpad & GP_DPAD_UP);
        case Btn_Down:     return !!(pad.dpad & GP_DPAD_DOWN);
        case Btn_Right:    return !!(pad.dpad & GP_DPAD_RIGHT);
        case Btn_Left:     return !!(pad.dpad & GP_DPAD_LEFT);
        default:           return false;
    }
}

static bool enqueue (const char *cmd)
{
    char buf[48];

    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    return grbl.enqueue_gcode(buf);
}

// One jog step of the shared Move step size along X or Y.
static void step_jog (uint_fast8_t axis, float dir)
{
    char cmd[48];

    if((axis == X_AXIS && GP_INVERT_X) || (axis == Y_AXIS && GP_INVERT_Y))
        dir = -dir;

    snprintf(cmd, sizeof(cmd), "$J=G91G21%c%.3fF%.0f", axis == X_AXIS ? 'X' : 'Y', dir * tft_move_step_mm(),
              settings.axis[axis].max_rate * GP_STEP_RATE_PCT / 100.0f);
    if(enqueue(cmd))
        enabled_since = hal.get_elapsed_ticks();
}

// Tap action, called on press.
static void on_press (gp_button_t b, sys_state_t state)
{
    bool job = !!(state & (STATE_CYCLE|STATE_HOLD|STATE_TOOL_CHANGE));

    switch(b) {

        case Btn_Circle:
            if(state & (STATE_CYCLE|STATE_JOG))
                grbl.enqueue_realtime_command(CMD_FEED_HOLD);
            break;

        case Btn_Cross:
            if(state & (STATE_HOLD|STATE_TOOL_CHANGE))
                grbl.enqueue_realtime_command(CMD_CYCLE_START);
            break;

        case Btn_PS:
            if(sys.blocking_event)
                grbl.enqueue_realtime_command(CMD_RESET);
            else if((state & STATE_ALARM) && sys.alarm != Alarm_HomingRequired)
                enqueue("$X");
            break;

        case Btn_Share:
            tft_move_step_next();
            send_state(state, 3);
            break;

        case Btn_Up:
        case Btn_Down:
            if(job)
                grbl.enqueue_realtime_command(b == Btn_Up ? CMD_OVERRIDE_FEED_COARSE_PLUS : CMD_OVERRIDE_FEED_COARSE_MINUS);
            else if(enabled && state == STATE_IDLE)
                step_jog(Y_AXIS, b == Btn_Up ? 1.0f : -1.0f);
            break;

        case Btn_Right:
        case Btn_Left:
            if(job)
                grbl.enqueue_realtime_command(b == Btn_Right ? CMD_OVERRIDE_SPINDLE_COARSE_PLUS : CMD_OVERRIDE_SPINDLE_COARSE_MINUS);
            else if(enabled && state == STATE_IDLE)
                step_jog(X_AXIS, b == Btn_Right ? 1.0f : -1.0f);
            break;

        default:
            break;
    }
}

// Hold action, called once when the button has been held for GP_HOLD_MS.
static void on_hold (gp_button_t b, sys_state_t state)
{
    bool idle = state == STATE_IDLE;

    switch(b) {

        case Btn_Circle:    // STOP: abort the held job, the machine stands still so the position is kept
            if((state & STATE_HOLD) && sys.holding_state == Hold_Complete) {
                grbl.enqueue_realtime_command(CMD_RESET);
                send_state(state, 3);
            }
            break;

        case Btn_Triangle:
            if((idle || (state & STATE_ALARM)) && enqueue("$H"))
                send_state(state, 3);
            break;

        case Btn_Square:
            if(idle && enqueue("G10L20P0X0Y0"))
                send_state(state, 3);
            break;

        case Btn_R1:
            if(idle && enqueue("G10L20P0Z0"))
                send_state(state, 3);
            break;

        default:
            break;
    }
}

static void poll_buttons (sys_state_t state, uint32_t now)
{
    for(uint_fast8_t i = 0; i < Btn_Count; i++) {

        bool down = is_down((gp_button_t)i);

        if(down && !btn[i].down) {
            btn[i].since = now;
            btn[i].fired = false;
            on_press((gp_button_t)i, state);
        } else if(down && !btn[i].fired && now - btn[i].since >= GP_HOLD_MS) {
            btn[i].fired = true;
            on_hold((gp_button_t)i, state);
        }

        btn[i].down = down;
    }
}

static void gp_realtime (sys_state_t state)
{
    static bool was_connected = false, options_was = false;

    if(on_execute_realtime)
        on_execute_realtime(state);

    if(gp_stream == NULL)
        return;

    uint32_t now = hal.get_elapsed_ticks();

    receive(now);

    if((now - last_poll) < JOY_POLL_MS)
        return;

    last_poll = now;

    bool connected = alive && pad.connected;

    // Lost connection or new connection: jog enable off, buttons released.
    if(connected != was_connected) {
        was_connected = connected;
        options_was = false;
        memset(step, 0, sizeof(step));
        memset(btn, 0, sizeof(btn));
        set_enabled(false, state);
    }

    // Alarm or job: jog enable off.
    if(state & (STATE_ALARM|STATE_ESTOP|STATE_CYCLE))
        set_enabled(false, state);

    // Options toggles the jog enable, switching on only while idle and at rest.
    bool options = connected && !!(pad.misc & GP_MISC_OPTIONS);
    if(options && !options_was) {
        if(enabled)
            set_enabled(false, state);
        else if(state == STATE_IDLE && at_rest())
            set_enabled(true, state);
    }
    options_was = options;

    if(connected)
        poll_buttons(state, now);

    // Events for the controller: alarm (long rumble), job finished (two pulses)
    static sys_state_t last_state = STATE_IDLE;
    static bool job_active = false;
    static uint32_t job_start = 0;
    uint8_t event = 0;

    if((state & (STATE_ALARM|STATE_ESTOP)) && !(last_state & (STATE_ALARM|STATE_ESTOP))) {
        event = 1;
        job_active = false;
    }
    if((state & STATE_CYCLE) && !job_active) {
        job_active = true;
        job_start = now;
    } else if(job_active && state == STATE_IDLE) {
        job_active = false;
        if(now - job_start >= GP_JOB_MIN_MS)
            event = 2;
    }
    last_state = state;

    if(alive)
        send_state(state, event);

    // Only act when idle or while a jog is running.
    if(state != STATE_IDLE && !(state & STATE_JOG)) {
        jogging = cancel_sent = false;
        memset(sent_sign, 0, sizeof(sent_sign));
        return;
    }

    int8_t q[3] = {0};

    if(connected)
        read_sticks(q);

    if(q[0] || q[1] || q[2])
        enabled_since = now;
    else if(enabled && now - enabled_since >= GP_ENABLE_IDLE_MS)
        set_enabled(false, state);

    if(!(connected && enabled && (q[0] || q[1] || q[2]))) {
        jog_stop(state);
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

    // Direction changed: stop first, restart when idle.
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

    if((plan_get_buffer_size() - plan_get_block_buffer_available()) < JOY_MAX_QUEUED)
        send_segment(q);
}

// ------------------------------------------------------------------------
// Reports
// ------------------------------------------------------------------------

#if GP_DEBUG
static void gp_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    char buf[48];

    snprintf(buf, sizeof(buf), "|Pad:%u,%u,%d,%d,%u,%u", (alive && pad.connected) ? 1 : (alive ? 0 : 9), enabled ? 1 : 0,
              pad.rx, pad.ly, pad.l2, pad.r2);
    stream_write(buf);

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}
#endif

static void gp_report_options (bool newopt)
{
    static char info[48];

    on_report_options(newopt);

    if(!newopt) {
        if(gp_stream == NULL)
            strcpy(info, GP_VERSION " (no serial port)");
        else
            snprintf(info, sizeof(info), GP_VERSION " (%s%s)", alive ? (pad.connected ? "connected" : "no controller") : "no ESP32",
                      alive && pad.connected ? (enabled ? ", jog enabled" : ", jog locked") : "");
        report_plugin("Gamepad", info);
    }
}

// ------------------------------------------------------------------------

void gamepad_init (void)
{
    gp_stream = stream_open_instance(GP_STREAM, GP_BAUD, gp_rx_char, "Gamepad");

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = gp_realtime;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = gp_report_options;

#if GP_DEBUG
    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = gp_realtime_report;
#endif
}

#endif // GAMEPAD_ENABLE
