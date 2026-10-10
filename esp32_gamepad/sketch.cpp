/*
  esp32_gamepad - Bluetooth gamepad receiver for the grblHAL gamepad plugin (BTT Octopus Pro)

  ESP32-WROOM-32 with Bluepad32. Connects a PS4 controller (DualShock 4; DualSense and
  Xbox Series X|S work the same way) and forwards its state over UART to the Octopus.
  All machine logic (jogging, dead man switch, buttons) is done in grblHAL, this sketch only
  transports data and drives the light bar and rumble.

  Wiring (Octopus TFT header):  5V -> 5V/VIN,  GND -> GND,  Octopus TX -> GPIO16 (RX2),  Octopus RX <- GPIO17 (TX2)

  Allowed controllers
    Only controllers stored in the allow list may connect, others are disconnected at once.
    A stored controller may always pair again (e.g. after it was used on a console).
    To add a new controller: hold BOOT (GPIO0) for 3 s, the on-board LED blinks for 60 s,
    put the controller in pairing mode (PS4: Share + PS until the light bar flashes).
    Hold BOOT for 10 s to clear the allow list.

  Protocol (ASCII lines, 115200 baud, checksum = XOR of all characters between '$' and '*')
    ESP32 -> Octopus, every 20 ms:
      $GP,<seq>,<conn>,<lx>,<ly>,<rx>,<ry>,<l2>,<r2>,<buttons>,<dpad>,<misc>,<battery>*<hh>
        conn     1 = controller connected, 0 = no controller (heartbeat, all values 0)
        lx..ry   sticks -512..511 (Bluepad32, up = negative)
        l2, r2   analog triggers 0..1023
        buttons  Bluepad32 button bitmask (hex), dpad (hex), misc buttons (hex)
        battery  0..255, 0 = unknown
    Octopus -> ESP32, on changes and once a second:
      $ST,<state>,<event>,<enabled>*<hh>
        state    0 idle, 1 run, 2 hold, 3 alarm, 4 jog, 5 homing, 6 other
        event    0 none, 1 alarm/limit, 2 job finished, 3 short click (acknowledge)
        enabled  jog enable 1/0

  Light bar
    idle locked blue, idle or jog with jog enable red, job green, hold orange, homing violet,
    alarm red blinking. Controller battery below 20 %: short yellow flash every 10 s.
  Rumble
    short click on acknowledges, long strong on an alarm, two medium pulses when a job is finished.
*/

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <Preferences.h>

#define UART_RX_PIN     16
#define UART_TX_PIN     17
#define UART_BAUD       115200
#define SEND_INTERVAL   20      // ms
#define PAIR_BUTTON_PIN        0
#define LED_PIN         2
#define PAIR_HOLD_MS    3000
#define CLEAR_HOLD_MS   10000
#define PAIR_WINDOW_MS  60000
#define MAX_ALLOWED     8
#define LIGHT_INTERVAL  100     // ms, light bar update
#define BLINK_MS        400     // alarm blink half period
#define BATTERY_LOW     51      // Bluepad32 battery 0..255, 51 = 20 %, 0 = unknown
#define BATTERY_FLASH_EVERY 10000
#define BATTERY_FLASH_MS    300

static ControllerPtr pad = nullptr;
static Preferences prefs;

static uint8_t allowed[MAX_ALLOWED][6];
static uint8_t n_allowed = 0;
static bool pairing = false;
static uint32_t pairing_until = 0;

static int machine_state = 0;      // 0 idle, 1 run, 2 hold, 3 alarm, 4 jog, 5 homing, 6 other
static int jog_enabled = 0;
static uint32_t light_shown = 0xFFFFFFFF;   // last color sent, forces an update when changed
static uint32_t second_pulse_at = 0;        // job finished: time of the second rumble pulse, 0 = none

// ------------------------------------------------------------------------
// Allow list (stored in NVS)
// ------------------------------------------------------------------------

static void allow_load (void)
{
    n_allowed = prefs.getBytes("allowed", allowed, sizeof(allowed)) / 6;
}

static void allow_save (void)
{
    prefs.putBytes("allowed", allowed, n_allowed * 6);
}

static bool is_allowed (const uint8_t *addr)
{
    for(uint8_t i = 0; i < n_allowed; i++) {
        if(!memcmp(allowed[i], addr, 6))
            return true;
    }
    return false;
}

static void allow_add (const uint8_t *addr)
{
    if(!is_allowed(addr) && n_allowed < MAX_ALLOWED) {
        memcpy(allowed[n_allowed++], addr, 6);
        allow_save();
    }
}

static void allow_clear (void)
{
    n_allowed = 0;
    prefs.remove("allowed");
    BP32.forgetBluetoothKeys();
}

// ------------------------------------------------------------------------
// Bluepad32 callbacks
// ------------------------------------------------------------------------

static void on_connected (ControllerPtr ctl)
{
    const uint8_t *addr = ctl->getProperties().btaddr;

    if(pairing && !is_allowed(addr)) {
        allow_add(addr);
        Console.printf("New controller allowed: %s\n", ctl->getModelName().c_str());
    }

    if(!is_allowed(addr) || (pad != nullptr && pad != ctl)) {
        Console.printf("Controller rejected: %02X:%02X:%02X:%02X:%02X:%02X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        ctl->disconnect();
        return;
    }

    pad = ctl;
    pairing = false;
    Console.printf("Controller connected: %s\n", ctl->getModelName().c_str());
    light_shown = 0xFFFFFFFF;   // set the light bar on the next update
    ctl->playDualRumble(0, 150, 0x60, 0x60);
}

static void on_disconnected (ControllerPtr ctl)
{
    if(ctl == pad) {
        pad = nullptr;
        Console.println("Controller disconnected");
    }
}

// ------------------------------------------------------------------------
// UART protocol
// ------------------------------------------------------------------------

static uint8_t checksum (const char *s)
{
    uint8_t c = 0;
    while(*s)
        c ^= (uint8_t)*s++;
    return c;
}

static void send_state (void)
{
    static uint8_t seq = 0;
    char body[96];

    if(pad && pad->isConnected())
        snprintf(body, sizeof(body), "GP,%u,1,%d,%d,%d,%d,%d,%d,%X,%X,%X,%u", seq++,
                  pad->axisX(), pad->axisY(), pad->axisRX(), pad->axisRY(),
                   pad->brake(), pad->throttle(), pad->buttons(), pad->dpad(), pad->miscButtons(), pad->battery());
    else
        snprintf(body, sizeof(body), "GP,%u,0,0,0,0,0,0,0,0,0,0,0", seq++);

    Serial2.printf("$%s*%02X\n", body, checksum(body));
}

#define RGB(r, g, b) (((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b))

// Light bar color for the machine state (DualShock 4 / DualSense)
static uint32_t state_color (uint32_t now)
{
    switch(machine_state) {
        case 0:  return jog_enabled ? RGB(100, 0, 0) : RGB(0, 0, 60);  // idle: red enabled, blue locked
        case 1:  return RGB(0, 70, 0);                                 // job: green
        case 2:  return RGB(100, 40, 0);                               // hold: orange
        case 3:  return (now / BLINK_MS) & 1 ? RGB(100, 0, 0) : 0;      // alarm: red blinking
        case 4:  return RGB(100, 0, 0);                                // jog: red (enabled)
        case 5:  return RGB(50, 0, 60);                                // homing: violet
        default: return RGB(30, 30, 30);                               // other: white
    }
}

static void update_light (uint32_t now)
{
    static uint32_t last = 0;

    if(!pad || now - last < LIGHT_INTERVAL)
        return;

    last = now;

    uint32_t color = state_color(now);
    uint8_t bat = pad->battery();

    if(bat && bat < BATTERY_LOW && now % BATTERY_FLASH_EVERY < BATTERY_FLASH_MS)
        color = RGB(100, 80, 0);    // low battery: yellow flash

    if(color != light_shown) {
        light_shown = color;
        pad->setColorLED(color >> 16, (color >> 8) & 0xFF, color & 0xFF);
    }
}

static void show_event (int event, uint32_t now)
{
    if(!pad)
        return;

    switch(event) {
        case 1:     // alarm / limit: long and strong
            pad->playDualRumble(0, 700, 0xFF, 0xFF);
            break;
        case 2:     // job finished: two medium pulses
            pad->playDualRumble(0, 250, 0x90, 0x60);
            second_pulse_at = now + 450;
            break;
        case 3:     // acknowledge: short click
            pad->playDualRumble(0, 60, 0x50, 0x00);
            break;
    }
}

static void update_rumble (uint32_t now)
{
    if(second_pulse_at && (int32_t)(now - second_pulse_at) >= 0) {
        second_pulse_at = 0;
        if(pad)
            pad->playDualRumble(0, 250, 0x90, 0x60);
    }
}

// Parse "$ST,<state>,<event>,<enabled>*hh" lines from the Octopus
static void receive (uint32_t now)
{
    static char line[48];
    static uint8_t len = 0;

    while(Serial2.available()) {
        char c = Serial2.read();
        if(c == '$')
            len = 0;
        else if(c == '\n' || c == '\r') {
            line[len] = '\0';
            char *star = strchr(line, '*');
            int state, event, enabled;
            if(star && len > 3) {
                *star = '\0';
                if(strtoul(star + 1, NULL, 16) == checksum(line) && sscanf(line, "ST,%d,%d,%d", &state, &event, &enabled) == 3) {
                    machine_state = state;
                    jog_enabled = enabled;
                    show_event(event, now);
                }
            }
            len = 0;
        } else if(len < sizeof(line) - 1)
            line[len++] = c;
    }
}

// ------------------------------------------------------------------------
// BOOT button: pairing window / clear allow list
// ------------------------------------------------------------------------

static void poll_boot_button (uint32_t now)
{
    static uint32_t down_since = 0;
    static bool handled = false;

    if(digitalRead(PAIR_BUTTON_PIN) == LOW) {
        if(!down_since)
            down_since = now;
        if(!handled && now - down_since >= CLEAR_HOLD_MS) {
            allow_clear();
            handled = true;
            Console.println("Allow list cleared");
        }
    } else {
        if(down_since && !handled && now - down_since >= PAIR_HOLD_MS) {
            pairing = true;
            pairing_until = now + PAIR_WINDOW_MS;
            Console.println("Pairing window open for 60 s");
        }
        down_since = 0;
        handled = false;
    }

    if(pairing && (int32_t)(now - pairing_until) >= 0)
        pairing = false;

    // LED: blinking while pairing, on while a controller is connected
    digitalWrite(LED_PIN, pairing ? ((now / 250) & 1) : (pad != nullptr));
}

// ------------------------------------------------------------------------

void setup (void)
{
    pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    prefs.begin("gamepad", false);
    allow_load();

    BP32.setup(&on_connected, &on_disconnected, true);
    BP32.enableVirtualDevice(false);    // no touchpad mouse
    BP32.enableBLEService(false);

    Console.printf("esp32_gamepad ready, %u allowed controller(s)\n", n_allowed);
}

void loop (void)
{
    static uint32_t last_send = 0;
    uint32_t now = millis();

    BP32.update();
    receive(now);
    poll_boot_button(now);
    update_light(now);
    update_rumble(now);

    if(now - last_send >= SEND_INTERVAL) {
        last_send = now;
        send_state();
    }

    vTaskDelay(1);
}
