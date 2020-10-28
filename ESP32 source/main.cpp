/*
Name:		ESP32-TMC2130-v10
Created:	10/21/2020 6:30pm
Modified:   

v10:  changes behavior to use peer-to-peer ESP NOW. Stepper #2 (closer to base) accepts
    ESP NOW commands and relays them to Stepper #1.

Derives full 180 (peak-to-peak) trajectory and RMT data using one RMT data structure
Uses RMT to create a set of STEP pulses to move peak-peak in a modulated sinusoidal pattern
Receives 'T' beat period commands via ESPNOW from Repeater (SSID=Tumba)
Receives parameters from web app; saves parameters in NVS
Smooth amplitude transitions; each trajectory is full peak-to-peak, e.g. no adjustments
    at a "zero" crossing. When transitioning amplitudes, zero crossings occur at odd places.
When converting trajectory to RMT data, defines accurate period (usec accuracy) for each 
    substep by adding fractional (no-pulse) events at the end. 
Supports zero-step substeps in trajectory. 
Supports fractional angles in trajectory calculation
Modulates LED intensity according to angle
Supports ArduinoOTA
Supports receiving full set of parameters in "S" command
Implements "S?" command to request parameter set, "S:" command to receive parameter set
Uses parameter for amplitude
(v9) Uses broadcast beat phase for half-speed synch
    * Added on_now flag
    * Added "ON" and "OFF" ESPNOW commands
    * Reworked brightness vs. angle
    * Implemented support for Infrared Remote control
    *   5 commands:  OFF, ON, STILL, BRIGHT+, BRIGHT-
    *   Supports "OFF" command from the IR remote that controls the LED strip on the mantle
    * Implemented receipt of IR commands via ESP-NOW
Works with "Stepper Parameters" website (params.html), which connects and sends:
    * "Pn:" where "n" is a number; various parameters
    * "eraseNVS" self-explanatory
    
160 degree maximum per quarter-cycle (ex.: 0 to peak) at microsteps=128
80 degree maximum per quarter-cycle (ex.: 0 to peak) at microsteps=256
Cannot go further without exceeding available memory in the rmt_data array (32 bits per step) 
Combines all flags and parameters into one NVS blob. 
Implemented "current angle" separate from "default angle"; latter is saved in NVS.
Adjusted peak location by 10msec (synchronizing a bit late)
Addressing:  GPIO35 = 0 indicates Stepper 0;  GPIO35 = 1 indicates Stepper 1

STATUS/UNSOLVED PROBLEMS:
    * Computed total + and - steps in accumulated_steps; ends up very close to zero (no attempt to balance)
    * 160 degree maximum per quarter-cycle (ex.: 0 to peak) at microsteps=128
    * 80 degree maximum per quarter-cycle (ex.: 0 to peak) at microsteps=256
        - Cannot go further without exceeding available memory in the rmt_data array (32 bits per step) 
    * WiFi reception problems. Changed to Channel 11. No help.
    * Added extra data to commands to improve robustness.

TODO: 
    * Maybe: downloadable curve (or selectable curve), pendulum, spike, reverse curve, etc.
*/
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include "SPIFFS.h"
#include <SPI.h>
#include <WiFiMulti.h>
#include <ArduinoOTA.h>
#include <TMC2130Stepper.h>
#include <driver/rmt.h>
#include <esp32-hal-rmt.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_now.h"

#include "PinDefinitionsAndMore.h"

#define IRMP_PROTOCOL_NAMES 1 
#define IRMP_INPUT_PIN 4
#define LED_BUILTIN 27
#define BRIGHT_MIN 32  /* Lowest visigble brightness, after Gamma correction will = 1 */

#include <irmpSelectMain15Protocols.h>  // This enables 15 main protocols
#include <irmp.c.h>

IRMP_DATA irmp_data;


#define SW_VERSION "ESP32-TMC2130 v10b"

using namespace std;  

WebSocketsServer webSocket(81);    // create a websocket server on port 81
// Telnet
WiFiServer server(23);
WiFiClient serverClient;

#define WIFI_DISCONNECT_REASON_NOT_AUTHED 6
#define WIFI_DISCONNECT_REASON_BEACON_TIMEOUT 200
#define WIFI_DISCONNECT_REASON_NO_AP_FOUND 201
#define WIFI_DISCONNECT_REASON_AUTH_FAIL 202
#define WIFI_DISCONNECT_REASON_ASSOC_FAIL 203
#define WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT 204

//const char *base_ssid = "Tumbolia30-2.4";
//const char *base_password = "garyparty2007";
const char *base_ssid = "Tumba";
const char *base_password = "admin1234";
const char* assid = "Stepper";
const char* asecret = "admin1234";
char ap_ssid[8];

#define MDNS_HOSTNAME "stepX"  /* "X" replaced by Stepper Number */

const String sw_version = SW_VERSION;
const String yello = "\e[1;33m";
const String endcolo = "\e[0m";


#define MICROSTEPS 128  // or 256 for finer resolution motion, half as far
#define SUBSTEP_COUNT 50
#define MAX_QUARTER_CYC_ANGLE 75
#define MAX_QUARTER_CYC_STEPS MAX_QUARTER_CYC_ANGLE * MICROSTEPS
#define MAX_PEAK_PEAK_STEPS 2 * MAX_QUARTER_CYC_STEPS


bool on_now = false;
bool on_beat_motion = false;

enum State { Stopped, Acquiring, Tracking, Stopping };
State state = Stopped;

struct Trajectory {
  int us_per_step;
  int steps_per_bin;
};
Trajectory trajectory[2 * SUBSTEP_COUNT];  // full peak-to-peak trajectory

static EventGroupHandle_t event_group;
int shaft_direction = 0;
int beat_phase = 0;  // indicate the arrival of beat data
bool beat_late = false;
bool half_speed = false;

// see ESP32 stepper v2.xlsx:
// Bouncing ball: rectified sinewave (could later be a pendulum rather than sinewave)
const float lookup[] = { 1000.000, 1000.494, 1001.977, 1004.458, 1007.948, 1012.465, 1018.032, 1024.678, 1032.436, 1041.348, 1051.462, 1062.834, 1075.527, 1089.616, 1105.184, 1122.326, 1141.153, 1161.788, 1184.374, 1209.072, 1236.068, 1265.574, 1297.836, 1333.136, 1371.801, 1414.214, 1460.820, 1512.146, 1568.815, 1631.569, 1701.302, 1779.095, 1866.275, 1964.477, 2075.750, 2202.689, 2348.635, 2517.954, 2716.472, 2952.135, 3236.068, 3584.344, 4021.072, 4584.144, 5336.711, 6392.453, 7978.730, 10626.054, 15925.971, 31836.225 };
unsigned long prevMillis = millis();
unsigned long target_next_beat_ms;
unsigned long last_T_ms = 0;
unsigned long last_real_T_ms = 0;
unsigned long this_real_T_ms = 0;
unsigned long at_peak_ms;
unsigned long reach_peak_at_ms;
unsigned long stopped_timer_ms;

uint16_t average_beat_period;
bool new_beat = false;
int first_phase;
uint32_t beats_count = 0;
bool last_beat_rejected;

char command[20];  // received from ESPNOW, terminated with /r
uint8_t mac_214[] = {0x24, 0x6F, 0x28, 0x17, 0x16, 0xE4};  // MAC Address of ESP 214 (Stepper #1)
uint8_t mac_218[] = {0xCC, 0x50, 0xE3, 0xB6, 0x22, 0xFC};  // MAC Address of ESP 218 (Stepper #2)

int command_index = 0;   

int amp = 45;
int mstep = 256;
bool running = false;
bool stopping = false;
int steps = 0;
int accumulated_steps = 0;   // incremted for steps in one direction; decremented for steps in the other
bool t_command = false;
int this_ESP;
bool startup = false;
int last_brightness = 0;
char mdns_id[8];
int steps_to_resting = 0;
uint8_t stepper_lsb;
int phase_sync = 0;
uint8_t old_level = 255;

#define TEST_AMPLITUDE 45.0
float this_amplitude = 0;
float next_amplitude = 0; 
float new_amplitude;
float amp_set[2 * SUBSTEP_COUNT];

TimerHandle_t timerHdl;
TimerHandle_t saveTimerHdl;
#define SAVE_TIME_MS 1000

int light_cycle;
uint16_t total_steps;  // first quarter-cycle, second quarter-cycle
int new_bpm_period;
int real_bpm_period;
int received_beats_average_period;

#define EN_PIN    16  //	enable (CFG6)
#define DIR_PIN   25  //	(not used; direction controlled by SPI)
#define STEP_PIN  17  //	step
#define CS_PIN    5   //	chip select

#define myMOSI	23
#define myMISO	19
#define mySCK	18
//                                                     Step:   LOGIC B0 
#define DEBUG1 (gpio_num_t)26  /* GPIO26 is on header pin 1:   LOGIC B3 */
#define DEBUG2 (gpio_num_t)21  /* GPIO21 is on header pin 2:   LOGIC B1 */
#define DEBUG4 (gpio_num_t)22  /* GPIO22 is on header pin 4:   LOGIC B2 */
//#define DEBUG8 (gpio_num_t)32  /* GPIO32 is on header pin 8    LOGIC B5 */

#define STEPPER_ADDRESS0 (gpio_num_t)33

//NOTE:  GPIO 12, 13, 14, 15 are used for debugger 

// GPIO tied to Gate of the N-channel MOS FET (LEDs)
const int ledPin = 27;  

// setting PWM properties for LED brightness modulation
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

static int WSopen = 0;  // true/false, is WS connection open?
string ipStr;
tcpip_adapter_ip_info_t info;

nvs_handle my_nvs_handle;

static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static int s_retry_num = 0;
#define ESP_MAXIMUM_RETRY 10
#define REATTEMPT_INTERVAL_MS  60 * 5 * 1000
#define WIFI_CHANNEL 11

// // ESP NOW message queue
// int now_iptr = 0;
// int now_optr = 0;
#define RUNNING_QUEUE_SIZE 20
int beat_time_queue[RUNNING_QUEUE_SIZE];
int currently_used_period;
int bin_size_in_usec;  // microseconds per subslot, based on beat period

uint32_t default_the_amplitude = 45;
uint32_t default_running_average_depth = 4;
uint32_t default_max_deviation_percentage = 15;
uint32_t default_wild_beat_threshold = 20;
uint32_t default_tracking_mode_threshold = 10;
uint32_t default_amplitude_damping = 20;
uint32_t default_tracking_shift = 20;
uint32_t default_illum_low = 10;
uint32_t default_illum_high = 200;
uint32_t default_illum_resting = 0;
uint32_t default_half_speed_threshold = 100; // BPM
uint32_t default_motor_current = 400;

float amp_damp;  // = amplitude_damping / 100 as a percentage
uint32_t half_speed_threshold_period;

#define PARAM_COUNT 17
struct parameter_t {
    union {
        struct {
            uint16_t flags;
            uint16_t the_amplitude;            // P0
            uint16_t running_average_depth;    // P1
            uint16_t max_deviation_percentage; // P2
            uint16_t wild_beat_threshold;      // P3
            uint16_t tracking_mode_threshold;  // P4
            uint16_t amplitude_damping;        // P5
            uint16_t tracking_shift;           // P6
            uint16_t illum_low;                // P7
            uint16_t illum_high;               // P8
            uint16_t illum_resting;            // P9
            uint16_t half_speed_threshold;     // Pa
            uint16_t motor_current;            // Pb
        };
        uint16_t param_set[PARAM_COUNT];
    };
};

#define m256_flag BIT0
#define test_amp BIT1
#define test_sync BIT2
#define test_half_speed BIT3
#define test_motor BIT4
#define stealth_chop BIT5
#define saved_on_now BIT14
#define saved_on_beat_motion BIT15

#define DEFAULT_MICROSTEPS 128
#define DEFAULT_MOTOR_CURRENT 200
int m256_mode = DEFAULT_MICROSTEPS;
int old_m256_mode = DEFAULT_MICROSTEPS;
int old_motor_current = DEFAULT_MOTOR_CURRENT;
bool dir = false;

parameter_t params;
int debugx = 0;

#define MISSING_BEATS_MS 3000

TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN, myMOSI, myMISO, mySCK);  // this initializes SW SPI

const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };
  
#define AMPLITUDE_THRESHOLD 45

// ------------------------------------------- Utilities --------------------------------------------------
static void io_set_1(gpio_num_t gpio_num) {
    gpio_set_level(gpio_num, 1);
}

static void io_set_0(gpio_num_t gpio_num) {
    gpio_set_level(gpio_num, 0);

}

static void io_pulse1(gpio_num_t gpio_num) {
    gpio_set_level(gpio_num, 1);
    gpio_set_level(gpio_num, 0);
}

// static void io_pulse0(gpio_num_t gpio_num) {
//     gpio_set_level(gpio_num, 0);
//     gpio_set_level(gpio_num, 1);
// }

void set_flag(int b, uint16_t value) {  // b is bitmask
    if (value)
        params.flags |= b;   // set it 
    else
        params.flags &= ~b;  // clear that bit
}

bool get_flag(uint16_t b) {
    uint16_t x = params.flags & b;
    return (x==b);
}

void print32(uint32_t value) {
	// for testing, use Serial.print to print a 32-bit unsigned integer in hex
	char charVal[9];
	sprintf(charVal, "%08X", value);
	Serial.println(charVal);
}


void set_brightness_level(uint8_t level) {
    if (level == 0) {
        ledcWrite(ledChannel, 0);
        return;
    }
    if (level <32)
        level = 32;
    ledcWrite(ledChannel, gamma8[level]);
}

void lightOn(xTimerHandle pxTimer) {
    uint8_t j;
    uint32_t bright, target, hi, lo;
    float angle_factor, fade_factor;
    int s = light_cycle / 50;
    // light_cycle counts 0 to 2*SUBSTEP_COUNT-1 (e.g. 0-99)

    j = s ? (2 * SUBSTEP_COUNT) - 1 - light_cycle : light_cycle;
    // j goes from 0 (peak, bright) to 49 (middle, dim), then 49 back down to 0
    // amp_set[i] is amplitude for each step i in full peak-to-peak trajectory

    fade_factor = amp_set[light_cycle] / params.the_amplitude;
    // fade_factor tracks amplitude change during fade-in or fade-out
    // fade_factor = 1.0 when running full amplitude (before fade)
    // fade_factor = 0 after fade to still

    target = (on_now) ? params.illum_resting : BRIGHT_MIN;
    // target = the brightness level (32- 255) we reach when fade_factor = 0

    hi = (fade_factor * params.illum_high) + (1.0 - fade_factor) * target;
    lo = (fade_factor * params.illum_low) + (1.0 - fade_factor) * target;

    angle_factor = (float)(SUBSTEP_COUNT - 1.0 - j) / (float)(SUBSTEP_COUNT - 1.0);
    // angle_factor = 1.0 at peaks, 0 in the middle
    bright = (angle_factor * hi) + (1.0 - angle_factor) * lo;

    set_brightness_level(bright);  // range 32-255

//    printf("(%2i)  ff:%-5.2f  t:%-4u  a:%-5.2f hi:%-4u lo:%-4u  bright:%-4u\n", j, fade_factor, target, angle_factor, hi, lo, bright);

    light_cycle++;
    if (light_cycle >= 2 * SUBSTEP_COUNT) {
        xTimerStop(pxTimer, 0);
    }
    if (light_cycle % 5 == 0) {
    uint32_t r = driver.DRVSTATUS();
        if (r && get_flag(test_motor)) 
            printf("otWarn: %i  ot: %i  StallGrd: %4u  CoilA: %4u  CoilB: %4u  Err: %i\n", driver.otpw(), driver.ot(), driver.sg_result(), driver.coil_A_current(), driver.coil_B_current(), driver.drv_err());
    }
}

class Swinger {
    public:
    rmt_item32_t * rmt_data;
    Swinger();
};

Swinger::Swinger(void) {
//    uint32_t start_heap = ESP.getFreeHeap();
//    printf("starting heap: %i\n", start_heap);
    try { 
        rmt_data = new rmt_item32_t[MAX_PEAK_PEAK_STEPS];
    }
    catch (exception& e) {
        printf("rmt_data memory allocation error: %s\n", e.what());
    }
}

Swinger swinger;

// ------------------------------------------- Ring class -------------------------------------------------
class Ring {
    public:
        int average(int n);
        int fullness();
        void reset();
        void add(int item);
        Ring(int size) {  // create a ring buffer of size size
            buffer_size = size;
            in_index = 0;
            out_index = 0;
            buffer = new int[size];
            memset(buffer, 0, sizeof(int) * size);
        }
    private:
        int in_index;
        int out_index;
        int * buffer;
        int buffer_size;
};

void Ring::reset() {
    in_index = 0;
    out_index = 0;
    memset(buffer, 0, sizeof(int) * buffer_size);
}

void Ring::add(int item) {
    buffer[in_index] = item;
    in_index = (in_index + 1) % buffer_size;
    if (in_index == out_index)
        out_index = (out_index + 1) % buffer_size;
    // we can fill this queue, even though we only look at most recent ones for average
}

int Ring::average(int m) { // return the average of the last m values entered
    // N = running_average_depth
    int sum, index, n;
    int f = fullness();
    // start with last entry
    sum = 0;
    n = (f >= m) ? m : f;
    for (int i = 0; i < n; i++) {
        index = (buffer_size + in_index - i - 1) % buffer_size;
        sum += buffer[index];
    }
    if (n == 0) return 0;
    return sum / n;
}

int Ring::fullness() {
    return (buffer_size + in_index - out_index) % buffer_size;
}

Ring received_beats(RUNNING_QUEUE_SIZE);
Ring displayed_beats(RUNNING_QUEUE_SIZE);
// Ring samples0(QUEUE_SIZE);
// Ring samples1(QUEUE_SIZE);

// ------------------------------------------ IR Handler --------------------------------------------------

bool save_params();  // forward reference

#define BRIGHT_STEP_SIZE 4
void brighten(int up) {
    uint16_t x = (up) ? BRIGHT_STEP_SIZE : -BRIGHT_STEP_SIZE;
    uint16_t val = params.illum_resting;
    val += x;
    if (val > 255) val = 255;
    if (val < BRIGHT_MIN) val = BRIGHT_MIN;
    params.illum_resting = val;
    xTimerStart(saveTimerHdl, 0);  // start or reset the save timer
}

void handle_ir_data() {
    if (irmp_get_data(&irmp_data)) {
        if (irmp_data.address == 0xFF80) {
            if (irmp_data.flags & IRMP_FLAG_REPETITION) { // repeated key
                switch (irmp_data.command) {
                case 0xF:
                    Serial.print("BRIGHT+");
                    brighten(1);
                    break;
                case 0x19:
                    Serial.print("BRIGHT-");
                    brighten(0);
                    break;
                default:
                    break;
                }
            Serial.println();
            return;
            } // not a repeated key: first touch
            switch (irmp_data.command) {
            case 0x13:
                Serial.print("OFF");
                on_now = false;
                set_flag(saved_on_now, false);
                break;
            case 0xF:
                Serial.print("BRIGHT+");
                brighten(1);
                break;
            case 0x10:
                Serial.print("STILL");
                on_now = true;
                set_flag(saved_on_now, true);
                on_beat_motion = false;
                set_flag(saved_on_beat_motion, false);
                break;
            case 0x14:
                Serial.print("ON");
                on_now = true;
                set_flag(saved_on_now, true);
                on_beat_motion = true;
                set_flag(saved_on_beat_motion, true);
                break;
            case 0x19:
                Serial.print("BRIGHT-");
                brighten(0);
                break;
            default:
                break;
            }
            save_params();
            Serial.println();
        }
        else {
            if (irmp_data.address == 0xFF00)
                if (!irmp_data.flags & IRMP_FLAG_REPETITION) 
                    if (irmp_data.command == 0x1F)
                        Serial.print("OFF");
                        on_now = false;
                        set_flag(saved_on_now, false);
                        save_params();
                        return;
        }
    }
}

// ------------------------------------------ Telnet Print ------------------------------------------------
template<typename... Args> void telnet_printf(const char * f, Args... args) {
    char cbuff [160];
    int len;
    if (serverClient || serverClient.connected()) {
        len = sprintf(cbuff, f, args...);
        cbuff[len++] = '\r';
        serverClient.write(cbuff, len); 
    }
}

void new_telnet_client() {
    if (WiFi.status() == WL_CONNECTED) {
        if (server.hasClient()) {
            if (!serverClient || !serverClient.connected()) {
                if (serverClient) serverClient.stop();
                serverClient = server.available();
                if (!serverClient) printf("available broken\n");
                telnet_printf("New telnet client: %s\n", serverClient.remoteIP().toString().c_str());
                telnet_printf("\e[1;33;40m%s\e[0m\n", sw_version.c_str());
                telnet_printf("\e[0;32mConnected to %s on channel %i\e[0m\n", base_ssid, WiFi.channel());
                tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &info);
                telnet_printf("Got IP address %s\n", ip4addr_ntoa(&info.ip));
            }
        }
        if (serverClient && !serverClient.connected()) {
            serverClient.stop();
        }
    }
}

template<typename... Args> void tprintf(const char * f, Args... args) {
    printf(f, args...);
    telnet_printf(f, args...);
}

// ----------------------------------------------- Tasks --------------------------------------------------
void check_half_speed() {
    int n;
    if (get_flag(test_half_speed)) 
        printf("check half speed:  hs-in: %i  rbap: %i  thresh: %i", half_speed, received_beats_average_period, params.half_speed_threshold);
    n = (110 * (int)half_speed_threshold_period / 100);
    if (half_speed) {
        if (received_beats_average_period > n)
            half_speed = false; 
    } else {
        if (received_beats_average_period < (int)half_speed_threshold_period)
            half_speed = true;
    }
    if (get_flag(test_half_speed)) 
        printf("  n:  %i  hs-out: %i\n", n, half_speed);
}

void accept(unsigned long ms) {
}

void handle_T_command(string s) {
    if (!on_now || !on_beat_motion) {
        return;
    }
    phase_sync = (s[1] - 'a') & 3;
    string s0 = s.substr(2);
    real_bpm_period = atoi(s0.c_str());
    received_beats.add(real_bpm_period);
    received_beats_average_period = received_beats.average(params.running_average_depth);
    this_real_T_ms = millis();
    half_speed = true;
    if (phase_sync & 1)   // accept every other beat
        return;
    new_bpm_period = 2 * real_bpm_period;
    displayed_beats.add(new_bpm_period);
    last_T_ms = this_real_T_ms - 10;
    new_beat = true;
    first_phase = (phase_sync >> 1);  // to set synchronized motor direction on startup
    currently_used_period = displayed_beats.average(params.running_average_depth);
    beat_phase ^= 1;
    new_amplitude = 1.0 * params.the_amplitude;  // target amplitude = parameter amplitude (in case fading up)
//    io_pulse1(DEBUG8);
}

// --------------------------------------------- NVS STUFF ------------------------------------------------

void initialize_nvs() {
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
//    ESP_LOGI(TAG, "Opening Non-Volatile Storage(NVS) handle");
    ret = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Error [%s] opening NVS handle!", esp_err_to_name(ret));
}

void check_nvs_save(esp_err_t ret, string s, int val) {
    char *cstr = new char[s.length() + 1];
    strcpy(cstr, s.c_str());
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ret = nvs_set_i32(my_nvs_handle, cstr, val);
            if (ret != ESP_OK)
                printf("Error saving first %s value!\n", cstr);
            break;
        default:
            printf("Error(%s) reading NVS!\n", esp_err_to_name(ret));
    }
}

bool save_params() {
    size_t size = sizeof(parameter_t);
    if (nvs_set_blob(my_nvs_handle, "Params", &params, size) != ESP_OK) {
        printf("Error saving parameters\n");
        return false;
    }
    return true;
}

void restore_parameters() {  // fetch parameters from NVS if available
    parameter_t x;
    esp_err_t ret;
    size_t size = sizeof(parameter_t);
    ret = nvs_get_blob(my_nvs_handle, "Params", &x, &size);
    if (ret == ESP_OK) {
        params = x;
        printf("Restored amplitude to %u\n", params.the_amplitude);
        printf("Restored Running Average Depth to %i\n", params.running_average_depth);
        printf("Restored Max Deviation Percentage to %i\n", params.max_deviation_percentage);
        printf("Restored Tracking Mode Threshold to %i\n", params.tracking_mode_threshold);
        printf("Restored Wild Beat Threshold to %i\n", params.wild_beat_threshold);
        amp_damp = (float)params.amplitude_damping/(float)100.0;
        printf("Restored Amplitude Damping to %i%%\n", params.amplitude_damping);
        printf("Restored Tracking Shift to %i%%\n", params.tracking_shift);
        printf("Restored Illumination Low to %i\n", params.illum_low);
        printf("Restored Illumination High to %i\n", params.illum_high);
        printf("Restored Illumination (Resting) to %i\n", params.illum_resting);
        printf("Restored Half-Speed Threshold (BPM) to %i\n", params.half_speed_threshold);
        half_speed_threshold_period = round(60000.0 / (float)params.half_speed_threshold);
        on_now = get_flag(saved_on_now);
        on_beat_motion = get_flag(saved_on_beat_motion); 
   } else {
        printf("Using default parameter values (%i)\n", int(ret));
        params.the_amplitude = default_the_amplitude;
        params.running_average_depth = default_running_average_depth;
        params.max_deviation_percentage = default_max_deviation_percentage;
        params.wild_beat_threshold = default_wild_beat_threshold;
        params.tracking_mode_threshold = default_tracking_mode_threshold;
        params.amplitude_damping = default_amplitude_damping;
        amp_damp = (float)params.amplitude_damping/(float)100.0;
        params.tracking_shift = default_tracking_shift;
        params.illum_low = default_illum_low;
        params.illum_high = default_illum_high;
        params.illum_resting = default_illum_resting;
        params.half_speed_threshold = default_half_speed_threshold;
        half_speed_threshold_period = round(60000.0 / (float)default_half_speed_threshold);
        save_params();
    }
    new_amplitude = params.the_amplitude; 
}

void send_param_set(uint8_t num) {
    // first param is flag set
    String payload = "S:[";
    for (int i = 0; i < PARAM_COUNT - 1; i++) {
        payload += String(params.param_set[i]) + ", ";
    }
    payload += String(params.param_set[PARAM_COUNT - 1]) + "]";
    webSocket.sendTXT(num, payload);  // JSON array of decimal numbers
    tprintf("Sent: %s\n",payload.c_str());
}

void save_action(xTimerHandle pxTimer) {
    xTimerStop(pxTimer, 0);
    if (save_params()) 
        tprintf("Saved parameters\n");
}

// ------------------------------------------ WebSockets --------------------------------------------------

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) { // When a WebSocket message is received
    String payload_str = String((char*)payload);
    String angle_str, bpm_str, per_str, ip_str, smo_str, ok_str, p_str;
    esp_err_t ret;
    switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
        printf("[%u] Disconnected!\n", num);
        break;
    case WStype_CONNECTED: {          // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        tprintf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        ip_str = "ESP32 #" + String(this_ESP) + "\n" + sw_version;
        webSocket.sendTXT(num, ip_str);
        delay(100);
        running = false;
        break;
    }
    case WStype_TEXT: {                  // if new text data is received
        tprintf("[%u] Received: %s\n", num, payload);
        if (payload_str == "eraseNVS") {
            ret = nvs_flash_erase();
            if (ret == ESP_OK) {
                tprintf("Erased/initialized NVS\n");
                webSocket.sendTXT(num, "e:OK");
                initialize_nvs();
            } else {
                tprintf("Problem erasing NVS: %d\n", ret);
                webSocket.sendTXT(num, "e:ERROR");
            }
            return;
        }
        if (payload_str == "S?") {
            send_param_set(num);
            return;
        }
        if (payload_str[0] == 'S') {  // received full parameter set
            char *token = strtok((char*)payload, "[");
            for (int j = 0; j < PARAM_COUNT; j ++) {
                token = strtok(NULL, ",");
                int i = String(token).toInt();
//                printf("%s  (%i)\n", token, i);
                params.param_set[j] = i;
            }
            save_params();
        }
        if (payload_str[0] == 'P') {
            p_str = payload_str.substring(3);
            int pv = p_str.toInt();
            switch (payload_str[1]) {
                case '0':
                    params.the_amplitude = pv;   // use it now
                    break;
                case '1': 
                    params.running_average_depth = pv;
                    break;
                case '2':
                    params.max_deviation_percentage = pv;
                    break;
                case '3':
                    params.wild_beat_threshold = pv;
                    break;
                case '4':
                    params.tracking_mode_threshold = pv;
                    break;
                case '5':
                    params.amplitude_damping = 100 - pv;
                    amp_damp = (float)params.amplitude_damping / (float)100.0;
                    break;
                case '6':
                    params.tracking_shift = pv;
                    break;
                case '7':
                    params.illum_low = pv;
                    break;
                case '8':
                    params.illum_high = pv;
                    break;
                case '9':
                    params.illum_resting = pv;
                    break;
                case 'a':
                    params.half_speed_threshold = pv;
                    half_speed_threshold_period = round((float)60000.0 / (float)pv);
                    break;
                case 'b':
                    params.motor_current = pv;
                    break;
               default:
                  printf("received unknown parameter\n");
            }
            save_params();
        }
        if (payload_str[0] == 'B') {  // Binary flags
            p_str = payload_str.substring(3);
            int pv = p_str.toInt();
            switch (payload_str[1]) {
                case '0':
                    set_flag(m256_flag, pv);   // use it now
                    m256_mode = (pv) ? 256 : 128;
                    break;
                case '1': 
                    set_flag(test_amp, pv);
                    break;
                case '2':
                    set_flag(test_sync, pv);
                    break;
                case '3':
                    set_flag(test_half_speed, pv);
                    break;
                case '4':
                    set_flag(test_motor, pv);
                    break;
                case '5':
                    set_flag(stealth_chop, pv);
                    break;
                default:
                  printf("received unknown flag B%i\n", payload_str[1]);
            }
            save_params();
        }
        break;
    }
    case WStype_BIN:
        tprintf("Received %u bytes of binary data\n", length);
        break;
    default:
        return;
    }
}

// -------------------------------------------- mDNS Stuff -------------------------------------------

void start_mdns() {
    char host[] = MDNS_HOSTNAME;
    host[4] = char(stepper_lsb - 101 + '1');
    MDNS.begin(host);
    printf("MDNS host: %s\n", host);
    MDNS.addService("ping", "tcp", 22);  
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws", "tcp", 81);
}

// --------------------------------------------- WiFi ------------------------------------------------

String showDisconnectReason(int i) {
  switch (i) {
    case WIFI_DISCONNECT_REASON_NOT_AUTHED:                 return F("Not authorized");
    case WIFI_DISCONNECT_REASON_BEACON_TIMEOUT:             return F("Beacon timeout");
    case WIFI_DISCONNECT_REASON_NO_AP_FOUND:                return F("AP disappeared");
    case WIFI_DISCONNECT_REASON_AUTH_FAIL:                  return F("Authorization fail");
    case WIFI_DISCONNECT_REASON_ASSOC_FAIL:                 return F("Association fail");
    case WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT:          return F("Handshake timeout");
  }
  return "Unknown (" + String(i) + ")";
}

void initialize_telnet() {
    server.begin();
    server.setNoDelay(true);
}

bool wifi_connect_now(bool do_disconnect) {
    int tries = 0;
    bool now_connected = false;
    if (do_disconnect) {
        WiFi.persistent(false);
        WiFi.disconnect(true);
        delay(100);
    }

    WiFi.begin(base_ssid, base_password, WIFI_CHANNEL);  // wifi channel 

    while (!now_connected) {
        // Wait for WiFi connection
        printf(".");
        fflush(stdout);
        delay(100);
        tries++;
        if (tries > ESP_MAXIMUM_RETRY * 2) {
            printf("\nTimeout waiting for %s\n", base_ssid);
            return false;
        }
        now_connected = xEventGroupGetBits(s_wifi_event_group) && WIFI_CONNECTED_BIT;
    }
    return true;
}

static void initialize_OTA(); // forward reference

void WiFiEvent(WiFiEvent_t event, system_event_info_t infor) {
    switch (event) {
        case SYSTEM_EVENT_STA_START:
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num = 0;
            if (!WSopen) {
                initialize_telnet();
                tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &info);
                tprintf("Got IP address %s\n", ip4addr_ntoa(&info.ip));
                this_ESP = info.ip.addr >> 24;
                initialize_OTA();
                start_mdns();
                webSocket.begin();                  // start the websocket server
                webSocket.onEvent(webSocketEvent);  // if there's an incomming websocket message, go to
                                                    // function 'webSocketEvent'
                printf("Started WebSocket server\n");
                initialize_telnet();
	            tprintf("\n\e[0;32mConnected to %s on channel %i\e[0m\n", base_ssid, WiFi.channel());
                WSopen = 1;
           }
             break;
		case SYSTEM_EVENT_STA_CONNECTED:
			printf("\nStation connected\n");
			break;
        case SYSTEM_EVENT_STA_DISCONNECTED: {
            int reason = infor.disconnected.reason;
            if (reason==202) {
                printf("~~");
                delay(500);
                WiFi.begin(base_ssid, base_password);  // channel 1 specified
                delay(100);
                break;
            }
             printf("STA disconnected (%i)  Reason (%s)\n", s_retry_num, showDisconnectReason(reason).c_str());
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            WSopen = 0;
            if (s_retry_num < ESP_MAXIMUM_RETRY) {
                printf("Quick retry to connect to %s\n", base_ssid);
                wifi_connect_now(false);
                s_retry_num++;
            } else {
                printf("Connect to %s failed. Waiting a while to try again.", base_ssid);
                vTaskDelay(REATTEMPT_INTERVAL_MS / portTICK_PERIOD_MS);
                wifi_connect_now(true);
                s_retry_num = 0;
            }
            break;
        }
        default:
            break;
    }
}

// ----------------------------------------- Compute Path -------------------------------------------------
#define X_FACTOR 0.5805
void compute_trajectory(int period, float start_amplitude, float end_amplitude) {
    // period in milliseconds;  amplitudes in degrees
    // compute a full peak-peak trajectory based on period and start/end amplitude
	// if (verbose) printf("per= %i  amp= %i \n", period, use_amplitude);
    float amplitude;
	float factor1 = X_FACTOR * period / MICROSTEPS;
    float factor;
    float amp_float;
    int j;
	bin_size_in_usec = 1000 * period / (2 * SUBSTEP_COUNT);  // bin size in microseconds = bin size in msec * 1000 = 1000 * (period / SUBSTEP_COUNT)
    if (get_flag(test_amp) && (abs(start_amplitude - end_amplitude) > 0.1)) 
        printf("(%i usec)  %.3f--%-7.3f\n", bin_size_in_usec, start_amplitude, end_amplitude);
    if (get_flag(test_half_speed)) {
        printf("real per.: %i  RBA: %i  new per.: %i  curr. used per.: %i", real_bpm_period, received_beats_average_period, new_bpm_period, currently_used_period);
        if (half_speed)
            printf(" (HS)\n");
        else
            printf("\n");
    }

	for (int i = 0; i < SUBSTEP_COUNT * 2; i++) {
    // us per step = $G11 * 2 * xfactor *M$2 / (M$3 * M$6)
    //             = lookup[i] * 2 * X_FACTOR * period / (amplitude * microsteps)
    //=A_1 * (2 * subslots -F19) + A_2 * F19 / (2 * subslots)
        j = (i / SUBSTEP_COUNT) ? i % SUBSTEP_COUNT : SUBSTEP_COUNT - 1 - i;
        amp_float = (start_amplitude * (2 * SUBSTEP_COUNT - i)) + (end_amplitude * i);
        amplitude = amp_float / (2.0 * SUBSTEP_COUNT);
        amp_set[i] = amplitude;
        factor = factor1 / amplitude;
		trajectory[i].us_per_step = round(lookup[j] * factor);
		trajectory[i].steps_per_bin = round(bin_size_in_usec / (lookup[j] * factor));
        if (trajectory[i].steps_per_bin == 0) trajectory[i].us_per_step = -1;
//         if ((i > 22) && (i < 26)) {
//    printf("%d)  amplitude: %6.2f  us per step: %i   steps: %i\n", i, amplitude, trajectory[i].us_per_step, trajectory[i].steps_per_bin);
//         }
	}
//    printf("\n");
}

uint16_t compute_rmt_data(int * step_count) {  // return with the number of RMT data items (not the number of steps)
    // loads rmt data with all steps for 180 degree trajectory
    int rmt, us_per, accum_us;   
    float f; // testing
    int sub0;  
    rmt = 0; // counts RMT data items
    accum_us = 0; // testing accumulated time 
    for (int sub = 0; sub < SUBSTEP_COUNT * 2; sub++) {
        sub0 = sub;
        us_per = trajectory[sub0].us_per_step - 1;  // minus 1 because the pulse is 1us long
        steps = trajectory[sub0].steps_per_bin;
        accum_us += steps * (us_per + 1);
        if (steps == 0) {  // no steps, just waste time with a no-pulse RMT item
            swinger.rmt_data[rmt].level1 = 0;  // no pulse
            swinger.rmt_data[rmt++].duration0 = bin_size_in_usec - 1;
            accum_us += bin_size_in_usec - 1;
        }
        for (int s = 0; s < steps; s ++) {
            if (us_per > bin_size_in_usec) {
//                printf("[%i] %i) super large us_per: %u  -- steps_per_bin: %i\n", half, sub, us_per, steps);
                us_per = bin_size_in_usec;
            }
            swinger.rmt_data[rmt].level1 = 1;  // reset in case it was used as a spacer before
            swinger.rmt_data[rmt++].duration0 = us_per;
        }
   }
    f = (float)accum_us / 1000.0;
    if (get_flag(test_sync)) 
        printf("Accumulated time (quarter-cycle): %7.3f\n", f);
    *step_count = steps;
    return rmt;
}

/* void dump_test_info(String s, int which) {
    io_set_1(DEBUG4);
    printf("%s\n", s.c_str());
    printf("      new_bpm_period: %6u                      now: %6lu\n", new_bpm_period, millis());
    printf("           last_T_ms: %6lu      target_next_beat_ms: %6lu\n", last_T_ms, target_next_beat_ms);
    if (which) 
        printf("    reach_peak_at_ms: %6lu\n", reach_peak_at_ms);
    else
        printf("                                     reach_zero_at_ms: %6lu\n", reach_zero_at_ms);
    printf("   direct_bpm_period: %6u      average_beat_period: %6u\n", direct_bpm_period, average_beat_period);
    printf("total steps:  %u\n\n", total_steps);
    io_set_0(DEBUG4);
}

 */void prepare_for_next_peak() {  // adjust amplitude as needed
    int steps_here, m;
    next_amplitude = this_amplitude + (new_amplitude - this_amplitude) * amp_damp;
    if (this_amplitude + next_amplitude > 0.1 ) {
        if (get_flag(test_sync)) 
            printf("\nPrepare for next peak, period: %i\n", currently_used_period);
        if (get_flag(test_amp))
            printf(" new_a: %7.3f  this_a: %7.3f  next_a: %7.3f\n", new_amplitude, this_amplitude, next_amplitude);
        compute_trajectory(currently_used_period, this_amplitude, next_amplitude);  
        this_amplitude = next_amplitude; // 
        total_steps = compute_rmt_data(&steps_here);  // 
        m = (shaft_direction) ? 1 : -1;
        accumulated_steps += m * steps_here;
//        if (get_flag(verbose)) printf("computed RMT data, total_steps = %i \n", total_steps);
    }
}

//------------------------------------------ Receive ESPNOW -----------------------------------------------
void handle_esp_now_result(esp_err_t result) {
    if (result == ESP_OK) return;
    Serial.print("Send Error: ");
    if (result == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("ESPNOW no memory");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        Serial.println("Peer not found.");
    } else {
        Serial.println("Not sure what happened");
    }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    // Buffers the incoming message, checks for complete one
    string istr;
    if (data_len < 20) {
            for (int i = 0; i < data_len; i++) {
                command[command_index++] = data[i];
            }
    } else {
        command_index = 0;  // reset to receive something else
        return;
    }
    command[data_len] = 0;  // terminate string
    // is it a complete valid command?
    char x = command[command_index - 1]; // last character received
    if (command[0] == 'T' &&  x == '\r') {
        io_pulse1(DEBUG2);
        command[command_index - 1] = 0;
        istr = command;
        handle_T_command(istr);
        command_index = 0;
    }
    if (command[0] == 'R' &&  x == '\r') {  // IR command relayed through Repeater
        switch (command[1]) {
            case '1':                  // OFF
                on_now = false;
                set_flag(saved_on_now, false);
                break;
            case '2':                  // ON
                on_now = true;
                set_flag(saved_on_now, true);
                on_beat_motion = true;
                set_flag(saved_on_beat_motion, true);
                break;
            case '3':
                brighten(1);           // BRIGHTEN+
                break;
            case '4':                  // STILL 
                on_now = true;
                set_flag(saved_on_now, true);
                on_beat_motion = false;
                set_flag(saved_on_beat_motion, false);
                break;
            case '5':
                brighten(0);           // BRIGHTEN-
                break;
        }
        save_params();
        command_index = 0;
    }
    if (command_index) {  // not handled above
        printf("ESPNOW received: %s\n ", command);
        if ((data_len > 1) && (x == '\r')) {  // unknown command
            command_index = 0;  // reset for next command
        }
    }
    // if Stepper #2, forward to Stepper #1
    if (stepper_lsb == 102) {
        esp_err_t result = esp_now_send(mac_214, data, data_len);
        handle_esp_now_result(result);
    }
}

// ------------------------------------------ Setup Tasks -------------------------------------------------
static void initialize_debug_io() {
    gpio_set_direction(DEBUG1, GPIO_MODE_OUTPUT);
    gpio_set_direction(DEBUG2, GPIO_MODE_OUTPUT);
    gpio_set_direction(DEBUG4, GPIO_MODE_OUTPUT);
//    gpio_set_direction(DEBUG8, GPIO_MODE_OUTPUT);
}

static void initialize_RMT() {
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL_0;
    config.gpio_num = (gpio_num_t)STEP_PIN;
    config.mem_block_num = 8;  // maximum amount of DMA (512 x 2 x 16-bit)
    config.tx_config.loop_en = 0;
    config.tx_config.carrier_en = 0;  // no carrier
    config.tx_config.idle_output_en = 1;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.carrier_duty_percent = 50;
    config.tx_config.carrier_freq_hz = 10000;
    config.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    config.clk_div = 80; // 1usec resolution
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    for (int i = 0; i < MAX_PEAK_PEAK_STEPS; i++) {
        swinger.rmt_data[i].level0 = 0;
        swinger.rmt_data[i].duration1 = 1;
        swinger.rmt_data[i].level1 = 1;
    }
}

void initialize_espnow() {
//    WiFi.disconnect();
    tprintf("Initializing ESP NOW\n");
    delay(100);
    if (esp_now_init() != ESP_OK) {
        tprintf("ESPNow Init Failed\n");
        ESP.restart();
    }
    if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK) 
        tprintf("ESP NOW register receive callback failed\n");
    // if Stepper 1 Register peer
    if (stepper_lsb == 102) {
        esp_now_peer_info_t peerInfo;
        memset(&peerInfo, 0, sizeof(peerInfo));
        memcpy(peerInfo.peer_addr, mac_214, 6);  // if Stepper #2, add Stepper #1 as peer
        peerInfo.channel = WIFI_CHANNEL;
        peerInfo.encrypt = false;
        //Add peer
        if (esp_now_add_peer(&peerInfo) != ESP_OK){
            printf("Failed to add peer\n");
        } else {
            printf("Added 214 as peer\n");
        }

    
    }
}

static void initialize_OTA() {
    ArduinoOTA.onStart([]() {
        string stype;
        Serial.end();   // for more reliable upload?
        if (ArduinoOTA.getCommand() == U_FLASH)
            stype = "sketch";
        else  // U_SPIFFS
        {
            stype = "filesystem";
            nvs_close(my_nvs_handle);
        }
        tprintf("Starting OTA update of %s\n", stype.c_str());
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.begin(115200);
        // xEventGroupSetBits(s_wifi_event_group, WIFI_OTA_NOT_IN_PROGRESS_BIT);
        tprintf("\nOTA error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            tprintf("Auth Failed\n");
        else if (error == OTA_BEGIN_ERROR)
            tprintf("Begin Failed\f");
        else if (error == OTA_CONNECT_ERROR)
            tprintf("Connect Failed\n");
        else if (error == OTA_RECEIVE_ERROR)
            tprintf("Receive Failed\n");
        else if (error == OTA_END_ERROR)
            tprintf("End Failed\n");
        esp_restart();
    });
    ArduinoOTA.onEnd([]() {
        tprintf("\nOTA success!\n");
    });
    ArduinoOTA.setPort(3232);
    ArduinoOTA.begin();
}

static void initialize_wifi() {
    s_wifi_event_group = xEventGroupCreate();
    IPAddress local_IP(192, 168, 4, stepper_lsb);  
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(assid, asecret, 1, 1, 10);  // channel = 1, hidden = true, 10 max
    WiFi.onEvent(WiFiEvent);
    WiFi.config(local_IP, gateway, subnet);
    delay(500);
    if (!wifi_connect_now(true))  // start by disconnecting 
        printf("Could not connect to %s\n", base_ssid);
	delay(1000);
	initialize_espnow();
}

static void initialize_tmc() {
    bool stealth = false;
	pinMode(MISO, INPUT_PULLUP);
	driver.begin(); 			   // Initiate pins and registries
	driver.rms_current(400); 	   // Set stepper current to x00mA. The command is the same as command TMC2130.setCurrent(x00, 0.11, 0.5);
//    if (get_flag(stealth_chop))
        stealth = true;
	driver.stealthChop(stealth); 	       // Enable extremely quiet stepping
	driver.TPWMTHRS(0x00000032);   // from Trinamic: upper velocity threshold to switch between stelthChop and chopper operation (in time between steps TSTEP)
	driver.CHOPCONF(0x030101D5);   // from Trinamic: CHOPCONF: MRES= 32 microsteps, TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
    driver.TPOWERDOWN(32);          // approx. 500 msec (1/8 of 4 seconds)
    driver.microsteps(MICROSTEPS); // set microsteps•
//    driver.microsteps(256);
	digitalWrite(EN_PIN, HIGH);	 // Disable the motor driver for now (wait for a beat)
    uint32_t gconf;
    gconf = driver.GCONF();
    if (test_motor) 
        printf("GCONF: %10x\n", gconf);
}

void initialize_stepper_address() {
    gpio_set_direction(STEPPER_ADDRESS0, GPIO_MODE_INPUT);
    gpio_pullup_en(STEPPER_ADDRESS0);
    int a = gpio_get_level(STEPPER_ADDRESS0);
    // Implemented only two addresses now. If STEPPER_ADDRESS0 = 0, address 101, if = 1, address 102
    stepper_lsb = a ? 102 : 101;
    printf("Set LSB to %i\n", stepper_lsb);
}

#define TEST_BPM 160

void to_Stopped();  // forward reference

void setup() {
    initArduino();
    String v;
	Serial.begin(115200);        // Start the Serial communication to send messages to the computer
//    uint32_t start_heap = ESP.getFreeHeap();
	delay(10);
    v = "\n\n" + yello + sw_version + endcolo;
    printf("%s\n", v.c_str());  
//    printf("Starting heap: %i\n", start_heap);
    initialize_debug_io();
    initialize_stepper_address();

  	initialize_wifi();
    initialize_nvs();
    restore_parameters();
    delay(100);
    initialize_tmc();
    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(ledPin, ledChannel);
    initialize_RMT();
    event_group = xEventGroupCreate();
    to_Stopped();
    timerHdl = xTimerCreate("Timer", 1000, pdTRUE, ( void * ) 0, lightOn);
    irmp_init();
    irmp_irsnd_LEDFeedback(false); // Enable receive signal feedback at LED_BUILTIN
    saveTimerHdl = xTimerCreate("Save", SAVE_TIME_MS, pdTRUE, ( void * ) 0, save_action);
}

// ---------------------------------------------- LOOP ----------------------------------------------------

void idle() {
    webSocket.loop();               // constantly check for websocket events
    ArduinoOTA.handle();            // handle any OTA
    new_telnet_client();
    handle_ir_data();
}

void idle_until_rmt_data_sent() {
    while (rmt_wait_tx_done(RMT_CHANNEL_0, 0) != ESP_OK) {
        idle();
    }
}

// --------------------------------------- State Machine --------------------------------------------------
uint16_t resting_illum() {
    return on_now ? params.illum_resting : 0;
}

void to_Stopped() {
    digitalWrite(EN_PIN, HIGH);	 // Disable the motor driver for now    
    printf("Stopped...\n");
    received_beats.reset();
    displayed_beats.reset();
    // if (get_flag(print_status))
    //     printf("accumulated_steps: %i\n", accumulated_steps);
    accumulated_steps = 0;
    stopped_timer_ms = millis();  // used to ramp the amplitude from low to resting
    steps_to_resting = 0;
//    new_amplitude = params.illum_resting;  // this was an error
    state = Stopped;
}

void to_Acquiring() {
    printf("Acquiring...\n");
    digitalWrite(EN_PIN, LOW);   // Enable the motor driver  
    state = Acquiring;
}

#define STEPS_TO_RESTING 50
#define RESTING_INTERVAL_MS 100

void do_Stopped() {
    accumulated_steps = 0;
    if (new_beat) {
        new_beat = false;
        last_real_T_ms = 0;
//        shaft_direction = first_phase;  // synchronize direction with beat phase
        to_Acquiring();
    }
    set_brightness_level(resting_illum());
}

void to_Stopping() {
    printf("Stopping...\n");
    new_amplitude = 0;  
    state = Stopping;
}

void do_Tracking() {
    int relative_peak, amount, half_per;
    float fraction, adjustment;
    if (((millis() - last_T_ms) > MISSING_BEATS_MS) || !on_now) {
        // too long without any beats received
        to_Stopping();
        return;
    }
    if (100 * abs(new_bpm_period - currently_used_period)/currently_used_period > params.wild_beat_threshold) {
        printf("wild: new_bpm_period: %i  currently_used_period: %i\n", new_bpm_period, currently_used_period);
        to_Acquiring();
    }
    // Adjust based on phase !!
    half_per = currently_used_period / 2;
    relative_peak = millis() - last_T_ms;
    if (last_T_ms > at_peak_ms) { // received a beat between this peak and prior peak
        if (relative_peak < half_per) { // last peak was late
            amount = -relative_peak;
        } else {  // last peak was early
            amount = last_T_ms - at_peak_ms;
        }
        fraction = (float)amount / (float)half_per;
        adjustment = currently_used_period * fraction * (float)params.tracking_shift / 100.0;
        if (get_flag(test_sync)) 
            printf("fraction: %.3f  amount: %i  cur per: %i ", fraction, amount, currently_used_period);
        currently_used_period += adjustment;
        if (get_flag(test_sync)) 
            printf("adjusted: %i  adjustment: %.3f\n", currently_used_period, adjustment);
    }
}

void to_Tracking() {
    printf("\e[1;32;40mTracking...\e[0m\n");
    state = Tracking;
}

void do_Acquiring() {
    if (((millis() - last_T_ms) > MISSING_BEATS_MS) || !on_now) {
        // too long without any beats received
        to_Stopping();
        return;
    }
    if (received_beats.fullness() >= params.running_average_depth) {  // we have some beats now
        if ( 100 * abs(new_bpm_period - currently_used_period)/currently_used_period < params.max_deviation_percentage )
            to_Tracking();
    }
}

void do_Stopping() {
    // wait for amplitude to die down
    if (this_amplitude > 0.1) {  // test for amplitude near zero
        // if beat received, go to Acquiring
        if (new_beat) {
            new_beat = false;
            to_Acquiring();
            return;
        }
    } else {  // amplitude has died down
        idle_until_rmt_data_sent();
        to_Stopped();
    }
}

void start_light_cycle() {
    if (!bin_size_in_usec) return;
    xTimerStop(timerHdl, 0);

    light_cycle = 0;
    xTimerChangePeriod( timerHdl, bin_size_in_usec / 1000, 0);    
//    set_brightness_level(brightest());  
    xTimerStart(timerHdl, 0);
}

void end_light_cycle() {
    xTimerStop(timerHdl, 0);
}

//****************************************************************************************************
unsigned long dir_timing;

void move_motor_peak_to_peak() {
    unsigned long travel_time;
    long late_early;

    io_set_1(DEBUG4);
    at_peak_ms = millis();
    start_light_cycle();
    
    // move from peak toward next peak
    if (total_steps > 0)
        rmt_write_items(RMT_CHANNEL_0, (const rmt_item32_t *)swinger.rmt_data, total_steps, false);

    idle_until_rmt_data_sent();
    io_set_0(DEBUG4);
    reach_peak_at_ms = millis();
    travel_time = millis() - at_peak_ms;
    late_early = (bin_size_in_usec / 10) - travel_time;  // 100 total steps (bins), convert usec to msec
    if (get_flag(test_sync)) {
        if (late_early > 0)
        printf("Elapsed time (msec.): %lu  early by %li msec.", travel_time, late_early);
        else if (late_early < 0)
            printf("Elapsed time (msec.): %lu  \e[1;31;40mlate\e[0m by %li msec.", travel_time, late_early);
        else if (late_early == 0)
            printf("Elapsed time perfect");
        if (half_speed) printf(" \e[1;35;40m(HS)\e[0m");
        printf("\n");
    }

    shaft_direction ^= 1;  // reverse direction and go back to other peak
    driver.shaft_dir(shaft_direction);

    if (m256_mode != old_m256_mode) {
        driver.microsteps(m256_mode); // set microsteps
        printf("Microsteps changed to %i\n", m256_mode);
        old_m256_mode = m256_mode;
    }
    if (params.motor_current != old_motor_current) {
        driver.rms_current(params.motor_current);
        old_motor_current = params.motor_current;
    }
    end_light_cycle();
    gpio_set_level(DEBUG1, shaft_direction);

// =============================== Reached 90 degrees (peak!) =============================================
}

// ---------------------------------------------- LOOP ----------------------------------------------------

void loop() {
    if (state > Stopped) {
        move_motor_peak_to_peak();
    }
    switch (state) {
        case Stopped:
            do_Stopped();
            break;
        case Stopping:
            do_Stopping();
            break;
        case Acquiring:
            do_Acquiring();
            break;
        case Tracking:
            do_Tracking();
            break;
    }
    if (state > Stopped) {
        prepare_for_next_peak();  // adjust phase as needed
    }

    idle();
}
