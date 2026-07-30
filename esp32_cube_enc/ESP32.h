// ESP32 pin assignments.  Keeping these in one file makes it easier to
// adapt the firmware if the PCB wiring changes.
//
// This header is shared across separate .cpp translation units (rather than
// being concatenated Arduino-.ino-style into one file), so it only declares
// things: types, macros, and `extern` globals.  The actual storage for the
// globals below is defined once, in esp32_cube_enc.cpp.
#include <Arduino.h>
#include <FastLED.h>

#define BUZZER      27
#define VBAT        34
#define INT_LED     2

#define BRAKE       26       // Motor-driver brake/enable input

#define DIR2        15
#define ENC2_1      13
#define ENC2_2      14
#define PWM2        25
#define PWM2_CH     0

#define DIR3        5
#define ENC3_1      16
#define ENC3_2      17
#define PWM3        18
#define PWM3_CH     2

#define DIR1        4
#define ENC1_1      35
#define ENC1_2      33
#define PWM1        32
#define PWM1_CH     1

#define TIMER_BIT   8        // 8-bit PWM: values range from 0 to 255
#define BASE_FREQ   20000    // 20 kHz PWM, above the audible motor range

#define MPU6050       0x68   // I2C device address
#define ACCEL_CONFIG  0x1C   // Accelerometer configuration register
#define GYRO_CONFIG   0x1B   // Gyroscope configuration register
#define PWR_MGMT_1    0x6B   // Power-management register
#define PWR_MGMT_2    0x6C

#define accSens 0            // 0 = ±2 g, 1 = ±4 g, 2 = ±8 g, 3 = ±16 g
#define gyroSens 0           // 0 = ±250°/s, 1 = ±500°/s, 2 = ±1000°/s, 3 = ±2000°/s

// Was 64.  The offsets struct occupies bytes 0-27; the tuning gains are
// stored after it, so the allocation needs to be larger.  The ESP32 EEPROM
// library is NVS-backed and expands in place, so existing saved calibration
// data is preserved when this grows.
#define EEPROM_SIZE   128

// Tuning gains are saved after the offsets struct.
#define GAINS_EEPROM_ADDR 32
#define NUM_GAINS         10
#define GAINS_ID          0x6A   // marks a valid saved gain set
struct GainsObj {
  int   ID;
  float v[NUM_GAINS];
};

#define LED_PIN       19     // Pin that connects to WS2812B
#define NUM_PIXELS    3      // The number of LEDs (pixels) on WS2812B

// Complementary-filter weight.  The gyro responds quickly, while the
// accelerometer slowly corrects gyro drift using the direction of gravity.
extern float Gyro_amount;

// These flags describe the cube's current operating state.
// Balancing is allowed only after calibration and when a valid upright pose
// has been detected.
extern bool vertical_vertex;
extern bool vertical_edge;
extern bool calibrating;
extern bool vertex_calibrated;
extern bool calibrated;
extern bool calibrated_leds;

// PID-like balancing gains for vertex mode:
// K1 = angle, K2 = angular rate, K3 = translational speed, K4 = motor speed.
// The Z axis uses separate gains because it is controlled differently.
extern float K1;
extern float K2;
extern float K3;
extern float K4;
extern float zK2;
extern float zK3;

// Gains used while balancing on an edge.  Edge mode uses motor 3 directly.
extern float eK1;
extern float eK2;
extern float eK3;
extern float eK4;

extern int loop_time;        // Main control period in milliseconds

// Accelerometer offsets measured in the two calibration poses.
// The vertex and edge poses have different gravity vectors, so each gets
// its own reference values.
struct OffsetsObj {
  int ID;
  float acXv;
  float acYv;
  float acZv;
  float acXe;
  float acYe;
  float acZe;
};
extern OffsetsObj offsets;

extern float alpha;          // Low-pass filter for gyro rate used by control

// Raw and corrected MPU6050 readings.
extern int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
extern float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
extern float speed_X, speed_Y;

// Gyro bias found during startup while the cube is stationary.
extern int16_t  GyZ_offset;
extern int16_t  GyY_offset;
extern int16_t  GyX_offset;
extern int32_t  GyZ_offset_sum;
extern int32_t  GyY_offset_sum;
extern int32_t  GyX_offset_sum;

extern float robot_angleX, robot_angleY; // Fused orientation estimates
extern float Acc_angleX, Acc_angleY;     // Orientation estimated from gravity only
extern int32_t motors_speed_X;            // Integrated speed feedback in X/Y/Z
extern int32_t motors_speed_Y;
extern int32_t motors_speed_Z;

// Two independent timers are used: one for the fast balancing loop and one
// for slower battery/calibration status messages.
extern long currentT, previousT_1, previousT_2;

// Battery voltage computed in the slow status loop (see battVoltage()).
// Stored here so the web interface can report it through /api/state.
extern float batt_voltage;

// --- Web command interface (see web_interface.cpp) ---------------------
// HTTP handlers never touch the motors.  They only store a request here,
// and the main control loop acts on it at the start of a control cycle.
#define WEB_CMD_NONE    0
#define WEB_CMD_STOP    1    // stop now, engage brake, and disarm
#define WEB_CMD_DISARM  2    // disarm (same effect; separate for clarity)
#define WEB_CMD_ARM     3    // allow balancing again
#define WEB_CMD_CAL_START   4  // begin calibration (same as Bluetooth "c+")
#define WEB_CMD_CAL_CAPTURE 5  // record the current pose (same as "c-")
#define WEB_CMD_CAL_SAVE    6  // write the offsets to EEPROM
#define WEB_CMD_GAINS_SAVE  7  // write the current gains to EEPROM
// volatile because it is written by an HTTP handler and read by the loop.
extern volatile uint8_t web_cmd_pending;

// Master enable for balancing.  Defaults to true so the cube behaves exactly
// as before unless the web interface explicitly disarms it.  While false,
// the control loop takes its normal "not balancing" path: no drive, brake
// engaged.  Only an ARM command or a restart clears it.
extern bool armed;

// Encoder counts are modified inside interrupt handlers, so they must be
// volatile.  The main loop periodically copies and resets them.
extern volatile int  enc_count1, enc_count2, enc_count3;
extern int16_t motor1_speed;
extern int16_t motor2_speed;
extern int16_t motor3_speed;

// Objects defined in esp32_cube_enc.cpp, used from functions.cpp.
extern CRGB leds[NUM_PIXELS];

// Result of the most recent calibration capture, shown on the dashboard.
// Bluetooth used to carry this feedback; now it travels in /api/state.
// Always points at a string literal, so it needs no allocation and is
// safe to embed in JSON (no quotes or backslashes in any of the values).
extern const char* cal_result;

// Entry points implemented in web_interface.cpp.  Every .ino file used to be
// concatenated into one translation unit, so Arduino auto-generated forward
// declarations for whatever a later file defined.  Now that each file is
// compiled on its own, that no longer happens - so every function called
// from a *different* .cpp file needs an explicit prototype here.
void startWebInterface();
void handleWebInterface();
void loadGains();
void saveGains();
bool balancingActive();

// Entry points implemented in functions.cpp.
void writeTo(byte device, byte address, byte value);
void beep();
void save();
void angle_setup();
void angle_calc();
void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z);
void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3);
void battVoltage(double voltage);
void pwmSet(uint8_t pin, uint32_t value);
void Motor1_control(int sp);
void Motor2_control(int sp);
void Motor3_control(int sp);
void ENC1_READ();
void ENC2_READ();
void ENC3_READ();
void calStart();
void calCapture();
int Tuning();
