// ESP32 pin assignments.  Keeping these in one file makes it easier to
// adapt the firmware if the PCB wiring changes.
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
float Gyro_amount = 0.996;

// These flags describe the cube's current operating state.
// Balancing is allowed only after calibration and when a valid upright pose
// has been detected.
bool vertical_vertex = false;
bool vertical_edge = false;
bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;

// PID-like balancing gains for vertex mode:
// K1 = angle, K2 = angular rate, K3 = translational speed, K4 = motor speed.
// The Z axis uses separate gains because it is controlled differently.
float K1 = 180;
float K2 = 30.00; 
float K3 = 1.6;
float K4 = 0.008;
float zK2 = 8.00;
float zK3 = 0.30;

// Gains used while balancing on an edge.  Edge mode uses motor 3 directly.
float eK1 = 190;
float eK2 = 31.00; 
float eK3 = 2.5;
float eK4 = 0.014;

int loop_time = 15;          // Main control period in milliseconds

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
OffsetsObj offsets;

float alpha = 0.7;           // Low-pass filter for gyro rate used by control

// Raw and corrected MPU6050 readings.
int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
float speed_X, speed_Y;

// Gyro bias found during startup while the cube is stationary.
int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;
int32_t  GyX_offset_sum = 0;

float robot_angleX, robot_angleY; // Fused orientation estimates
float Acc_angleX, Acc_angleY;     // Orientation estimated from gravity only
int32_t motors_speed_X;            // Integrated speed feedback in X/Y/Z
int32_t motors_speed_Y;
int32_t motors_speed_Z;

// Two independent timers are used: one for the fast balancing loop and one
// for slower battery/calibration status messages.
long currentT, previousT_1, previousT_2;

// Battery voltage computed in the slow status loop (see battVoltage()).
// Stored here so the web interface can report it through /api/state.
float batt_voltage = 0;

// --- Web command interface (see web_interface.ino) ---------------------
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
volatile uint8_t web_cmd_pending = WEB_CMD_NONE;

// Entry points implemented in web_interface.ino.  Declared explicitly
// because Arduino's automatic prototype generation stops emitting
// declarations for functions defined after the dashboard's large raw-string
// literal, which leaves setup()/loop() unable to see these.
void startWebInterface();
void handleWebInterface();

// Master enable for balancing.  Defaults to true so the cube behaves exactly
// as before unless the web interface explicitly disarms it.  While false,
// the control loop takes its normal "not balancing" path: no drive, brake
// engaged.  Only an ARM command or a restart clears it.
bool armed = true;

// Encoder counts are modified inside interrupt handlers, so they must be
// volatile.  The main loop periodically copies and resets them.
volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int16_t motor1_speed;         
int16_t motor2_speed;         
int16_t motor3_speed;     
