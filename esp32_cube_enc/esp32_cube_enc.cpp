#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>

// Storage for the globals declared `extern` in ESP32.h.  This is the one
// translation unit that defines them; every other file just links against
// these.  (Split out of ESP32.h itself so including that header from
// multiple .cpp files doesn't define each variable more than once.)
float Gyro_amount = 0.996;

bool vertical_vertex = false;
bool vertical_edge = false;
bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;

float K1 = 180;
float K2 = 30.00;
float K3 = 1.6;
float K4 = 0.008;
float zK2 = 8.00;
float zK3 = 0.30;

float eK1 = 190;
float eK2 = 31.00;
float eK3 = 2.5;
float eK4 = 0.014;

int loop_time = 15;

OffsetsObj offsets;

float alpha = 0.7;

int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
float speed_X, speed_Y;

int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;
int32_t  GyX_offset_sum = 0;

float robot_angleX, robot_angleY;
float Acc_angleX, Acc_angleY;
int32_t motors_speed_X;
int32_t motors_speed_Y;
int32_t motors_speed_Z;

long currentT, previousT_1, previousT_2;

float batt_voltage = 0;

volatile uint8_t web_cmd_pending = WEB_CMD_NONE;

bool armed = true;

volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int16_t motor1_speed;
int16_t motor2_speed;
int16_t motor3_speed;

CRGB leds[NUM_PIXELS];
const char* cal_result = "";   // see ESP32.h

void setup() {
  // ---- MOTOR SAFETY: this block must run before anything slow. ----
  // Until LEDC is attached, the PWM pins float, and this driver hardware
  // treats a floating/LOW PWM input as FULL DRIVE (the PWM is inverted:
  // 255 = stopped).  Engage the brake and force all three PWM outputs to
  // the stopped level immediately, so the motors cannot run away during
  // the LED animation and gyro-bias measurement below.
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, LOW);   // brake engaged; the balancing loop releases
                              // it only when an upright pose is active
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  // ESP32 core 3.x API: ledcAttach() replaces ledcSetup()+ledcAttachPin()
  // and manages the channel internally; PWM is now addressed by pin.
  bool pwm_ok = ledcAttach(PWM1, BASE_FREQ, TIMER_BIT);
  pwm_ok = ledcAttach(PWM2, BASE_FREQ, TIMER_BIT) && pwm_ok;
  pwm_ok = ledcAttach(PWM3, BASE_FREQ, TIMER_BIT) && pwm_ok;
  Motor1_control(0);          // duty 255 = stopped (inverted PWM)
  Motor2_control(0);
  Motor3_control(0);
  // ---- end motor safety block ----

  // Start the two serial interfaces: USB serial is useful for diagnostics,
  // while Bluetooth is used to tune gains and run calibration.
  Serial.begin(115200);
  // If any LEDC attach failed, the PWM pins are still floating and the
  // motors are NOT safe: report it loudly and keep the brake engaged.
  if (!pwm_ok)
    Serial.println("ERROR: motor PWM attach failed - outputs unconfigured!");
  EEPROM.begin(EEPROM_SIZE);

  // The three WS2812B LEDs provide visual feedback during startup,
  // calibration, and low-battery warnings.
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS);  // GRB ordering is typical

  pinMode(BUZZER, OUTPUT);

  // Cycle through red, green, and blue to show that the LEDs are working.
  for (int i=0;i<=255;i+=10) {
    leds[0] = CRGB(i, 0, 0);
    leds[1] = CRGB(i, 0, 0);
    leds[2] = CRGB(i, 0, 0);
    FastLED.show();
    delay(5);
  }
  delay(300);
  for (int i=0;i<=255;i+=10) {
    leds[0] = CRGB(0, i, 0);
    leds[1] = CRGB(0, i, 0);
    leds[2] = CRGB(0, i, 0);
    FastLED.show();
    delay(5);
  }
  delay(300);
  for (int i=0;i<=255;i+=10) {
    leds[0] = CRGB(0, 0, i);
    leds[1] = CRGB(0, 0, i);
    leds[2] = CRGB(0, 0, i);
    FastLED.show();
    delay(5);
  }
  delay(300);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();
  
  // Encoder inputs: both channels of each encoder trigger the same
  // quadrature decoder on every edge.  (Direction/PWM outputs were already
  // configured in the motor-safety block at the top of setup().)
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);

  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);

  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);

  // A valid ID means that accelerometer offsets were previously saved.
  // EEPROM data survives power cycles, so calibration is normally needed only
  // after changing the hardware or clearing the EEPROM.
  EEPROM.get(0, offsets);
  if (offsets.ID == 96)
    calibrated = true;

  // Restore any tuning gains saved from the web interface.  If none were
  // ever saved, the compiled-in defaults stay in force.
  loadGains();

  delay(200);
  // Configure the MPU6050 and measure the gyro's stationary bias.
  angle_setup();

  // Start the Wi-Fi access point and web server (see web_interface.cpp).
  // Done last so it cannot disturb the gyro-bias measurement above.
  startWebInterface();
}

void loop() {
  currentT = millis();
  // This is the fast control loop.  It is deliberately time-based rather
  // than delay-based so sensor and motor work can run at a stable period.
  if (currentT - previousT_1 >= loop_time) {
    Tuning();
    angle_calc();

    // Encoder interrupts accumulate counts between control-loop iterations.
    // Copy each interval's count as a speed estimate, then start a new one.
    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;
    // Convert the three motor speeds into the cube's X/Y motion components.
    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;
    
    // Act on any command left by the web interface.  This runs after
    // angle_calc() (which can set the pose flags) and before the balancing
    // branches below, so a stop cannot be undone within the same iteration.
    // The motors are never touched here: clearing these flags routes the
    // control loop into its existing "not balancing" branch, which stops
    // the drive and engages the brake.
    switch (web_cmd_pending) {
      case WEB_CMD_STOP:
      case WEB_CMD_DISARM:
        armed = false;              // blocks balancing until re-armed
        vertical_vertex = false;    // forget the current upright pose
        vertical_edge = false;
        break;
      case WEB_CMD_ARM:
        armed = true;
        // Leave the pose flags cleared: angle_calc() re-detects an upright
        // pose only within its tight angle window, so arming can never make
        // the cube jump straight back into balancing from a stale pose.
        vertical_vertex = false;
        vertical_edge = false;
        break;
      // Calibration commands run the same functions as the Bluetooth "c+"
      // and "c-" commands.  Each is refused while the cube is actively
      // balancing; the HTTP handler checks this too, but it is re-checked
      // here because the cube may have started balancing in between.
      case WEB_CMD_CAL_START:
        if (!balancingActive() && !calibrating) calStart();
        break;
      case WEB_CMD_CAL_CAPTURE:
        // Only meaningful once calibration has been started.
        if (!balancingActive() && calibrating) calCapture();
        break;
      case WEB_CMD_CAL_SAVE:
        // Commit the offsets recorded so far.  A normal two-pose
        // calibration already saves automatically after the edge pose;
        // this is for saving explicitly from the dashboard.
        if (!balancingActive() && calibrating && vertex_calibrated) save();
        break;
      case WEB_CMD_GAINS_SAVE:
        // Persist the gains currently in use.  Editing gains from the
        // dashboard only changes RAM; the EEPROM write happens here, once
        // per explicit Save, so tuning never wears out the flash.
        saveGains();
        break;
    }
    web_cmd_pending = WEB_CMD_NONE; // request consumed

    // Vertex mode controls two tilt axes and the common Z rotation axis.
    // "armed" gates both balancing branches; when false the else branch
    // below stops the motors and engages the brake.
    if (armed && vertical_vertex && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroY = GyY / 131.0;
      gyroZ = GyZ / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      
      // Each term counters a different part of the motion:
      // angle keeps the cube upright, gyro rate damps it, and the speed terms
      // reduce motion that would otherwise build up around the equilibrium.
      int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
      int pwm_Z = constrain(zK2 * gyroZ + zK3 * motors_speed_Z, -255, 255);

      // A small accumulated speed correction acts like an integral term.
      motors_speed_X += speed_X / 5; 
      motors_speed_Y += speed_Y / 5;
      // Transform desired X/Y/Z forces into the three motor commands.
      XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
    } else if (armed && vertical_edge && calibrated && !calibrating) {
      // In edge mode, only motor 3 is used to correct the detected tilt.
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      
      int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);
      
      motors_speed_X += motor3_speed / 5;
      Motor3_control(pwm_X);
    } else {
      // If the cube is not in a recognized balancing pose, stop applying
      // drive and engage the brake.  This protects the motors during setup or
      // after the cube has fallen.
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0;
    }
    previousT_1 = currentT;
  }
  
  // Slow status loop: check battery voltage and blink LEDs until calibration
  // has been completed.
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / 204); // value 204 must be selected by measuring battery voltage!
    if (!calibrated && !calibrating) {
      Serial.println("Not calibrated yet - use the web dashboard "
                     "(http://192.168.4.1) or send c+ / c- over USB serial.");
      if (!calibrated_leds) {
        leds[0] = CRGB(0, 255, 0);
        leds[1] = CRGB(0, 255, 0);
        leds[2] = CRGB(0, 255, 0);
        FastLED.show();
        calibrated_leds = true; 
      } else {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        leds[2] = CRGB::Black;
        FastLED.show();
        calibrated_leds = false; 
      }
    }
    previousT_2 = currentT;
  }

  // Service pending HTTP clients (web_interface.cpp).  Non-blocking: it
  // returns immediately when no client is connected, so the timed balancing
  // loop above is unaffected.
  handleWebInterface();
}
