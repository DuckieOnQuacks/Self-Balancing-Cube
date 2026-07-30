#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>

void writeTo(byte device, byte address, byte value) {
  // Helper for writing one byte to an MPU6050 register over I2C.
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
    // The active buzzer sounds while its pin is HIGH.
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

void save() {
    // ESP32 EEPROM emulation needs commit() to persist the new calibration.
    EEPROM.put(0, offsets);
    EEPROM.commit();
    EEPROM.get(0, offsets);
    if (offsets.ID == 96) calibrated = true;
    calibrating = false;
    Serial.println("Calibrating off.");
    beep();
}

void angle_setup() {
  // Wake the MPU6050 and select the measurement ranges from ESP32.h.
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);
  
  // Average 512 stationary samples on each gyro axis to measure its bias.
  beep();
  leds[2] = CRGB(0, 0, 200);
  FastLED.show();
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 9;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  beep();
  leds[2] = CRGB::Black;
  FastLED.show();

  leds[1] = CRGB(0, 0, 200);
  FastLED.show();
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 9;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  beep();
  leds[1] = CRGB::Black;
  FastLED.show();

  leds[0] = CRGB(0, 0, 200);
  FastLED.show();
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 9;
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
  beep();
  beep();
  leds[0] = CRGB::Black;
  FastLED.show();

  leds[0] = CRGB(255, 0, 0);
  leds[1] = CRGB(255, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  FastLED.show();
  delay(300);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();
  delay(150);
  leds[0] = CRGB(255, 0, 0);
  leds[1] = CRGB(255, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  FastLED.show();
  delay(300);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();
  delay(300);
}

void angle_calc() {
  // Read the three raw gyro registers (0x43 through 0x48).
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // Read the three raw accelerometer registers (0x3B through 0x40).
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true); 
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // Use the calibration values for the currently detected pose: small |AcX|
  // means vertex mode, while a larger |AcX| means edge mode.
  if (abs(AcX) < 2000) {
    AcXc = AcX - offsets.acXv;
    AcYc = AcY - offsets.acYv;
    AcZc = AcZ - offsets.acZv;
  } else {
    AcXc = AcX - offsets.acXe;
    AcYc = AcY - offsets.acYe;
    AcZc = AcZ - offsets.acZe;
  }
  // Remove the stationary gyro bias before integrating angular velocity.
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  // Integrate gyro rate into an angle.  65.536 converts raw readings at the
  // selected ±250°/s gyro range into degrees per second.
  robot_angleY += GyY * loop_time / 1000 / 65.536;
  Acc_angleY = atan2(AcXc, -AcZc) * 57.2958;
  // Combine fast gyro response with the accelerometer's long-term reference.
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  robot_angleX += GyX * loop_time / 1000 / 65.536;
  Acc_angleX = -atan2(AcYc, -AcZc) * 57.2958;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  // Recognize a stable upright vertex or edge.  The tight angle thresholds
  // prevent balancing from starting while the cube is being placed.
  if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_vertex = true;
  } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_edge = true;
  // Leaving the upright range disables the active balancing mode.
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_vertex) {
    vertical_vertex = false;
    yaw_rate_request = 0;   // see below
    yaw_rate_cmd = 0;
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_edge) {
    vertical_edge = false;
    // Losing the pose means the cube fell.  Forget any commanded spin: the
    // pose flags re-set on their own once it is upright again, so without
    // this the cube would resume balancing AND spin straight back up to the
    // old yaw rate - with whoever just picked it up still holding it.
    yaw_rate_request = 0;
    yaw_rate_cmd = 0;
  }
}

void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z) {
  // The three motors are arranged 120° apart.  Convert desired X/Y/Z axis
  // commands into individual motor commands using the inverse mix.
  // 0.5 and 0.866 are cos(60°) and sin(60°); the other factors compensate
  // for this hardware's motor geometry and scale.
  int16_t m1 = round((0.5 * pwm_X - 0.866 * pwm_Y) / 1.37 + pwm_Z);  
  int16_t m2 = round((0.5 * pwm_X + 0.866 * pwm_Y) / 1.37 + pwm_Z);
  int16_t m3 = -pwm_X / 1.37 + pwm_Z;  
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3) {
  // Forward mix: reconstruct X/Y movement from measured motor speeds.
  speed_X = ((in_speed3 - (in_speed2 + in_speed1) * 0.5) * 0.5) * 1.81;
  speed_Y = -(-0.866 * (in_speed2 - in_speed1)) / 1.1;
}

void battVoltage(double voltage) {
  // voltage is the ADC value divided by a board-specific scale factor.  The
  // buzzer warns while the battery reading is in the configured low range.
  batt_voltage = voltage;   // keep a copy for the web interface (/api/state)
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void pwmSet(uint8_t pin, uint32_t value) {
  // Write an 8-bit duty-cycle value to a PWM output.  ESP32 core 3.x
  // addresses LEDC by pin number (the old API used channel numbers).
  ledcWrite(pin, value);
}

void Motor1_control(int sp) {
  // Add the measured speed so the command includes motor-speed feedback.
  sp = sp + motor1_speed;
  // The caller's command is already limited to +/-255, but adding the
  // encoder feedback can push past it.  Without this clamp, 255 - abs(sp)
  // would go negative and wrap to a huge value in pwmSet's uint32_t duty
  // argument, producing an out-of-range duty instead of full braking.
  sp = constrain(sp, -255, 255);
  if (sp < 0)
    digitalWrite(DIR1, LOW);
  else 
    digitalWrite(DIR1, HIGH);
  // The driver uses inverted PWM: 255 is stopped and smaller values drive
  // the motor harder.
  pwmSet(PWM1, 255 - abs(sp));
}

void Motor2_control(int sp) {
  // Motor 2 uses the same direction and inverted-PWM convention as motor 1.
  sp = sp + motor2_speed;
  sp = constrain(sp, -255, 255);   // see Motor1_control
  if (sp < 0)
    digitalWrite(DIR2, LOW);
  else 
    digitalWrite(DIR2, HIGH);
  pwmSet(PWM2, 255 - abs(sp));
}

void Motor3_control(int sp) {
  // Motor 3 uses the same direction and inverted-PWM convention as motor 1.
  sp = sp + motor3_speed;
  sp = constrain(sp, -255, 255);   // see Motor1_control
  if (sp < 0)
    digitalWrite(DIR3, LOW);
  else 
    digitalWrite(DIR3, HIGH);
  pwmSet(PWM3, 255 - abs(sp));
}

void ENC1_READ() {
  // Quadrature decoder: remember the previous channel states and count only
  // valid clockwise/counter-clockwise transitions.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}

void ENC2_READ() {
  // Same quadrature decoder for motor 2's encoder.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}

void ENC3_READ() {
  // Same quadrature decoder for motor 3's encoder.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}

void calStart() {
  // Calibration is a two-step process: record a valid vertex first,
  // then record a valid edge and save both offsets to EEPROM.
  // Extracted from Tuning() so the Bluetooth and web interfaces run the
  // exact same calibration code rather than two copies that could drift.
  calibrating = true;
  cal_result = "Calibration started.";
  Serial.println("Calibrating on.");
  Serial.println("Set the cube on vertex...");
  leds[0] = CRGB(250, 250, 0);
  leds[1] = CRGB(250, 250, 0);
  leds[2] = CRGB(250, 250, 0);
  FastLED.show();
}

void calCapture() {
  // Record whichever pose the cube is currently in.  Shared by the
  // Bluetooth "c-" command and the web interface's Capture Pose button.
  Serial.print("X: "); Serial.print(AcX); Serial.print(" Y: "); Serial.print(AcY); Serial.print(" Z: "); Serial.println(AcZ + 16384);
  // Vertex pose: gravity is mostly along Z, so X and Y are near zero.
  if (abs(AcX) < 2000 && abs(AcY) < 2000) {
    offsets.ID = 96;
    offsets.acXv = AcX;
    offsets.acYv = AcY;
    offsets.acZv = AcZ + 16384;
    cal_result = "Vertex captured. Now set the cube on an edge.";
    Serial.println("Vertex OK.");
    Serial.println("Set the cube on edge...");
    vertex_calibrated = true;
    leds[0] = CRGB(0, 250, 250);
    leds[1] = CRGB(0, 250, 250);
    leds[2] = CRGB(0, 250, 250);
    FastLED.show();
    beep();
  // Edge pose: X has a characteristic gravity reading and Y remains
  // near zero.  Refuse edge calibration until the vertex was accepted.
  } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000 && vertex_calibrated) {
    Serial.print("X: "); Serial.print(AcX); Serial.print(" Y: "); Serial.print(AcY); Serial.print(" Z: "); Serial.println(AcZ + 16384);
    cal_result = "Edge captured. Calibration saved.";
    Serial.println("Edge OK.");
    offsets.acXe = AcX;
    offsets.acYe = AcY;
    offsets.acZe = AcZ + 16384;
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    leds[2] = CRGB::Black;
    FastLED.show();
    save();
  } else {
    // Neither pose matched.  This is the feedback Bluetooth used to carry;
    // it now reaches the dashboard through cal_result in /api/state.
    cal_result = "Pose not recognised - check the cube is settled and level.";
    Serial.println("The angles are wrong!!!");
    beep();
    beep();
  }
}

// True while the control loop is actively driving the motors to balance.
// Mirrors the balancing branch conditions in loop(); calibration commands
// are refused while this is true.
bool balancingActive() {
  return armed && (vertical_vertex || vertical_edge) && calibrated && !calibrating;
}

int Tuning() {
  // Wired fallback for calibration, over USB serial.  This used to be the
  // Bluetooth channel; it moved to Serial when Bluetooth was removed, so
  // there is still a way in if the Wi-Fi access point ever fails to start.
  // The protocol is unchanged: two characters, a parameter then an action.
  // c+ starts calibration, c- records the current pose.
  if (!Serial.available())  return 0;
  char param = Serial.read();                 // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                   // get command byte
  switch (param) {
    case 'a':
      // Arm/disarm over the wire.  This is the way back in when the Wi-Fi
      // access point fails to start, which is also the case that disarms the
      // cube automatically - without this it would be unusable, not merely
      // unstoppable.  Routed through the same command flag the dashboard
      // uses, so both paths behave identically; Tuning() runs earlier in the
      // same loop iteration that consumes it.
      // A stop already waiting must never be overwritten by an arm - the
      // same rule the HTTP handler enforces.
      if (cmd == '+' && web_cmd_pending != WEB_CMD_STOP) {
        web_cmd_pending = WEB_CMD_ARM;
        Serial.println("Arming.");
      } else if (cmd == '-') {
        web_cmd_pending = WEB_CMD_DISARM;
        Serial.println("Disarming.");
      }
      break;
    case 'c':
      // Refuse to calibrate while the motors are actively balancing - the
      // same rule the web interface enforces.  The old Bluetooth path was
      // missing this check, so a c+ mid-balance dropped the cube.
      if (balancingActive()) {
        Serial.println("Refused: cannot calibrate while balancing. Disarm first.");
        break;
      }
      if (cmd == '+' && !calibrating) {
        calStart();
      }
      if (cmd == '-' && calibrating)  {
        calCapture();
      }
      break;
   }
   return 1;
}
