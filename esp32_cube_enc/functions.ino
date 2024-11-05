void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

void save() {
    EEPROM.put(0, offsets);
    EEPROM.commit();
    EEPROM.get(0, offsets);
    if (offsets.ID == 96) calibrated = true;
    calibrating = false;
    SerialBT.println("Calibrating off.");
    beep();
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);
  
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
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true); 
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  if (abs(AcX) < 2000) {
    AcXc = AcX - offsets.acXv;
    AcYc = AcY - offsets.acYv;
    AcZc = AcZ - offsets.acZv;
  } else {
    AcXc = AcX - offsets.acXe;
    AcYc = AcY - offsets.acYe;
    AcZc = AcZ - offsets.acZe;
  }
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  robot_angleY += GyY * loop_time / 1000 / 65.536;
  Acc_angleY = atan2(AcXc, -AcZc) * 57.2958;
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  robot_angleX += GyX * loop_time / 1000 / 65.536;
  Acc_angleX = -atan2(AcYc, -AcZc) * 57.2958;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_vertex = true;
  } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_edge = true;
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_vertex) {
    vertical_vertex = false;
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_edge) {
    vertical_edge = false;
  }
}

void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z) {
  int16_t m1 = round((0.5 * pwm_X - 0.866 * pwm_Y) / 1.37 + pwm_Z);  
  int16_t m2 = round((0.5 * pwm_X + 0.866 * pwm_Y) / 1.37 + pwm_Z);
  int16_t m3 = -pwm_X / 1.37 + pwm_Z;  
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3) {
  speed_X = ((in_speed3 - (in_speed2 + in_speed1) * 0.5) * 0.5) * 1.81;
  speed_Y = -(-0.866 * (in_speed2 - in_speed1)) / 1.1;
}

void battVoltage(double voltage) {
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void Motor1_control(int sp) {
  sp = sp + motor1_speed;
  if (sp < 0) 
    digitalWrite(DIR1, LOW);
  else 
    digitalWrite(DIR1, HIGH);
  pwmSet(PWM1_CH, 255 - abs(sp));
}

void Motor2_control(int sp) {
  sp = sp + motor2_speed;
  if (sp < 0) 
    digitalWrite(DIR2, LOW);
  else 
    digitalWrite(DIR2, HIGH);
  pwmSet(PWM2_CH, 255 - abs(sp));
}

void Motor3_control(int sp) {
  sp = sp + motor3_speed;
  if (sp < 0) 
    digitalWrite(DIR3, LOW);
  else 
    digitalWrite(DIR3, HIGH);
  pwmSet(PWM3_CH, 255 - abs(sp));
}

void ENC1_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}

void ENC2_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}

void ENC3_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}

int Tuning() {
  if (!SerialBT.available())  return 0;
  char param = SerialBT.read();               // get parameter byte
  if (!SerialBT.available()) return 0;
  char cmd = SerialBT.read();                 // get command byte
  switch (param) {
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        SerialBT.println("Calibrating on.");
        SerialBT.println("Set the cube on vertex...");
        leds[0] = CRGB(250, 250, 0);
        leds[1] = CRGB(250, 250, 0);
        leds[2] = CRGB(250, 250, 0);
        FastLED.show();
      }
      if (cmd == '-' && calibrating)  {
        SerialBT.print("X: "); SerialBT.print(AcX); SerialBT.print(" Y: "); SerialBT.print(AcY); SerialBT.print(" Z: "); SerialBT.println(AcZ + 16384);
        if (abs(AcX) < 2000 && abs(AcY) < 2000) {
          offsets.ID = 96;
          offsets.acXv = AcX;
          offsets.acYv = AcY;
          offsets.acZv = AcZ + 16384;
          SerialBT.println("Vertex OK.");
          SerialBT.println("Set the cube on edge...");
          vertex_calibrated = true;
          leds[0] = CRGB(0, 250, 250);
          leds[1] = CRGB(0, 250, 250);
          leds[2] = CRGB(0, 250, 250);
          FastLED.show();
          beep();
        } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000 && vertex_calibrated) {
          SerialBT.print("X: "); SerialBT.print(AcX); SerialBT.print(" Y: "); SerialBT.print(AcY); SerialBT.print(" Z: "); SerialBT.println(AcZ + 16384);
          SerialBT.println("Edge OK.");
          offsets.acXe = AcX;
          offsets.acYe = AcY;
          offsets.acZe = AcZ + 16384;
          leds[0] = CRGB::Black;
          leds[1] = CRGB::Black;
          leds[2] = CRGB::Black;
          FastLED.show();
          save();
        } else {
          SerialBT.println("The angles are wrong!!!");
          beep();
          beep();
        }
      }
      break;              
   }
   return 1;
}
