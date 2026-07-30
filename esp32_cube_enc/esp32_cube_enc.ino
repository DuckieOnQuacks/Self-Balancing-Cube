#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include <FastLED.h>

BluetoothSerial SerialBT;
CRGB leds[NUM_PIXELS];

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
  SerialBT.begin("ESP32-Cube"); // Bluetooth device name
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

  delay(200);
  // Configure the MPU6050 and measure the gyro's stationary bias.
  angle_setup();

  // Start the Wi-Fi access point and web server (see web_interface.ino).
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
    
    // Vertex mode controls two tilt axes and the common Z rotation axis.
    if (vertical_vertex && calibrated && !calibrating) {    
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
    } else if (vertical_edge && calibrated && !calibrating) {
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
      SerialBT.println("first you need to calibrate the balancing points...");
      Serial.println("first you need to calibrate the balancing points (over bluetooth)...");
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

  // Service pending HTTP clients (web_interface.ino).  Non-blocking: it
  // returns immediately when no client is connected, so the timed balancing
  // loop above is unaffected.
  handleWebInterface();
}
