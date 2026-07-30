# Self-Balancing-Cube

**UPDATE (2024-07-14)**

New code is in **esp32_cube_enc** folder. I changed sensor orientation. So, if you make my cube before, to use this code, you need to reprint one part.

Or you can print redesigned cube https://www.thingiverse.com/thing:6695891

<img src="/pictures/cube2.jpg" alt="Cubli"/>

The red connections you see in the schematic must be connected!

If something doesn't work, try the motors test sketch. It tests all motors, rotation directions and encoders.

Folow this video https://youtu.be/ZU0oTBRDgOE

---

ESP32, MPU6050, Nidec 24H brushless motors, 500 mAh LiPo battery.

## Wi-Fi web interface

The cube hosts its own Wi-Fi access point, so no router or internet is
needed. Connect a phone to the **Cube-Control** network and open
**http://192.168.4.1**.

The dashboard shows live tilt on an attitude target (the outer ring is the
±7° angle at which balancing disengages), the three motor speeds, battery
voltage, and status. From it you can:

- **SAFE STOP / ARM / DISARM** — stop the motors and keep them stopped
- **Calibrate** — start, capture each pose, save to EEPROM
- **Tune gains** — edit K1–K4, zK2, zK3 and eK1–eK4 live, with validation
  and limits. Changes apply immediately but are only written to EEPROM when
  you press Save. There is also a Restore Defaults button.

Set your own access-point password in `esp32_cube_enc/web_interface.cpp`
(`WIFI_PASSWORD`). WPA2 requires at least 8 characters — a shorter one
means the network never starts.

Bluetooth has been removed: it was 40% of the firmware image, the web
interface replaced everything it did, and Espressif rates a simultaneous
SoftAP + Bluetooth Classic as unstable on the ESP32's shared radio.

## Self-righting ("jump up onto an edge") — tried, and why it doesn't work

A jump-up was implemented and tested on this cube: spin a reaction wheel to
high speed, brake it hard, and let the transferred angular momentum tip the
cube from lying flat onto one of its edges. **It does not work with motor
braking on this hardware, and the reason is torque, not momentum.** The code
was removed again; it lives in the git history if you want it.

### What was measured

| | |
|---|---|
| Cube mass | 996 g |
| Flywheel | 74 g, 125 mm OD |
| Peak wheel speed | 420 encoder counts per 15 ms loop |
| Time to reach that peak | 2402 ms |
| Cube rotation when braked | **0.1°** (i.e. none), by gyro integration |

Both the driver's `BRAKE` input and active reverse-driving of the motor were
tried, at brake durations from 60 to 250 ms. All gave the same result.

### Why

Lying flat, the cube's own weight holds it down with a leverage of half an
edge length — roughly **0.68 N·m**. Until the wheel's braking reaction
exceeds that, the cube does not tip a little; it does not tip **at all**.
The floor simply redistributes its normal force to absorb any smaller
couple. This is a threshold, not a proportional response, which is why no
combination of brake duration or target speed produced partial movement.

A motor short-circuit brake sheds the wheel's momentum on roughly the same
time constant it took to spin it up (~0.5–0.8 s here, inferred from the
2402 ms rise). That yields perhaps **0.05–0.3 N·m** — somewhere between 3×
and 20× short of the threshold. Worse, gravity cancels the impulse within
about 130 ms, and in that window an exponential brake delivers only 15–23%
of the stored momentum.

The stored momentum itself is *also* marginal — about 0.4–0.9× of what the
tip-up needs, once the encoder resolution is bounded by a power sanity check
(the pack simply cannot supply the current that a low count-per-rev would
imply). So the flywheel is not oversized either; it is just not the binding
constraint.

**Uncertainty, stated honestly:** the encoder's counts-per-revolution and
the cube's edge length were never measured, so the wheel's true RPM is known
only to within about a factor of two. The conclusion is robust to that — the
torque deficit does not close anywhere in the plausible range — but the
individual numbers above should be read as ranges, not measurements.

### What would actually change it

Ranked by leverage:

1. **A mechanical brake.** A servo barrier or solenoid pawl arresting the
   wheel in under ~10 ms gives 5–13 N·m, clearing the threshold by an order
   of magnitude. This is what the ETH Zurich Cubli uses, and why. It is the
   only fix that works with these motors.
2. **Much larger motors, no brake at all.** The approach taken by the
   Wheelbot (Geist et al., ICRA 2022): pick motors whose *continuous* torque
   exceeds the tipping threshold outright. That is roughly 30× a Nidec 24H.
3. **Lower the threshold instead of raising the torque.** The threshold is
   set by the support half-width, not by anything fundamental. Resting the
   cube on a narrow central ridge collapses it toward zero. **This is the
   cheapest decisive experiment here** — if the cube tips off a ridge but
   not off a flat face, the whole analysis is confirmed for the price of one
   printed part. Pre-tilting has the same effect: starting 20° up drops the
   threshold to ~0.41 N·m.
4. **More wheel momentum** (rim-weighted flywheel, higher pack voltage —
   brake torque is proportional to wheel speed, so voltage genuinely helps).
   Needed anyway, but on its own it does not clear the gate.

If you want to re-open this, the three measurements worth taking first are:
the encoder's counts per revolution (mark the wheel and count edges), the
wheel-speed decay curve through a brake event (this measures the brake time
constant directly), and the cube's edge length.

### Two firmware notes found along the way

Neither is fixed, because both are baked into the current tuning and
changing them would alter balancing behaviour:

- **`Motor*_control(0)` does not mean "stop".** It adds the measured wheel
  speed to the command (`sp = sp + motorN_speed`) before clamping, so once
  a wheel exceeds 255 counts/loop, commanding zero produces *full drive*.
  This is why the jump code drove the motor directly instead.
- **The gyro scale is inconsistent.** `gyroSens = 0` selects ±250 °/s
  (131 LSB per °/s), and the rate terms correctly divide by `131.0` — but
  the angle integration in `angle_calc()` divides by `65.536`, the ±500 °/s
  constant. The gyro half of the complementary filter therefore contributes
  twice the rotation it should. The accelerometer half corrects it in steady
  state, and `K1`/`K2` are tuned around the result, so fixing it would
  require retuning.

<img src="/pictures/cube1.jpg" alt="Self-Balancing-Cube"/>

<img src="/pictures/schematic.png" alt="Self-Balancing-Cube-Schematic"/>

About schematic:

Battery: 3S1P LiPo (11.1V). 
Buzzer: any 5V active buzzer.
Voltage regulator: any 5V regulator (7805).
All red connections not nescesary for this project! But if you are designing a PCB I recommend making these connections. Maybe I use encoders in the future, you will be able to use the new firmware without any changes.
 
How to build:

https://youtu.be/AJQZFHJzwt4

If something doesn't work, try the motors test sketch. It tests all motors, rotation directions and speeds. This helps you understand the problem is in software or in hardware.

You can also make this balancing cube with Arduino nano controller. All other parts remain the same.

<img src="/pictures/arduino_schematic.png" alt="Self-Balancing-Cube-Schematic"/>

In this version I make offsets setting procedure more simple. Calibrate from
the web dashboard: open the **Calibration** section, press **Start**, set the
cube on a vertex and press **Capture pose**, then set it on an edge and
capture again. The second capture writes the offsets to EEPROM automatically.
The dashboard shows the raw accelerometer counts and tells you whether each
pose was accepted.

The same `c+` / `c-` commands still work over **USB serial** as a wired
fallback, for when the access point is unavailable. Calibration is refused
while the cube is actively balancing — disarm first.

ESP32 version also has an updated balancing point setting procedure. Important! In this video you can learn how to set the balancing points:

https://youtu.be/Nkm9PoihZOI


