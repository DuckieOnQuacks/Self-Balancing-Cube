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


