// web_interface.ino — phone-accessible Wi-Fi control panel for the cube.
//
// The ESP32 runs its own Wi-Fi ACCESS POINT (no router or internet needed):
// connect a phone to the "Cube-Control" network, then open the IP address
// printed on the Serial monitor (normally http://192.168.4.1).
//
// Design rules for this file:
//  * HTTP handlers must never call motor-control functions directly and
//    must never block.  They only read shared state (later phases will set
//    request flags that the main control loop acts on).
//  * The balancing loop in esp32_cube_enc.ino stays fully independent:
//    loop() just calls handleWebInterface(), which returns immediately
//    when no client is waiting.
//
// Only built-in ESP32 Arduino libraries are used here.
#include <WiFi.h>
#include <WebServer.h>

// Access-point credentials.  Change the password before real use;
// WPA2 requires it to be at least 8 characters long.
const char* WIFI_NAME = "Cube-Control";
const char* WIFI_PASSWORD = "poop1234";

// HTTP server on the standard port, so plain http://<ip> works.
WebServer webServer(80);

// GET /  — simple confirmation page (replaced by a dashboard in Phase 3).
void handleRoot() {
  webServer.send(200, "text/plain", "Cube web interface is running.");
}

// GET /api/state  — read-only telemetry snapshot as JSON.
//
// This handler only READS shared state; it never commands the motors.
// No locking is needed: handleWebInterface() is called from the same loop()
// as the balancing code, so this can never interrupt a control iteration
// mid-update — the values below are always from a completed iteration.
void handleApiState() {
  // Fixed stack buffer instead of String concatenation: bounded memory and
  // no heap fragmentation on a long-running controller.
  char json[512];
  snprintf(json, sizeof(json),
    "{"
      "\"robot_angleX\":%.3f,"
      "\"robot_angleY\":%.3f,"
      "\"gyroXfilt\":%.3f,"
      "\"gyroYfilt\":%.3f,"
      "\"gyroZ\":%.3f,"
      "\"motor1_speed\":%d,"
      "\"motor2_speed\":%d,"
      "\"motor3_speed\":%d,"
      "\"speed_X\":%.3f,"
      "\"speed_Y\":%.3f,"
      "\"vertical_vertex\":%s,"
      "\"vertical_edge\":%s,"
      "\"calibrated\":%s,"
      "\"calibrating\":%s,"
      "\"batt_voltage\":%.2f"
    "}",
    robot_angleX, robot_angleY,
    gyroXfilt, gyroYfilt, gyroZ,
    motor1_speed, motor2_speed, motor3_speed,
    speed_X, speed_Y,
    // JSON has no C-style booleans, so emit the literals true/false.
    vertical_vertex ? "true" : "false",
    vertical_edge   ? "true" : "false",
    calibrated      ? "true" : "false",
    calibrating     ? "true" : "false",
    batt_voltage);
  webServer.send(200, "application/json", json);
}

// Called once from setup(): bring up the access point, register routes,
// and start the HTTP server.
void startWebInterface() {
  WiFi.mode(WIFI_AP);                      // stand-alone AP, no router needed

  // softAP() returns false if the AP could not be started.  The most common
  // cause is a password shorter than the 8-character WPA2 minimum, which
  // makes the network silently never appear — so report failure loudly.
  if (!WiFi.softAP(WIFI_NAME, WIFI_PASSWORD)) {
    Serial.println("ERROR: Wi-Fi AP failed to start!"
                   "  (password must be at least 8 characters)");
    return;                                // no AP: skip starting the server
  }

  // Print the address the phone should open (the AP's own IP).
  Serial.print("Wi-Fi AP \"");
  Serial.print(WIFI_NAME);
  Serial.print("\" started.  Open http://");
  Serial.println(WiFi.softAPIP());

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/api/state", HTTP_GET, handleApiState);
  webServer.begin();
}

// Called every pass of loop(): service at most one pending HTTP request.
// handleClient() is non-blocking and returns immediately when idle, so it
// does not disturb the 15 ms balancing period.
void handleWebInterface() {
  webServer.handleClient();
}
