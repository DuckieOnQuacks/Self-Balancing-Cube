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
const char* WIFI_PASSWORD = "poop";

// HTTP server on the standard port, so plain http://<ip> works.
WebServer webServer(80);

// GET /  — simple confirmation page (replaced by a dashboard in Phase 3).
void handleRoot() {
  webServer.send(200, "text/plain", "Cube web interface is running.");
}

// Called once from setup(): bring up the access point, register routes,
// and start the HTTP server.
void startWebInterface() {
  WiFi.mode(WIFI_AP);                      // stand-alone AP, no router needed
  WiFi.softAP(WIFI_NAME, WIFI_PASSWORD);

  // Print the address the phone should open (the AP's own IP).
  Serial.print("Wi-Fi AP \"");
  Serial.print(WIFI_NAME);
  Serial.print("\" started.  Open http://");
  Serial.println(WiFi.softAPIP());

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.begin();
}

// Called every pass of loop(): service at most one pending HTTP request.
// handleClient() is non-blocking and returns immediately when idle, so it
// does not disturb the 15 ms balancing period.
void handleWebInterface() {
  webServer.handleClient();
}
