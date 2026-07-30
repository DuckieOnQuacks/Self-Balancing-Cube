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

// The dashboard page.  Stored in flash (PROGMEM) rather than RAM, and sent
// with send_P() so it is streamed straight from flash — no RAM copy, no
// String building.  Everything is inlined because the phone is connected to
// the cube's own access point and has no internet access to fetch assets.
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cube Control</title><style>
body{font-family:system-ui,sans-serif;background:#14171c;color:#e8eaed;
margin:0;padding:16px;-webkit-text-size-adjust:100%}
h1{font-size:1.1rem;margin:0 0 12px;letter-spacing:.02em}
.g{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}
.c{background:#1e232b;border-radius:8px;padding:10px 12px}
.k{font-size:.7rem;text-transform:uppercase;color:#9aa3af;letter-spacing:.05em}
.v{font-size:1.35rem;font-variant-numeric:tabular-nums;margin-top:2px}
.f{grid-column:1/-1}
.p{display:inline-block;padding:3px 9px;border-radius:999px;font-size:.85rem;
background:#3a4150}
.on{background:#1c7a3e}.off{background:#8a2b2b}
#stop{width:100%;padding:20px;font-size:1.3rem;font-weight:700;color:#fff;
background:#c62828;border:0;border-radius:10px;letter-spacing:.05em}
#stop:active{background:#8e1f1f}
#s{font-size:.75rem;color:#9aa3af;margin-top:10px;text-align:center}
</style></head><body>
<h1>Self-Balancing Cube</h1>
<div class="g">
<div class="c"><div class="k">Angle X</div><div class="v" id="ax">-</div></div>
<div class="c"><div class="k">Angle Y</div><div class="v" id="ay">-</div></div>
<div class="c"><div class="k">Motor 1</div><div class="v" id="m1">-</div></div>
<div class="c"><div class="k">Motor 2</div><div class="v" id="m2">-</div></div>
<div class="c"><div class="k">Motor 3</div><div class="v" id="m3">-</div></div>
<div class="c"><div class="k">Battery</div><div class="v" id="bv">-</div></div>
<div class="c f"><div class="k">Status</div><div class="v">
<span class="p" id="cal">calibration</span> <span class="p" id="mode">mode</span>
</div></div>
</div>
<button id="stop">SAFE STOP</button>
<div id="s">connecting...</div>
<script>
var busy=false;                        // one request at a time: the ESP32
                                       // WebServer serves a single client,
                                       // so never let polls pile up
function pill(el,on,txt){el.textContent=txt;el.className='p '+(on?'on':'off');}
function poll(){
 if(busy)return; busy=true;
 fetch('/api/state',{cache:'no-store'}).then(function(r){return r.json();})
 .then(function(d){
  document.getElementById('ax').textContent=d.robot_angleX.toFixed(2)+'°';
  document.getElementById('ay').textContent=d.robot_angleY.toFixed(2)+'°';
  document.getElementById('m1').textContent=d.motor1_speed;
  document.getElementById('m2').textContent=d.motor2_speed;
  document.getElementById('m3').textContent=d.motor3_speed;
  document.getElementById('bv').textContent=d.batt_voltage.toFixed(2)+' V';
  pill(document.getElementById('cal'),d.calibrated,
       d.calibrating?'CALIBRATING':(d.calibrated?'CALIBRATED':'NOT CALIBRATED'));
  var m=d.vertical_vertex?'VERTEX':(d.vertical_edge?'EDGE':'IDLE');
  pill(document.getElementById('mode'),d.vertical_vertex||d.vertical_edge,m);
  document.getElementById('s').textContent='live';
 }).catch(function(){document.getElementById('s').textContent='disconnected';})
 .then(function(){busy=false;});
}
setInterval(poll,300);                 // 300 ms refresh (spec: 250-500 ms)
poll();
document.getElementById('stop').onclick=function(){
 // Sends a command request only; the main control loop acts on it.
 fetch('/api/command',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:'cmd=stop'})
 .then(function(r){document.getElementById('s').textContent=
   r.ok?'STOP sent':'STOP failed ('+r.status+')';})
 .catch(function(){document.getElementById('s').textContent='STOP failed';});
};
</script></body></html>)rawliteral";

// GET /  — serve the dashboard straight from flash.
void handleRoot() {
  webServer.send_P(200, "text/html", DASHBOARD_HTML);
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
