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

// --- Tunable gains -----------------------------------------------------
// One table describes every gain: where it lives, the range the web
// interface will accept, and its compiled-in default.  Everything else
// (JSON output, validation, EEPROM save/load, Restore Defaults) loops over
// this table, so adding a gain means adding one row here and nothing else.
//
// The limits are generous around the defaults - they exist to stop a typo
// (a missing decimal point, a pasted value in the wrong field) from
// reaching the controller, not to constrain honest tuning.
struct GainDef {
  const char* name;
  float* ptr;         // the live variable used by the balancing loop
  float lo, hi;       // accepted range, inclusive
  float def;          // value restored by "Restore Defaults"
};
const GainDef GAIN_DEFS[] = {
  {"K1",  &K1,  0.0, 500.0, 180.0  },  // vertex: angle
  {"K2",  &K2,  0.0, 200.0,  30.0  },  // vertex: angular rate
  {"K3",  &K3,  0.0,  50.0,   1.6  },  // vertex: translational speed
  {"K4",  &K4,  0.0,   1.0,   0.008},  // vertex: motor speed
  {"zK2", &zK2, 0.0, 200.0,   8.0  },  // Z axis: angular rate
  {"zK3", &zK3, 0.0,  50.0,   0.30 },  // Z axis: motor speed
  {"eK1", &eK1, 0.0, 500.0, 190.0  },  // edge: angle
  {"eK2", &eK2, 0.0, 200.0,  31.0  },  // edge: angular rate
  {"eK3", &eK3, 0.0,  50.0,   2.5  },  // edge: motor 3 speed
  {"eK4", &eK4, 0.0,   1.0,   0.014},  // edge: motor speed
};
// Keep the table and the EEPROM record in step at compile time.
static_assert(sizeof(GAIN_DEFS) / sizeof(GAIN_DEFS[0]) == NUM_GAINS,
              "GAIN_DEFS and NUM_GAINS disagree");
// Prove the gain record cannot overlap the calibration offsets at address 0,
// and that both fit inside the EEPROM allocation.  Getting this wrong would
// silently corrupt calibration, so let the compiler check it.
static_assert(sizeof(OffsetsObj) <= GAINS_EEPROM_ADDR,
              "gains would overwrite the calibration offsets");
static_assert(GAINS_EEPROM_ADDR + sizeof(GainsObj) <= EEPROM_SIZE,
              "gains do not fit within EEPROM_SIZE");

// Restore saved gains at startup.  Anything missing, corrupt, or outside
// the accepted range is ignored so the compiled-in default stays in force.
void loadGains() {
  GainsObj g;
  EEPROM.get(GAINS_EEPROM_ADDR, g);
  if (g.ID != GAINS_ID) return;          // nothing saved yet
  for (int i = 0; i < NUM_GAINS; i++) {
    float v = g.v[i];
    if (isnan(v) || v < GAIN_DEFS[i].lo || v > GAIN_DEFS[i].hi) continue;
    *GAIN_DEFS[i].ptr = v;
  }
  Serial.println("Loaded saved tuning gains from EEPROM.");
}

// Write the live gains to EEPROM.  Called only from the control loop, in
// response to an explicit Save from the dashboard - never on every edit,
// because EEPROM/NVS has a finite number of write cycles.
void saveGains() {
  GainsObj g;
  g.ID = GAINS_ID;
  for (int i = 0; i < NUM_GAINS; i++) g.v[i] = *GAIN_DEFS[i].ptr;
  EEPROM.put(GAINS_EEPROM_ADDR, g);
  EEPROM.commit();
  Serial.println("Saved tuning gains to EEPROM.");
}

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
.ab{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px}
.b{padding:14px;font-size:1rem;font-weight:600;color:#e8eaed;background:#2c333f;
border:0;border-radius:8px}
.b:active{background:#3a4150}
.gl{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px}
.gr{display:flex;align-items:center;gap:6px}
.gr span{font-size:.8rem;color:#9aa3af;width:2.4em}
.gr input{flex:1;min-width:0;background:#14171c;color:#e8eaed;border:1px solid
#3a4150;border-radius:6px;padding:7px;font-size:.9rem;font-variant-numeric:
tabular-nums}
.gr input:invalid{border-color:#c62828}
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
<span class="p" id="armed">armed</span> <span class="p" id="cal">calibration</span>
<span class="p" id="mode">mode</span>
</div></div>
</div>
<button id="stop">SAFE STOP</button>
<div class="ab"><button class="b" id="arm">ARM</button>
<button class="b" id="disarm">DISARM</button></div>
<div class="c" style="margin-top:12px">
<div class="k">Calibration</div>
<div id="ch" style="font-size:.8rem;color:#9aa3af;margin:6px 0 8px">-</div>
<div class="ab" style="margin:0">
<button class="b" id="cstart">START</button>
<button class="b" id="ccap">CAPTURE POSE</button></div>
<button class="b" id="csave" style="width:100%;margin-top:8px">SAVE CALIBRATION</button>
</div>
<div class="c" style="margin-top:12px">
<div class="k">Gains</div>
<div id="gl" class="gl">loading...</div>
<div class="ab" style="margin-top:10px">
<button class="b" id="gapply">APPLY</button>
<button class="b" id="gdef">RESTORE DEFAULTS</button></div>
<button class="b" id="gsave" style="width:100%;margin-top:8px">SAVE GAINS TO EEPROM</button>
<div id="gm" style="font-size:.75rem;color:#9aa3af;margin-top:8px">
Changes apply immediately but are lost on restart until saved.</div>
</div>
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
  pill(document.getElementById('armed'),d.armed,d.armed?'ARMED':'DISARMED');
  pill(document.getElementById('cal'),d.calibrated,
       d.calibrating?'CALIBRATING':(d.calibrated?'CALIBRATED':'NOT CALIBRATED'));
  var m=d.vertical_vertex?'VERTEX':(d.vertical_edge?'EDGE':'IDLE');
  pill(document.getElementById('mode'),d.vertical_vertex||d.vertical_edge,m);
  // Tell the user which calibration step comes next.  Calibration is
  // blocked while balancing, so say that instead when it applies.
  var bal=d.armed&&(d.vertical_vertex||d.vertical_edge)&&d.calibrated
          &&!d.calibrating;
  document.getElementById('ch').textContent=
   bal?'Balancing - press DISARM before calibrating':
   (!d.calibrating?'Idle. Press START to begin.':
   (!d.vertex_calibrated?'Step 1: set cube on VERTEX, press CAPTURE POSE':
    'Step 2: set cube on EDGE, press CAPTURE POSE (saves automatically)'));
  document.getElementById('s').textContent='live';
 }).catch(function(){document.getElementById('s').textContent='disconnected';})
 .then(function(){busy=false;});
}
setInterval(poll,300);                 // 300 ms refresh (spec: 250-500 ms)
poll();
// Sends a command request only; the main control loop acts on it.
function send(cmd){
 fetch('/api/command',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:'cmd='+cmd})
 .then(function(r){document.getElementById('s').textContent=
   r.ok?cmd.toUpperCase()+' sent':cmd.toUpperCase()+' failed ('+r.status+')';})
 .catch(function(){document.getElementById('s').textContent=
   cmd.toUpperCase()+' failed';});
}
document.getElementById('stop').onclick=function(){send('stop');};
document.getElementById('disarm').onclick=function(){send('disarm');};
// Arming re-enables balancing, so require a deliberate confirmation.
document.getElementById('arm').onclick=function(){
 if(confirm('Arm the cube? Balancing will resume.'))send('arm');
};
// --- gain editing -----------------------------------------------------
// The form is built from /api/gains so the firmware's gain table stays the
// single source of truth: add a gain there and it appears here too.
var G=null;
function loadG(){
 fetch('/api/gains',{cache:'no-store'}).then(function(r){return r.json();})
 .then(function(g){
  G=g; var h='';
  for(var k in g){
   h+='<label class="gr"><span>'+k+'</span><input id="g_'+k+
      '" type="number" step="any" min="'+g[k].lo+'" max="'+g[k].hi+
      '" value="'+(+g[k].v.toFixed(4))+'"></label>';
  }
  document.getElementById('gl').innerHTML=h;
 });
}
loadG();
function applyG(){
 if(!G)return;
 // Send every field; the firmware validates each one and rejects the whole
 // request if any is out of range.
 var b=[];
 for(var k in G){b.push(k+'='+document.getElementById('g_'+k).value);}
 fetch('/api/gains',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:b.join('&')})
 .then(function(r){return r.json().then(function(j){
   document.getElementById('gm').textContent=
    r.ok?'Applied. Not saved yet - press SAVE to keep after restart.'
        :('Rejected: '+j.error);
   if(r.ok)loadG();                    // re-read what the firmware accepted
  });})
 .catch(function(){document.getElementById('gm').textContent='Apply failed.';});
}
document.getElementById('gapply').onclick=applyG;
document.getElementById('gdef').onclick=function(){
 // Restore Defaults just fills the form with the firmware's defaults and
 // applies them - still not saved until SAVE is pressed.
 if(!G||!confirm('Restore default gains?'))return;
 for(var k in G){document.getElementById('g_'+k).value=+G[k].d.toFixed(4);}
 applyG();
};
document.getElementById('gsave').onclick=function(){
 if(confirm('Save current gains to EEPROM?'))send('gains_save');
};
document.getElementById('cstart').onclick=function(){send('cal_start');};
document.getElementById('ccap').onclick=function(){send('cal_capture');};
// Saving writes EEPROM, so confirm before spending a write cycle.
document.getElementById('csave').onclick=function(){
 if(confirm('Save calibration to EEPROM?'))send('cal_save');
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
      "\"vertex_calibrated\":%s,"
      "\"armed\":%s,"
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
    calibrating       ? "true" : "false",
    vertex_calibrated ? "true" : "false",
    armed             ? "true" : "false",
    batt_voltage);
  webServer.send(200, "application/json", json);
}

// GET /api/gains  — current value, accepted range, and default for each gain.
// Read-only; the dashboard uses it to build the editing form.
void handleApiGains() {
  char json[1024];
  int n = 0;
  n += snprintf(json + n, sizeof(json) - n, "{");
  for (int i = 0; i < NUM_GAINS && n < (int)sizeof(json); i++) {
    n += snprintf(json + n, sizeof(json) - n,
                  "%s\"%s\":{\"v\":%.4f,\"lo\":%.4f,\"hi\":%.4f,\"d\":%.4f}",
                  i ? "," : "", GAIN_DEFS[i].name, *GAIN_DEFS[i].ptr,
                  GAIN_DEFS[i].lo, GAIN_DEFS[i].hi, GAIN_DEFS[i].def);
  }
  n += snprintf(json + n, sizeof(json) - n, "}");
  if (n >= (int)sizeof(json)) {         // never send truncated JSON
    webServer.send(500, "application/json",
                   "{\"ok\":false,\"error\":\"gain buffer overflow\"}");
    return;
  }
  webServer.send(200, "application/json", json);
}

// POST /api/gains  — temporarily change one or more gains, e.g. "K1=185&K3=2".
//
// Changes apply in RAM only; they are lost on restart unless the user then
// presses Save.  Every supplied value is validated BEFORE any is applied, so
// a single bad field cannot leave the controller half-updated.
//
// Assigning the gains here is safe despite this being an HTTP handler:
// handleClient() is called from loop(), so this code cannot interrupt a
// control-loop iteration - it runs strictly between them.
void handleApiGainsSet() {
  float staged[NUM_GAINS];
  bool  present[NUM_GAINS] = {false};
  int   count = 0;

  // Pass 1: parse and validate everything.
  for (int i = 0; i < NUM_GAINS; i++) {
    if (!webServer.hasArg(GAIN_DEFS[i].name)) continue;
    String raw = webServer.arg(GAIN_DEFS[i].name);

    // strtod rather than toFloat(): toFloat() silently returns 0 for
    // garbage, which would quietly zero a gain instead of reporting an error.
    const char* s = raw.c_str();
    char* end;
    double v = strtod(s, &end);
    while (*end == ' ') end++;                 // tolerate trailing spaces
    bool bad = (end == s) || (*end != '\0') || isnan(v) || isinf(v);
    if (!bad && (v < GAIN_DEFS[i].lo || v > GAIN_DEFS[i].hi)) bad = true;

    if (bad) {
      char err[160];
      snprintf(err, sizeof(err),
               "{\"ok\":false,\"error\":\"%s must be a number between "
               "%.4f and %.4f\"}",
               GAIN_DEFS[i].name, GAIN_DEFS[i].lo, GAIN_DEFS[i].hi);
      webServer.send(400, "application/json", err);
      return;                                  // nothing applied
    }
    staged[i] = (float)v;
    present[i] = true;
    count++;
  }

  if (count == 0) {
    webServer.send(400, "application/json",
                   "{\"ok\":false,\"error\":\"no known gain supplied\"}");
    return;
  }

  // Pass 2: everything validated, so apply.
  for (int i = 0; i < NUM_GAINS; i++)
    if (present[i]) *GAIN_DEFS[i].ptr = staged[i];

  char json[64];
  snprintf(json, sizeof(json), "{\"ok\":true,\"applied\":%d}", count);
  webServer.send(200, "application/json", json);
}

// POST /api/command  — accepts "cmd=stop", "cmd=disarm" or "cmd=arm".
//
// SAFETY: this handler does not command the motors and does not change the
// balancing state itself.  It only records a request; the main control loop
// applies it at the start of its next cycle (within one 15 ms period).
void handleApiCommand() {
  if (!webServer.hasArg("cmd")) {
    webServer.send(400, "application/json",
                   "{\"ok\":false,\"error\":\"missing cmd\"}");
    return;
  }
  String cmd = webServer.arg("cmd");

  uint8_t req;
  bool is_cal = false;               // calibration commands are restricted
  if (cmd == "stop")        req = WEB_CMD_STOP;
  else if (cmd == "disarm") req = WEB_CMD_DISARM;
  else if (cmd == "arm")    req = WEB_CMD_ARM;
  else if (cmd == "cal_start")   { req = WEB_CMD_CAL_START;   is_cal = true; }
  else if (cmd == "cal_capture") { req = WEB_CMD_CAL_CAPTURE; is_cal = true; }
  else if (cmd == "cal_save")    { req = WEB_CMD_CAL_SAVE;    is_cal = true; }
  else if (cmd == "gains_save")  req = WEB_CMD_GAINS_SAVE;
  else {
    // Reject anything unrecognised rather than silently ignoring it.
    webServer.send(400, "application/json",
                   "{\"ok\":false,\"error\":\"unknown cmd\"}");
    return;
  }

  // Never calibrate while the motors are actively balancing.  Rejecting here
  // gives the user immediate feedback; the control loop re-checks before
  // acting, since the cube could start balancing in between.
  if (is_cal && balancingActive()) {
    webServer.send(409, "application/json",
                   "{\"ok\":false,\"error\":\"cannot calibrate while balancing"
                   " - disarm first\"}");
    return;
  }

  // A stop already waiting to be processed always wins.  Without this, an
  // arm arriving in the same 15 ms window could overwrite a pending stop
  // and the cube would never stop at all.
  if (web_cmd_pending == WEB_CMD_STOP && req != WEB_CMD_STOP) {
    webServer.send(409, "application/json",
                   "{\"ok\":false,\"error\":\"stop pending\"}");
    return;
  }
  web_cmd_pending = req;

  char json[64];
  snprintf(json, sizeof(json), "{\"ok\":true,\"cmd\":\"%s\"}", cmd.c_str());
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
  webServer.on("/api/command", HTTP_POST, handleApiCommand);
  webServer.on("/api/gains", HTTP_GET, handleApiGains);
  webServer.on("/api/gains", HTTP_POST, handleApiGainsSet);
  webServer.begin();
}

// Called every pass of loop(): service at most one pending HTTP request.
// handleClient() is non-blocking and returns immediately when idle, so it
// does not disturb the 15 ms balancing period.
void handleWebInterface() {
  webServer.handleClient();
}
