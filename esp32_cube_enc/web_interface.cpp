// web_interface.cpp — phone-accessible Wi-Fi control panel for the cube.
//
// The ESP32 runs its own Wi-Fi ACCESS POINT (no router or internet needed):
// connect a phone to the "Cube-Control" network, then open the IP address
// printed on the Serial monitor (normally http://192.168.4.1).
//
// Design rules for this file:
//  * HTTP handlers must never call motor-control functions directly and
//    must never block.  They only read shared state (later phases will set
//    request flags that the main control loop acts on).
//  * The balancing loop in esp32_cube_enc.cpp stays fully independent:
//    loop() just calls handleWebInterface(), which returns immediately
//    when no client is waiting.
//
// Only built-in ESP32 Arduino libraries are used here.
#include "ESP32.h"
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

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
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0e1216">
<title>Cube Control</title><style>
:root{--bg:#0e1216;--pnl:#161d24;--ln:#243039;--ink:#e8eff5;--dim:#7b8b99;
--live:#ffab1f;--ok:#3ecf8e;--stop:#ff453a}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--ink);font:400 16px/1.45
system-ui,-apple-system,sans-serif;-webkit-text-size-adjust:100%;
padding:16px 14px calc(20px + env(safe-area-inset-bottom))}
/* placard labels: the engineering-instrument voice */
.k{font-size:10px;text-transform:uppercase;letter-spacing:.16em;color:var(--dim)}
/* header ------------------------------------------------------------ */
header{display:flex;align-items:baseline;gap:8px;margin-bottom:16px}
h1{font-size:13px;font-weight:600;letter-spacing:.14em;text-transform:uppercase;
margin:0;flex:1}
#s{font-size:11px;color:var(--dim);text-align:right}
#lamp{width:7px;height:7px;border-radius:50%;background:var(--ln);flex:none;
align-self:center}
#lamp.a{background:var(--live);animation:p 1.6s ease-in-out infinite}
@keyframes p{50%{opacity:.25}}
/* signature: attitude target with the three wheels at their real 120° */
.inst{position:relative;width:100%;max-width:330px;margin:0 auto;
aspect-ratio:1;display:grid;place-items:center}
svg{width:76%;height:76%;overflow:visible}
.ring{fill:none;stroke:var(--ln);stroke-width:1}
.ring.o{stroke:#33414d}
.ax{stroke:var(--ln);stroke-width:1;stroke-dasharray:2 5}
#dot{fill:var(--live);transition:cx .12s linear,cy .12s linear,fill .2s}
#dot.q{fill:var(--ok)}
.tick{font:9px ui-monospace,monospace;fill:var(--dim);letter-spacing:.05em}
/* three motor readouts, placed where the wheels actually are */
.w{position:absolute;width:74px;text-align:center}
.w.a{top:-2px;left:50%;transform:translateX(-50%)}      /* M3  top    */
.w.b{bottom:2px;left:-2px}                              /* M1  lower left  */
.w.c{bottom:2px;right:-2px}                             /* M2  lower right */
.w b{display:block;font:400 17px/1.1 ui-monospace,SFMono-Regular,Menlo,monospace;
font-variant-numeric:tabular-nums;margin-top:3px}
.bar{height:2px;background:var(--ln);margin-top:5px;border-radius:2px;
overflow:hidden}
.bar i{display:block;height:100%;width:0;background:var(--live);
transition:width .15s linear}
/* angle readouts ----------------------------------------------------- */
.ang{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:18px 0 4px}
.ang div{background:var(--pnl);border:1px solid var(--ln);border-radius:10px;
padding:10px 12px}
.ang b{display:block;font:300 30px/1.1 ui-monospace,SFMono-Regular,Menlo,monospace;
font-variant-numeric:tabular-nums;letter-spacing:-.03em;margin-top:4px}
/* status ------------------------------------------------------------- */
.st{display:flex;flex-wrap:wrap;gap:6px;margin:14px 0}
.p{font-size:11px;font-weight:600;letter-spacing:.1em;padding:5px 10px;
border-radius:6px;background:var(--pnl);border:1px solid var(--ln);
color:var(--dim)}
.p.on{color:var(--ok);border-color:#1f4d3a}
.p.live{color:var(--live);border-color:#5c4212}
.p.off{color:var(--stop);border-color:#5c231f}
/* controls ----------------------------------------------------------- */
button{font-family:inherit;border:0;border-radius:10px;color:var(--ink);
touch-action:manipulation;user-select:none}
#stop{width:100%;min-height:76px;font-size:19px;font-weight:700;
letter-spacing:.14em;color:#fff;background:var(--stop);
box-shadow:0 6px 20px -8px var(--stop)}
#stop:active{background:#c9302a;box-shadow:none}
.ab{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
.b{min-height:48px;padding:12px;font-size:14px;font-weight:600;
letter-spacing:.06em;background:var(--pnl);border:1px solid var(--ln)}
.b:active{background:#1f2831}
.b.w1{grid-column:1/-1}
/* collapsible sections ----------------------------------------------- */
details{background:var(--pnl);border:1px solid var(--ln);border-radius:10px;
margin-top:10px}
summary{padding:14px;font-size:12px;font-weight:600;letter-spacing:.14em;
text-transform:uppercase;cursor:pointer;list-style:none;display:flex;
align-items:center;gap:8px}
summary::-webkit-details-marker{display:none}
summary:after{content:'';width:6px;height:6px;border-right:1.5px solid var(--dim);
border-bottom:1.5px solid var(--dim);transform:rotate(45deg);margin-left:auto;
transition:transform .2s}
details[open] summary:after{transform:rotate(-135deg)}
.bd{padding:0 14px 14px}
.note{font-size:12px;color:var(--dim);margin:0 0 12px}
/* gain grid ---------------------------------------------------------- */
.gl{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.gr{display:flex;align-items:center;gap:7px}
.gr span{font:11px ui-monospace,monospace;color:var(--dim);width:2.6em;
letter-spacing:.04em}
.gr input{flex:1;min-width:0;background:var(--bg);color:var(--ink);
border:1px solid var(--ln);border-radius:7px;padding:9px 8px;font:14px
ui-monospace,SFMono-Regular,Menlo,monospace;font-variant-numeric:tabular-nums}
.gr input:focus{outline:2px solid var(--live);outline-offset:-1px;
border-color:transparent}
:focus-visible{outline:2px solid var(--live);outline-offset:2px}
@media(prefers-reduced-motion:reduce){*{animation:none!important;
transition:none!important}}
</style></head><body>

<header><span id="lamp"></span><h1>Cube</h1><div id="s">connecting</div></header>

<!-- Attitude target. The outer ring is the real +/-7 deg disengage limit from
     angle_calc(); the radial scale is sqrt so the sub-degree angles seen while
     balancing are actually visible. Motors sit at their true 120 deg spacing. -->
<div class="inst">
<svg viewBox="0 0 200 200" aria-hidden="true">
<circle class="ring o" cx="100" cy="100" r="70"/>
<circle class="ring" cx="100" cy="100" r="46"/>
<circle class="ring" cx="100" cy="100" r="26"/>
<line class="ax" x1="18" y1="100" x2="182" y2="100"/>
<line class="ax" x1="100" y1="18" x2="100" y2="182"/>
<circle class="ring" cx="100" cy="100" r="4"/>
<text class="tick" x="103" y="70">3°</text>
<text class="tick" x="103" y="26">7°</text>
<circle id="dot" cx="100" cy="100" r="5"/>
</svg>
<div class="w a"><span class="k">M3</span><b id="m3">—</b>
<div class="bar"><i id="b3"></i></div></div>
<div class="w b"><span class="k">M1</span><b id="m1">—</b>
<div class="bar"><i id="b1"></i></div></div>
<div class="w c"><span class="k">M2</span><b id="m2">—</b>
<div class="bar"><i id="b2"></i></div></div>
</div>

<div class="ang">
<div><span class="k">Tilt X</span><b id="ax">—</b></div>
<div><span class="k">Tilt Y</span><b id="ay">—</b></div>
</div>

<div class="st">
<span class="p" id="armed">ARMED</span>
<span class="p" id="cal">CALIBRATION</span>
<span class="p" id="mode">MODE</span>
<span class="p" id="bv">— V</span>
</div>

<button id="stop">SAFE STOP</button>
<div class="ab"><button class="b" id="arm">ARM</button>
<button class="b" id="disarm">DISARM</button></div>

<details><summary>Calibration</summary><div class="bd">
<p class="note" id="ch">—</p>
<p class="note" id="cr" style="color:var(--live)"></p>
<p class="note" id="raw" style="font-family:ui-monospace,monospace;font-size:11px">
raw</p>
<div class="ab" style="margin:0">
<button class="b" id="cstart">Start</button>
<button class="b" id="ccap">Capture pose</button>
<button class="b w1" id="csave">Save calibration</button></div>
</div></details>

<details><summary>Gains</summary><div class="bd">
<div id="gl" class="gl">Loading…</div>
<div class="ab">
<button class="b" id="gapply">Apply</button>
<button class="b" id="gdef">Restore defaults</button>
<button class="b w1" id="gsave">Save to EEPROM</button></div>
<p class="note" id="gm" style="margin:12px 0 0">Changes take effect at once.
They are lost on restart until you save.</p>
</div></details>

<script>
var $=function(i){return document.getElementById(i);};
var busy=false;                        // one request at a time: the ESP32
                                       // WebServer serves a single client,
                                       // so never let polls pile up
var mmax=40;                           // motor bar full-scale, auto-ranging
function pill(el,cls,txt){el.textContent=txt;el.className='p '+cls;}
// Degrees -> radius. sqrt so 0.5 deg is visible while 7 deg lands on the ring.
function rad(a){
 var m=Math.min(Math.abs(a)/7,1);
 return (a<0?-1:1)*Math.sqrt(m)*70;
}
function poll(){
 if(busy)return; busy=true;
 fetch('/api/state',{cache:'no-store'}).then(function(r){return r.json();})
 .then(function(d){
  $('ax').textContent=d.robot_angleX.toFixed(2)+'°';
  $('ay').textContent=d.robot_angleY.toFixed(2)+'°';
  $('dot').setAttribute('cx',100+rad(d.robot_angleX));
  $('dot').setAttribute('cy',100-rad(d.robot_angleY));
  // Green inside the vertex capture window, amber once it is drifting.
  var q=Math.abs(d.robot_angleX)<0.4&&Math.abs(d.robot_angleY)<0.4;
  $('dot').setAttribute('class',q?'q':'');  // SVG: className is read-only
  var s=[d.motor1_speed,d.motor2_speed,d.motor3_speed];
  mmax=Math.max(40,Math.abs(s[0]),Math.abs(s[1]),Math.abs(s[2]));
  for(var i=0;i<3;i++){
   $('m'+(i+1)).textContent=s[i];
   $('b'+(i+1)).style.width=(Math.abs(s[i])/mmax*100)+'%';
  }
  $('bv').textContent=d.batt_voltage.toFixed(2)+' V';
  $('lamp').className=d.armed?'a':'';
  pill($('armed'),d.armed?'live':'off',d.armed?'ARMED':'DISARMED');
  pill($('cal'),d.calibrated?'on':'off',
       d.calibrating?'CALIBRATING':(d.calibrated?'CALIBRATED':'NOT CALIBRATED'));
  // Pose is detected even while disarmed, so only colour it when it can act.
  var m=d.vertical_vertex?'VERTEX':(d.vertical_edge?'EDGE':'IDLE');
  pill($('mode'),(d.vertical_vertex||d.vertical_edge)&&d.armed?'live':'',m);
  // Tell the user which calibration step comes next.  Calibration is
  // blocked while balancing, so say that instead when it applies.
  var bal=d.armed&&(d.vertical_vertex||d.vertical_edge)&&d.calibrated
          &&!d.calibrating;
  $('ch').textContent=
   bal?'Balancing. Press DISARM before calibrating.':
   (!d.calibrating?'Idle. Press Start to begin.':
   (!d.vertex_calibrated?'Step 1 — set the cube on a VERTEX, then capture.':
    'Step 2 — set the cube on an EDGE, then capture. Saves automatically.'));
  // Result of the last capture, plus the raw counts the pose test actually
  // uses. Both used to be Bluetooth-only.
  $('cr').textContent=d.cal_result||'';
  $('raw').textContent='raw  X '+d.acX+'   Y '+d.acY+'   Z '+d.acZ;
  $('s').textContent='live';
 }).catch(function(){$('s').textContent='no signal';})
 .then(function(){busy=false;});
}
setInterval(poll,300);                 // 300 ms refresh (spec: 250-500 ms)
poll();
// Sends a command request only; the main control loop acts on it.
function send(cmd){
 fetch('/api/command',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:'cmd='+cmd})
 .then(function(r){$('s').textContent=
   r.ok?cmd.replace('_',' ')+' sent':cmd.replace('_',' ')+' failed '+r.status;})
 .catch(function(){$('s').textContent=cmd.replace('_',' ')+' failed';});
}
$('stop').onclick=function(){send('stop');};
$('disarm').onclick=function(){send('disarm');};
// Arming re-enables balancing, so require a deliberate confirmation.
$('arm').onclick=function(){
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
  $('gl').innerHTML=h;
 });
}
loadG();
function applyG(){
 if(!G)return;
 // Send every field; the firmware validates each one and rejects the whole
 // request if any is out of range.
 var b=[];
 for(var k in G){b.push(k+'='+$('g_'+k).value);}
 fetch('/api/gains',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:b.join('&')})
 .then(function(r){return r.json().then(function(j){
   $('gm').textContent=r.ok?'Applied. Save to keep them after a restart.'
                           :('Rejected — '+j.error);
   if(r.ok)loadG();                    // re-read what the firmware accepted
  });})
 .catch(function(){$('gm').textContent='Apply failed.';});
}
$('gapply').onclick=applyG;
$('gdef').onclick=function(){
 // Restore Defaults just fills the form with the firmware's defaults and
 // applies them - still not saved until SAVE is pressed.
 if(!G||!confirm('Restore default gains?'))return;
 for(var k in G){$('g_'+k).value=+G[k].d.toFixed(4);}
 applyG();
};
$('gsave').onclick=function(){
 if(confirm('Save current gains to EEPROM?'))send('gains_save');
};
$('cstart').onclick=function(){send('cal_start');};
$('ccap').onclick=function(){send('cal_capture');};
// Saving writes EEPROM, so confirm before spending a write cycle.
$('csave').onclick=function(){
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
  // 640 rather than 512: the raw accelerometer values and cal_result string
  // added when Bluetooth was removed push the worst case past the old size,
  // and snprintf truncates silently - which would emit malformed JSON.
  char json[640];
  int n = snprintf(json, sizeof(json),
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
      "\"batt_voltage\":%.2f,"
      // Raw accelerometer counts.  During a first-time calibration the
      // corrected angles above are computed from EEPROM garbage, so these
      // are the only trustworthy numbers - and the pose accept/reject
      // thresholds are applied to exactly these values.
      "\"acX\":%d,"
      "\"acY\":%d,"
      "\"acZ\":%d,"
      "\"cal_result\":\"%s\""
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
    batt_voltage,
    AcX, AcY, AcZ, cal_result);
  // Truncated JSON would be malformed, so refuse to send it rather than let
  // the dashboard silently fail to parse.
  if (n < 0 || n >= (int)sizeof(json)) {
    webServer.send(500, "application/json",
                   "{\"ok\":false,\"error\":\"state buffer overflow\"}");
    return;
  }
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
    // No AP means no dashboard, and therefore no SAFE STOP button.  Do not
    // leave the cube able to spin up three reaction wheels with no reachable
    // way to stop it: disarm, and let the operator re-arm over USB serial
    // once they can see what is going on.
    armed = false;
    Serial.println("Balancing disarmed: no web interface available.");
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
