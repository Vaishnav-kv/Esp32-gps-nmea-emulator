#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

// ================= PIN DEFINITIONS =================
// UART to the FC's GPS port.  ESP32 TX (17) -> FC GPS RX,  ESP32 RX (16) <- FC GPS TX,  GND common.
#define GPS_UART_TX_PIN 17
#define GPS_UART_RX_PIN 16
#define LOAD_PWM_PIN    25

// ================= WI-FI =================
const char* ssid = "xxxxxxx";  // set your Wi-Fi SSID here
const char* pwd  = "xxxxxxxxx";  // set your Wi-Fi password here
// Fallback access point if the lab Wi-Fi is not reachable (browse to http://192.168.4.1)
const char* AP_SSID = "GPS-Bench";
const char* AP_PWD  = "gpsbench123";

// ================= PWM =================
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH()  ledcAttach(LOAD_PWM_PIN, PWM_FREQ, PWM_RES)
  #define PWM_WRITE(v)  ledcWrite(LOAD_PWM_PIN, (v))
#else
  #define PWM_ATTACH()  do { ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(LOAD_PWM_PIN, 0); } while (0)
  #define PWM_WRITE(v)  ledcWrite(0, (v))
#endif

// ================= STATE =================
double sim_lat  = 12.981500;   // CURRENT simulated position (what the FC sees)
double sim_lon  = 80.198500;
double tgt_lat  = 12.981500;   // TARGET position (set from the web page)
double tgt_lon  = 80.198500;
float  move_speed_mps = 10.0f; // travel speed toward the target; 0 = instant jump
float  cur_speed_mps  = 0.0f;  // speed actually being reported right now
float  cur_course_deg = 0.0f;  // course over ground (held when stopped)
float  dist_left_m    = 0.0f;
float  sim_alt  = 15.0f;
int    sim_sats = 12;
float  sim_hdop = 0.9f;
bool   fix_enabled   = true;   // report a 3D fix
int    fix_delay_s   = 0;      // seconds after (re)start before the fix appears
bool   auto_power_mode = true; // GPS boot power profile
int    manual_pwm = 0;
int    active_pwm = 0;
unsigned long profile_t0 = 0;  // start of boot profile / fix delay
unsigned long nmea_count = 0;
char last_gga[120] = "";

WebServer server(80);
HardwareSerial FCSerial(2);

// ================= NMEA =================
static void sendNMEA(const char* body) {
  uint8_t cs = 0;
  for (const char* p = body; *p; p++) cs ^= (uint8_t)*p;
  FCSerial.printf("$%s*%02X\r\n", body, cs);
}

// ddmm.mmmmm,N  (lat)  /  dddmm.mmmmm,E (lon) - integer maths so 59.999999' never rounds to 60'
static void fmtCoord(double deg, bool isLat, char* out, size_t n) {
  char dir = isLat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W');
  deg = fabs(deg);
  int d = (int)deg;
  long m = lround((deg - d) * 60.0 * 100000.0);   // units of 1e-5 minute
  if (m >= 6000000L) { m -= 6000000L; d++; }
  snprintf(out, n, isLat ? "%02d%02ld.%05ld,%c" : "%03d%02ld.%05ld,%c",
           d, m / 100000L, m % 100000L, dir);
}

static bool hasFix() {
  return fix_enabled && ((millis() - profile_t0) / 1000UL >= (unsigned long)fix_delay_s);
}

static bool utcNow(struct tm* t, int* centis) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec > 1700000000L) {                 // NTP has synced
    gmtime_r(&tv.tv_sec, t);
    *centis = (tv.tv_usec / 10000 / 20) * 20;    // align to 200 ms
    return true;
  }
  // fallback: fixed date + uptime
  unsigned long ms = millis();
  memset(t, 0, sizeof(*t));
  t->tm_year = 126; t->tm_mon = 10; t->tm_mday = 9;
  unsigned long s = ms / 1000;
  t->tm_hour = (s / 3600) % 24; t->tm_min = (s / 60) % 60; t->tm_sec = s % 60;
  *centis = ((ms % 1000) / 200) * 20;
  return false;
}

// ================= MOTION =================
// Moves the simulated position toward the target at move_speed_mps so the FC sees a
// smooth track (with matching speed/course in RMC/VTG) instead of a teleport.
static void updateMotion() {
  static unsigned long lastMove = 0;
  unsigned long now = millis();
  if (lastMove == 0) { lastMove = now; return; }
  float dt = (now - lastMove) / 1000.0f;
  lastMove = now;
  if (dt <= 0.0f || dt > 2.0f) return;           // ignore stalls

  const double M_PER_DEG = 111320.0;
  double dN = (tgt_lat - sim_lat) * M_PER_DEG;
  double dE = (tgt_lon - sim_lon) * M_PER_DEG * cos(sim_lat * DEG_TO_RAD);
  double dist = sqrt(dN * dN + dE * dE);
  dist_left_m = (float)dist;

  if (dist < 0.01) {                              // arrived
    cur_speed_mps = 0.0f;
    return;
  }

  float c = (float)(atan2(dE, dN) * RAD_TO_DEG);
  if (c < 0) c += 360.0f;
  cur_course_deg = c;

  double step = (double)move_speed_mps * dt;
  if (move_speed_mps <= 0.0f || step >= dist) {   // instant mode, or final step
    sim_lat = tgt_lat;
    sim_lon = tgt_lon;
    cur_speed_mps = 0.0f;
    dist_left_m = 0.0f;
  } else {
    double f = step / dist;
    sim_lat += (tgt_lat - sim_lat) * f;
    sim_lon += (tgt_lon - sim_lon) * f;
    cur_speed_mps = move_speed_mps;
    dist_left_m = (float)(dist - step);
  }
}

static void sendGPSEpoch() {
  struct tm t; int cs;
  utcNow(&t, &cs);
  char tm_s[16], dt_s[8];
  snprintf(tm_s, sizeof(tm_s), "%02d%02d%02d.%02d", t.tm_hour, t.tm_min, t.tm_sec, cs);
  snprintf(dt_s, sizeof(dt_s), "%02d%02d%02d", t.tm_mday, t.tm_mon + 1, t.tm_year % 100);

  char body[140];
  bool fix = hasFix();

  if (fix) {
    char la[20], lo[20];
    fmtCoord(sim_lat, true, la, sizeof(la));
    fmtCoord(sim_lon, false, lo, sizeof(lo));

    float spd_kn  = cur_speed_mps * 1.943844f;
    float spd_kmh = cur_speed_mps * 3.6f;

    snprintf(body, sizeof(body), "GPGGA,%s,%s,%s,1,%02d,%.1f,%.1f,M,0.0,M,,",
             tm_s, la, lo, sim_sats, sim_hdop, sim_alt);
    strncpy(last_gga, body, sizeof(last_gga) - 1);
    sendNMEA(body);

    snprintf(body, sizeof(body), "GPRMC,%s,A,%s,%s,%.2f,%.2f,%s,,,A",
             tm_s, la, lo, spd_kn, cur_course_deg, dt_s);
    sendNMEA(body);

    snprintf(body, sizeof(body), "GPVTG,%.2f,T,,M,%.2f,N,%.2f,K,A",
             cur_course_deg, spd_kn, spd_kmh);
    sendNMEA(body);

    // GSA: auto mode, 3D fix, list of PRNs
    char prn[64] = "";
    for (int i = 0; i < 12; i++) {
      char one[6];
      if (i < sim_sats) snprintf(one, sizeof(one), "%02d,", i + 1); else snprintf(one, sizeof(one), ",");
      strcat(prn, one);
    }
    snprintf(body, sizeof(body), "GPGSA,A,3,%s%.1f,%.1f,%.1f", prn, sim_hdop * 1.5f, sim_hdop, sim_hdop * 1.2f);
    sendNMEA(body);
  } else {
    snprintf(body, sizeof(body), "GPGGA,%s,,,,,0,00,99.9,,M,,M,,", tm_s);
    strncpy(last_gga, body, sizeof(last_gga) - 1);
    sendNMEA(body);
    snprintf(body, sizeof(body), "GPRMC,%s,V,,,,,,,%s,,,N", tm_s, dt_s);
    sendNMEA(body);
    sendNMEA("GPVTG,,T,,M,,N,,K,N");
    sendNMEA("GPGSA,A,1,,,,,,,,,,,,,99.9,99.9,99.9");
  }
  nmea_count++;
}

// ================= LOAD =================
static void updateLoad() {
  if (!auto_power_mode) {
    active_pwm = manual_pwm;
  } else {
    unsigned long up = millis() - profile_t0;
    if (up < 15000)      active_pwm = 225;  // acquisition - highest draw
    else if (up < 30000) active_pwm = 135;  // tracking
    else                 active_pwm = 79;   // steady state
  }
  PWM_WRITE(active_pwm);
}

// ================= WEB =================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GPS Bench</title>
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/leaflet@1.9.4/dist/leaflet.css">
<style>
:root{color-scheme:dark}
body{margin:0;font-family:system-ui,sans-serif;background:#1b1d21;color:#e8e8e8}
#map{height:55vh;min-height:280px;background:#2a2d33}
.wrap{padding:12px;display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(260px,1fr))}
.card{background:#25282e;border-radius:8px;padding:12px}
h3{margin:0 0 8px;font-size:15px;color:#8fc7ff}
label{display:flex;justify-content:space-between;align-items:center;gap:8px;margin:5px 0;font-size:14px}
input[type=text],input[type=number]{width:55%;padding:5px;background:#1b1d21;color:#fff;border:1px solid #444;border-radius:4px}
button{padding:7px 12px;border:0;border-radius:5px;background:#2f7fd1;color:#fff;font-size:14px;cursor:pointer}
button.alt{background:#444}
#st{font:12px monospace;white-space:pre;color:#9fd69f}
#nomap{display:none;padding:20px;text-align:center;color:#ccc}
</style></head><body>
<div id="map"></div>
<div id="nomap">Map tiles unavailable (no internet on this PC?). Use the lat/lon fields below.</div>
<div class="wrap">
 <div class="card"><h3>Target position (click map or drag pin)</h3>
  <label>Search <input type="text" id="q" placeholder="place name"><button class="alt" onclick="search()">Go</button></label>
  <label>Lat <input type="text" id="lat"></label>
  <label>Lon <input type="text" id="lon"></label>
  <label>Travel speed (m/s, 0 = jump) <input type="number" id="spd" min="0" max="300" step="0.5"></label>
  <label>Alt (m) <input type="number" id="alt" step="0.1"></label>
  <label>Satellites <input type="number" id="sats" min="0" max="12"></label>
  <label>HDOP <input type="number" id="hdop" step="0.1" min="0.5"></label>
  <label><span>Provide 3D fix</span><input type="checkbox" id="fix"></label>
  <label>Fix delay (s) <input type="number" id="fixdelay" min="0" max="600"></label>
 </div>
 <div class="card"><h3>Power load (22&#937;)</h3>
  <label><span>Auto GPS boot profile</span><input type="checkbox" id="auto"></label>
  <label>Manual PWM (0-255) <input type="number" id="pwm" min="0" max="255"></label>
  <p><button onclick="send()">Apply</button> <button class="alt" onclick="fetch('/api/restart').then(send)">Restart profile</button></p>
  <div id="st">connecting...</div>
 </div>
</div>
<script src="https://cdn.jsdelivr.net/npm/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const $=id=>document.getElementById(id);
let map=null,marker=null,curMarker=null,first=true;
if(window.L){
  const osm=L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap'});
  const sat=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{maxZoom:19,attribution:'Esri'});
  map=L.map('map',{layers:[osm]}).setView([12.9815,80.1985],16);
  L.control.layers({Street:osm,Satellite:sat}).addTo(map);
  marker=L.marker([12.9815,80.1985],{draggable:true}).addTo(map);
  curMarker=L.circleMarker([12.9815,80.1985],{radius:7,color:'#ff5a5a',fillColor:'#ff5a5a',fillOpacity:0.9}).addTo(map);
  map.on('click',e=>setPos(e.latlng.lat,e.latlng.lng,true));
  marker.on('dragend',()=>{const p=marker.getLatLng();setPos(p.lat,p.lng,true)});
}else{$('map').style.display='none';$('nomap').style.display='block';}
function setPos(lat,lon,push){
  if(marker)marker.setLatLng([lat,lon]);
  $('lat').value=lat.toFixed(6);$('lon').value=lon.toFixed(6);
  if(push)send();
}
function send(){
  const q=new URLSearchParams({lat:$('lat').value,lon:$('lon').value,alt:$('alt').value,sats:$('sats').value,
    hdop:$('hdop').value,fix:$('fix').checked?1:0,fixdelay:$('fixdelay').value,auto:$('auto').checked?1:0,pwm:$('pwm').value,
    speed:$('spd').value});
  fetch('/api/set?'+q).then(r=>r.json()).then(show).catch(()=>$('st').textContent='offline');
}
function search(){
  const q=$('q').value.trim();if(!q)return;
  fetch('https://nominatim.openstreetmap.org/search?format=json&limit=1&q='+encodeURIComponent(q))
   .then(r=>r.json()).then(j=>{if(j[0]){const la=+j[0].lat,lo=+j[0].lon;if(map)map.setView([la,lo],17);setPos(la,lo,true);}});
}
function show(s){
  if(first){
    setPos(s.lat,s.lon,false);if(map)map.setView([s.clat,s.clon],17);
    $('alt').value=s.alt;$('sats').value=s.sats;$('hdop').value=s.hdop;$('fix').checked=!!s.fix;
    $('fixdelay').value=s.fixdelay;$('auto').checked=!!s.auto;$('pwm').value=s.pwm;$('spd').value=s.spd;first=false;
  }
  if(curMarker)curMarker.setLatLng([s.clat,s.clon]);
  $('st').textContent='FC fix sent : '+(s.hasfix?'YES (3D)':'NO')+'\nFC position : '+s.clat.toFixed(6)+', '+s.clon.toFixed(6)+'\nMoving      : '+s.vel.toFixed(1)+' m/s, '+Math.round(s.dist)+' m to go\nActive PWM  : '+s.active+' ('+Math.round(s.active/2.55)+'%)\nUptime      : '+s.up+' s\nNMEA epochs : '+s.tx+'\nUTC source  : '+(s.ntp?'NTP':'uptime')+'\nWi-Fi       : '+s.ip+'  '+s.rssi+' dBm';
}
function poll(){fetch('/api/status').then(r=>r.json()).then(show).catch(()=>{});}
poll();setInterval(poll,1000);
</script></body></html>)rawliteral";

static String statusJson() {
  char b[640];
  struct timeval tv; gettimeofday(&tv, nullptr);
  snprintf(b, sizeof(b),
    "{\"lat\":%.6f,\"lon\":%.6f,\"clat\":%.6f,\"clon\":%.6f,\"spd\":%.1f,\"vel\":%.2f,\"dist\":%.1f,"
    "\"alt\":%.1f,\"sats\":%d,\"hdop\":%.1f,\"fix\":%d,\"fixdelay\":%d,"
    "\"auto\":%d,\"pwm\":%d,\"active\":%d,\"up\":%lu,\"tx\":%lu,\"rssi\":%d,\"ip\":\"%s\",\"hasfix\":%d,\"ntp\":%d}",
    tgt_lat, tgt_lon, sim_lat, sim_lon, move_speed_mps, cur_speed_mps, dist_left_m,
    sim_alt, sim_sats, sim_hdop, fix_enabled, fix_delay_s,
    auto_power_mode, manual_pwm, active_pwm, millis() / 1000UL, nmea_count,
    WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
    (WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(),
    hasFix(), tv.tv_sec > 1700000000L);
  return String(b);
}

static void handleRoot()   { server.send_P(200, "text/html", INDEX_HTML); }
static void handleStatus() { server.send(200, "application/json", statusJson()); }
static void handleRestart(){ profile_t0 = millis(); server.send(200, "application/json", "{}"); }

static void handleSet() {
  if (server.hasArg("lat"))  tgt_lat  = constrain(server.arg("lat").toDouble(), -90.0, 90.0);
  if (server.hasArg("lon"))  tgt_lon  = constrain(server.arg("lon").toDouble(), -180.0, 180.0);
  if (server.hasArg("speed")) move_speed_mps = constrain(server.arg("speed").toFloat(), 0.0f, 300.0f);
  if (server.hasArg("alt"))  sim_alt  = server.arg("alt").toFloat();
  if (server.hasArg("sats")) sim_sats = constrain(server.arg("sats").toInt(), 0, 12);
  if (server.hasArg("hdop")) sim_hdop = constrain(server.arg("hdop").toFloat(), 0.5f, 99.9f);
  if (server.hasArg("fix"))  fix_enabled = server.arg("fix").toInt() != 0;
  if (server.hasArg("fixdelay")) fix_delay_s = constrain(server.arg("fixdelay").toInt(), 0, 600);
  if (server.hasArg("auto")) {
    bool a = server.arg("auto").toInt() != 0;
    if (a && !auto_power_mode) profile_t0 = millis();   // re-enter profile from the top
    auto_power_mode = a;
  }
  if (server.hasArg("pwm"))  manual_pwm = constrain(server.arg("pwm").toInt(), 0, 255);
  Serial.printf("[WEB] tgt=%.6f,%.6f spd=%.1f alt=%.1f sats=%d hdop=%.1f fix=%d auto=%d pwm=%d\n",
                tgt_lat, tgt_lon, move_speed_mps, sim_alt, sim_sats, sim_hdop, fix_enabled, auto_power_mode, manual_pwm);
  server.send(200, "application/json", statusJson());
}

// ================= SETUP / LOOP =================
void setup() {
  // keep the FET off while booting
  pinMode(LOAD_PWM_PIN, OUTPUT);
  digitalWrite(LOAD_PWM_PIN, LOW);

  Serial.begin(115200);
  Serial.println("\n--- GPS & Load Emulator ---");

  // begin(baud, config, RX pin, TX pin)
  FCSerial.begin(115200, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
  PWM_ATTACH();
  PWM_WRITE(0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // avoids latency spikes in the 5 Hz stream
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pwd);
  Serial.printf("Connecting to %s", ssid);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print("."); }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWi-Fi OK  ->  http://%s  (or http://gpsbench.local)\n", WiFi.localIP().toString().c_str());
    configTime(0, 0, "pool.ntp.org", "time.google.com");   // real UTC for the NMEA time/date
    if (MDNS.begin("gpsbench")) MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("\nWi-Fi failed -> starting fallback AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PWD);
    Serial.printf("Join '%s' then browse http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }

  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/set", handleSet);
  server.on("/api/restart", handleRestart);
  server.begin();

  profile_t0 = millis();   // boot profile starts when the bench is ready
}

void loop() {
  server.handleClient();
  updateLoad();

  static unsigned long lastNMEA = 0;
  unsigned long now = millis();
  if (now - lastNMEA >= 200) {          // 5 Hz
    lastNMEA = now;
    updateMotion();
    sendGPSEpoch();
  }

  static unsigned long lastDbg = 0;
  if (now - lastDbg >= 1000) {
    lastDbg = now;
    Serial.printf("[DBG] up=%lus pwm=%3d fix=%d sats=%d spd=%.1f left=%.0fm | $%s\n",
                  now / 1000, active_pwm, hasFix(), sim_sats, cur_speed_mps, dist_left_m, last_gga);
  }

  // Wi-Fi watchdog: if we were on the lab network and dropped, try again
  static unsigned long lastWifi = 0;
  if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED && now - lastWifi > 10000) {
    lastWifi = now;
    WiFi.reconnect();
  }
}