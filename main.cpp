#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <NewPing.h>
#include <ArduinoJson.h>
#include <math.h>

// Hardware and PWM functions from the physically tested firmware.
#define ENABLE_PIN 16
#define IN1_PIN 2
#define IN2_PIN 14
#define IN3_PIN 12
#define IN4_PIN 13
#define TRIG_PIN 15
#define ECHO_PIN 5
#define TRIG_PIN_SPATE 1
#define ECHO_PIN_SPATE 3
#define SERVO_PIN 4
// No Serial: GPIO1/GPIO3 are used by the rear sonar.
const int motorSpeed = 200;
const int maxDist = 200;
const int MODEL_MAX_DIST_CM = 100; // Preserve the existing Python input scaling.
const int distantaStop = 8;
const int ANGLE_RIGHT = 1;
const int ANGLE_FRONT = 90;
const int ANGLE_LEFT = 179;
const int PAS_SCANARE = 5;
const int SERVO_DELAY = 450;
const int PRAG_CONECT = 25;
const int PRAG_SPIKE = 25;
#define NR_PASI ((180 / PAS_SCANARE) + 1)
#define MAP_W 60
#define MAP_H 60
const float SCALA_HARTA = 0.25;
char harta[MAP_H][MAP_W];
const int scanOriginX = MAP_W / 2;
const int scanOriginY = MAP_H / 2;
int distFataBuf[NR_PASI];
int distSpateBuf[NR_PASI];
String radarDate = "Scanarea este disponibila cand robotul este oprit.";

// Original model coordinate frame. Heading 0 is the initial forward direction.
const int MAP_SIZE = 25;
const int CELL_SIZE_CM = 5;
int robotX = MAP_SIZE / 2;
int robotY = MAP_SIZE / 2;
int robotDir = 0; // 0 up, 1 right, 2 down, 3 left
float targetX = 0.0f;
float targetY = 0.0f;
// Supply measured values in the web form. 100/125 ms steering pulses in the
// pasted code are NOT calibrated quarter turns or 5 cm travel times.
unsigned long forwardStepMs = 0;
unsigned long turnLeft90Ms = 0;
unsigned long turnRight90Ms = 0;
bool poseKnown = true;
bool running = false;
bool targetTrimis = false;
bool motionBusy = false;
String robotStatus = "Configureaza tinta si timpii masurati, apoi START AI.";
unsigned long lastSensorSend = 0;
const unsigned long sensorIntervalMs = 500;

const char* apSsid = "Robot_Mapare";
const char* apPassword = "12345678";
const char* pythonTargetUrl = "http://192.168.4.2:5000/target";
const char* pythonSensorUrl = "http://192.168.4.2:5000/sensor";
Servo turela;
NewPing sonar(TRIG_PIN, ECHO_PIN, maxDist);
NewPing sonar_spate(TRIG_PIN_SPATE, ECHO_PIN_SPATE, maxDist);
ESP8266WebServer server(80);
WiFiClient wifiClient;

void moveForward()  { analogWrite(IN1_PIN, motorSpeed); analogWrite(IN2_PIN, 0); analogWrite(IN3_PIN, motorSpeed); analogWrite(IN4_PIN, 0); }
void moveBackward() { analogWrite(IN1_PIN, 0); analogWrite(IN2_PIN, motorSpeed); analogWrite(IN3_PIN, 0); analogWrite(IN4_PIN, motorSpeed); }
void rotateRight()  { analogWrite(IN1_PIN, motorSpeed); analogWrite(IN2_PIN, 0); analogWrite(IN3_PIN, 0); analogWrite(IN4_PIN, motorSpeed); }
void rotateLeft()   { analogWrite(IN1_PIN, 0); analogWrite(IN2_PIN, motorSpeed); analogWrite(IN3_PIN, motorSpeed); analogWrite(IN4_PIN, 0); }
void stopMotors()   { analogWrite(IN1_PIN, 0); analogWrite(IN2_PIN, 0); analogWrite(IN3_PIN, 0); analogWrite(IN4_PIN, 0); }

// =====================================================================
//  DISTANTA  (un singur ping, FARA mediere)
// =====================================================================
int calculateDistance() {
  int cm = sonar.ping_cm();
  return (cm <= 0) ? 0 : cm;
}

int calculateDistanceBack() {
  int cm = sonar_spate.ping_cm();
  return (cm <= 0) ? 0 : cm;
}

// Elimina varfurile izolate: un punct mult mai departe decat AMBII vecini = reflexie.
void curataSpike(int dist[]) {
  for (int i = 1; i < NR_PASI - 1; i++) {
    if (dist[i] <= 0) continue;
    if (dist[i - 1] <= 0 || dist[i + 1] <= 0) continue;
    if ((dist[i] - dist[i - 1]) > PRAG_SPIKE &&
        (dist[i] - dist[i + 1]) > PRAG_SPIKE) {
      dist[i] = 0;
    }
  }
}

// =====================================================================
//  HARTA - utilitare
// =====================================================================
void initHarta() {
  for (int y = 0; y < MAP_H; y++)
    for (int x = 0; x < MAP_W; x++)
      harta[y][x] = '.';   // '.' = necunoscut
}

void seteazaCelula(int x, int y, char val) {
  if (x >= 0 && x < MAP_W && y >= 0 && y < MAP_H)
    harta[y][x] = val;
}

void clampLaHarta(int &x, int &y) {
  if (x < 0)      x = 0;
  if (x >= MAP_W) x = MAP_W - 1;
  if (y < 0)      y = 0;
  if (y >= MAP_H) y = MAP_H - 1;
}

void puncteFata(int unghi, int distanta, int &x, int &y) {
  float rad = unghi * PI / 180.0;
  x = scanOriginX + cos(rad) * distanta * SCALA_HARTA;
  y = scanOriginY - sin(rad) * distanta * SCALA_HARTA;
}

void puncteSpate(int unghi, int distanta, int &x, int &y) {
  float rad = unghi * PI / 180.0;
  x = scanOriginX - cos(rad) * distanta * SCALA_HARTA;
  y = scanOriginY + sin(rad) * distanta * SCALA_HARTA;
}

void punctDirectie(int unghi, int distanta, bool spate, int &x, int &y) {
  if (spate) puncteSpate(unghi, distanta, x, y);
  else       puncteFata(unghi, distanta, x, y);
}

// Bresenham: marcheaza LIBER ( ' ' ) robot->punct, fara capat. Nu atinge zidurile.
void marcheazaLinieLibera(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  while (!(x0 == x1 && y0 == y1)) {
    if (x0 >= 0 && x0 < MAP_W && y0 >= 0 && y0 < MAP_H && harta[y0][x0] != '#')
      harta[y0][x0] = ' ';
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 <  dx) { err += dx; y0 += sy; }
  }
}

// Bresenham: marcheaza ZID ( '#' ) toata linia, inclusiv capetele.
void deseneazaLiniePerete(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  while (true) {
    seteazaCelula(x0, y0, '#');
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 <  dx) { err += dx; y0 += sy; }
  }
}

float distCelule(int x0, int y0, int x1, int y1) {
  float dx = x1 - x0, dy = y1 - y0;
  return sqrt(dx * dx + dy * dy);
}

// =====================================================================
//  DESENARE PERETI  (alb) + spatiu liber (gri)
//  Legam punctele normal: punctele consecutive apropiate se unesc.
// =====================================================================
void deseneazaPereti(int dist[], bool spate) {
  int prevX = -1, prevY = -1;

  for (int i = 0; i < NR_PASI; i++) {
    int unghi = i * PAS_SCANARE;
    int d = dist[i];

    if (d <= 0 || d > maxDist) {
      // fara ecou -> spatiu liber pana la max, fara perete
      int fx, fy;
      punctDirectie(unghi, maxDist, spate, fx, fy);
      clampLaHarta(fx, fy);
      marcheazaLinieLibera(scanOriginX, scanOriginY, fx, fy);
      prevX = -1;
      continue;
    }

    int x, y;
    punctDirectie(unghi, d, spate, x, y);

    int fx = x, fy = y;
    clampLaHarta(fx, fy);
    marcheazaLinieLibera(scanOriginX, scanOriginY, fx, fy);

    if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) { prevX = -1; continue; }

    seteazaCelula(x, y, '#');
    if (prevX >= 0 && distCelule(prevX, prevY, x, y) <= PRAG_CONECT)
      deseneazaLiniePerete(prevX, prevY, x, y);

    prevX = x; prevY = y;
  }
}

// Cauta cel mai departat punct dintr-un baleiaj; actualizeaza maximul global.
void gasesteMaxim(int dist[], bool spate, int &maxd, int &mx, int &my) {
  for (int i = 0; i < NR_PASI; i++) {
    int d = dist[i];
    if (d <= 0 || d > maxDist) continue;
    if (d <= maxd) continue;

    int unghi = i * PAS_SCANARE;
    int x, y;
    punctDirectie(unghi, d, spate, x, y);
    if (x >= 0 && x < MAP_W && y >= 0 && y < MAP_H) {
      maxd = d; mx = x; my = y;
    }
  }
}

// Marcheaza zona iesirii cu verde (un mic patrat 3x3 ca sa fie vizibil).
void marcheazaIesireZona(int cx, int cy) {
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++)
      seteazaCelula(cx + dx, cy + dy, '&');
}

// =====================================================================
//  GENERARE HARTA
// =====================================================================
void genereazaHarta() {
  initHarta();

  // --- Faza 1: baleiaj unic, un singur ping per senzor (fara mediere) ---
  for (int i = 0; i < NR_PASI; i++) {
    int unghi = i * PAS_SCANARE;
    turela.write(unghi);
    delay(SERVO_DELAY);
    distFataBuf[i]  = calculateDistance();
    delay(30);
    distSpateBuf[i] = calculateDistanceBack();
  }
  turela.write(90);

  // --- Faza 2: scoatem reflexiile izolate (ca iesirea sa nu cada pe zgomot) ---
  curataSpike(distFataBuf);
  curataSpike(distSpateBuf);

  // --- Faza 3: desenam peretii, legand punctele normal ---
  deseneazaPereti(distFataBuf, false);
  deseneazaPereti(distSpateBuf, true);

  // --- Faza 4: cel mai departat punct = iesirea (verde) ---
  int maxd = -1, mx = -1, my = -1;
  gasesteMaxim(distFataBuf, false, maxd, mx, my);
  gasesteMaxim(distSpateBuf, true,  maxd, mx, my);
  if (mx >= 0) marcheazaIesireZona(mx, my);
}

// =====================================================================
//  AFISARE DISTANTA
// =====================================================================
void afiseazaDistanta() {
  int distanta = calculateDistance();
  if (distanta == 0) radarDate = "<h3>Nu s-a detectat niciun obiect!</h3>";
  else {
    radarDate  = "<h3>Distanta pana la obiect (fata):</h3>";
    radarDate += "<p style='font-size:30px;color:#2ecc71;'>" + String(distanta) + " cm</p>";
  }
}

void afiseazaDistantaSpate() {
  int distanta = calculateDistanceBack();
  if (distanta == 0) radarDate = "<h3>Nu s-a detectat niciun obiect!</h3>";
  else {
    radarDate  = "<h3>Distanta pana la obiect (spate):</h3>";
    radarDate += "<p style='font-size:30px;color:#2ecc71;'>" + String(distanta) + " cm</p>";
  }
}

// =====================================================================
//  SCANARE RADAR SIMPLA
// =====================================================================
void ruleazaRadar() {
  radarDate = "<h3>Rezultate Radar:</h3><ul>";
  for (int unghi = 25; unghi <= 155; unghi += 20) {
    turela.write(unghi);
    delay(350);
    int distanta = calculateDistance();
    radarDate += "<li>Unghi " + String(unghi) + " grade : " + String(distanta) + " cm</li>";
  }
  radarDate += "</ul>";
  turela.write(90);
}

// =====================================================================
//  TRIMITERE HARTA (chunked)
// =====================================================================
void trimiteHartaHTML() {
  const int pixel = 3;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  server.sendContent(
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<style>body{font-family:Arial;text-align:center;background:#2c3e50;color:white;}"
    "a{display:inline-block;margin-top:20px;background:#3498db;padding:15px 30px;"
    "border-radius:10px;color:white;text-decoration:none;font-size:18px;}</style>"
    "</head><body>");

  server.sendContent("<h1>Scanare locala in jurul robotului</h1>");
  server.sendContent("<p>Negru = necunoscut, gri = liber, alb = zid, verde = punct indepartat (estimare), rosu = robot</p>");

  String head = "<svg width='" + String(MAP_W * pixel) +
                "' height='" + String(MAP_H * pixel) +
                "' style='background:black;border:2px solid white;'>";
  server.sendContent(head);

  String buf = "";
  for (int y = 0; y < MAP_H; y++) {
    for (int x = 0; x < MAP_W; x++) {
      char c = harta[y][x];
      const char *fill = nullptr;
      if      (c == ' ') fill = "#222";
      else if (c == '#') fill = "white";
      else if (c == '&') fill = "green";

      if (fill) {
        buf += "<rect x='" + String(x * pixel) +
               "' y='" + String(y * pixel) +
               "' width='" + String(pixel) +
               "' height='" + String(pixel) +
               "' fill='" + fill + "'/>";
        if (buf.length() > 1500) { server.sendContent(buf); buf = ""; }
      }
    }
  }
  if (buf.length() > 0) server.sendContent(buf);

  String robot = "<rect x='" + String(scanOriginX * pixel) +
                 "' y='" + String(scanOriginY * pixel) +
                 "' width='" + String(pixel) +
                 "' height='" + String(pixel) +
                 "' fill='red'/>";
  server.sendContent(robot);

  server.sendContent("</svg>");
  server.sendContent("<br><a href='/'>&larr; Inapoi la control</a>");
  server.sendContent("</body></html>");
  server.sendContent("");
}


// The scan/map is for inspection. Only Python's PPO policy chooses movement.
float robotXModel() { return float(robotX) / (MAP_SIZE - 1) * 2.0f - 1.0f; }
float robotYModel() { return 1.0f - float(robotY) / (MAP_SIZE - 1) * 2.0f; }
float normalizeDistanceCm(int cm) {
  return float(constrain(cm, 0, MODEL_MAX_DIST_CM)) / MODEL_MAX_DIST_CM * 2.0f;
}
bool targetReached() {
  // Half a grid cell accommodates targets between two discrete cells.
  const float tolerance = 1.0f / (MAP_SIZE - 1) + 0.0001f;
  return fabsf(targetX - robotXModel()) <= tolerance &&
         fabsf(targetY - robotYModel()) <= tolerance;
}
void haltRobot(const String &reason) {
  stopMotors();
  running = false;
  robotStatus = reason;
}
bool connectionReady() { return WiFi.softAPgetStationNum() > 0; }

// Local checks may veto a move, but never choose an alternative direction.
bool serviceMotion() {
  server.handleClient();
  yield();
  if (!running || !connectionReady()) {
    haltRobot("Oprit / conexiune Wi-Fi pierduta.");
    return false;
  }
  return true;
}
int citesteLaUnghi(int unghi) {
  stopMotors();
  turela.write(unghi);
  delay(SERVO_DELAY);
  return calculateDistance();
}
bool citesteSenzoriPentruModel(float &d1, float &d2) {
  int front = citesteLaUnghi(ANGLE_FRONT);
  delay(30);
  int rear = calculateDistanceBack();
  if (front <= 0 || rear <= 0) {
    // A missing echo is unknown, not a reliable free-space measurement.
    haltRobot("Senzor fara ecou valid. Verifica senzorii si reporneste START AI.");
    return false;
  }
  d1 = normalizeDistanceCm(front);
  d2 = normalizeDistanceCm(rear);
  return true;
}

bool trimiteTargetLaPython() {
  stopMotors();
  HTTPClient http;
  http.setTimeout(1500);
  if (!http.begin(wifiClient, pythonTargetUrl)) {
    robotStatus = "Nu pot initializa conexiunea /target.";
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  String body = "{\"target_x\":" + String(targetX, 4) +
                ",\"target_y\":" + String(targetY, 4) + "}";
  int code = http.POST(body);
  String response = code > 0 ? http.getString() : "";
  http.end();
  StaticJsonDocument<256> doc;
  if (code != 200 || deserializeJson(doc, response)) {
    robotStatus = "Eroare /target HTTP " + String(code);
    return false;
  }
  const char* status = doc["status"] | "";
  if (strcmp(status, "target_set") != 0) {
    robotStatus = "Confirmare /target invalida.";
    return false;
  }
  robotStatus = "Tinta confirmata de Python.";
  return true;
}

bool turnQuarter(bool right) {
  // Check both sides before pivoting; robot dimensions still need clearance
  // calibration. This is not a geometric collision guarantee.
  int left = citesteLaUnghi(ANGLE_LEFT);
  int rightDistance = citesteLaUnghi(ANGLE_RIGHT);
  turela.write(ANGLE_FRONT);
  delay(SERVO_DELAY);
  if (left <= distantaStop || rightDistance <= distantaStop) {
    haltRobot("Rotire blocata: obstacol lateral sau citire invalida.");
    return false;
  }
  if (!serviceMotion()) return false;
  unsigned long duration = right ? turnRight90Ms : turnLeft90Ms;
  if (right) rotateRight(); else rotateLeft();
  unsigned long started = millis();
  while (millis() - started < duration) {
    if (!serviceMotion()) {
      poseKnown = false; // Partial turns invalidate the discrete heading.
      return false;
    }
    delay(5);
  }
  stopMotors();
  robotDir = (robotDir + (right ? 1 : 3)) % 4;
  return true;
}

void aplicaActiune(int action) {
  stopMotors();
  if (action == -1) { haltRobot("STOP primit de la Python."); return; }
  if (action < 0 || action > 3) { haltRobot("Actiune Python invalida."); return; }
  if (!running || !poseKnown || !forwardStepMs || !turnLeft90Ms || !turnRight90Ms) {
    haltRobot("Pozitie sau calibrare indisponibila.");
    return;
  }
  // Python actions are absolute directions, NOT steering commands.
  const int desiredHeading[] = {1, 3, 0, 2};
  const int dx[] = {1, -1, 0, 0};
  const int dy[] = {0, 0, -1, 1};
  int nextX = robotX + dx[action];
  int nextY = robotY + dy[action];
  if (nextX < 0 || nextX >= MAP_SIZE || nextY < 0 || nextY >= MAP_SIZE) {
    haltRobot("Actiunea depaseste cadrul de coordonate.");
    return;
  }
  motionBusy = true;
  int difference = (desiredHeading[action] - robotDir + 4) % 4;
  bool turned = true;
  if (difference == 3) turned = turnQuarter(false);
  else for (int i = 0; i < difference && turned; ++i) turned = turnQuarter(true);
  if (!turned) { motionBusy = false; return; }
  int distance = citesteLaUnghi(ANGLE_FRONT);
  if (distance <= distantaStop + CELL_SIZE_CM) {
    haltRobot("Pas blocat: obstacol in fata sau citire invalida.");
    motionBusy = false;
    return;
  }
  if (!serviceMotion()) { motionBusy = false; return; }
  moveForward();
  unsigned long started = millis();
  unsigned long lastPing = started;
  while (millis() - started < forwardStepMs) {
    if (!serviceMotion()) {
      poseKnown = false;
      motionBusy = false;
      return;
    }
    if (millis() - lastPing >= 40) {
      lastPing = millis();
      int cm = calculateDistance();
      if (cm <= distantaStop) {
        haltRobot("Pas intrerupt: obstacol sau senzor fara ecou.");
        poseKnown = false;
        motionBusy = false;
        return;
      }
    }
    delay(5);
  }
  stopMotors();
  // Open-loop estimate only: no wheel encoders are present in the supplied code.
  robotX = nextX;
  robotY = nextY;
  motionBusy = false;
  robotStatus = "Pas AI executat (pozitie estimata).";
  if (targetReached()) haltRobot("Tinta atinsa conform pozitiei estimate.");
}

void trimiteSenzoriLaPython() {
  stopMotors();
  float d1, d2;
  if (!citesteSenzoriPentruModel(d1, d2)) return;
  // Preserve the eight-input model: Python derives target and delta fields.
  String body = "{\"d1\":" + String(d1, 4) + ",\"d2\":" + String(d2, 4) +
                ",\"x\":" + String(robotXModel(), 4) +
                ",\"y\":" + String(robotYModel(), 4) + "}";
  HTTPClient http;
  http.setTimeout(1500);
  if (!http.begin(wifiClient, pythonSensorUrl)) {
    haltRobot("Nu pot initializa conexiunea /sensor.");
    return;
  }
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  String response = code > 0 ? http.getString() : "";
  http.end();
  if (code == 409) {
    targetTrimis = false; // Python restarted: repeat the target handshake.
    robotStatus = "Python solicita retrimiterea tintei.";
    return;
  }
  if (code != 200) {
    haltRobot("Eroare /sensor HTTP " + String(code));
    return;
  }
  StaticJsonDocument<768> doc;
  if (deserializeJson(doc, response) || !doc["action"].is<int>()) {
    haltRobot("Raspuns JSON invalid de la Python.");
    return;
  }
  // Process a queued STOP before applying any returned action.
  server.handleClient();
  if (running) aplicaActiune(doc["action"].as<int>());
}

void afiseazaPagina() {
  String html = F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><meta charset='utf-8'><title>Robot AI</title></head><body><h1>Robot AI - Python / PPO</h1><p>");
  html += robotStatus + "</p><p>x=" + String(robotXModel(), 3) +
          ", y=" + String(robotYModel(), 3) + ", directie=" + String(robotDir) +
          ", pozitie valida=" + String(poseKnown ? "da" : "nu") + "</p>";
  html += F("<form method='post' action='/stop'><button>STOP</button></form><form method='post' action='/start'><button>START AI</button></form><h2>Configurare (robot oprit)</h2><p>Timpi masurati la PWM 200/255. Tinta in intervalul -1..1. Directia initiala a robotului este SUS.</p><form method='post' action='/configure'>");
  html += "Tinta X <input name='x' type='number' min='-1' max='1' step='0.001' value='" + String(targetX, 4) + "'><br>";
  html += "Tinta Y <input name='y' type='number' min='-1' max='1' step='0.001' value='" + String(targetY, 4) + "'><br>";
  html += "5 cm inainte (ms) <input name='forward' type='number' min='1' max='10000' value='" + String(forwardStepMs) + "'><br>";
  html += "90 grade stanga (ms) <input name='left' type='number' min='1' max='10000' value='" + String(turnLeft90Ms) + "'><br>";
  html += "90 grade dreapta (ms) <input name='right' type='number' min='1' max='10000' value='" + String(turnRight90Ms) + "'><br>";
  html += F("<button>Salveaza</button></form><p>Configuratia se pierde la restart. Dupa un pas intrerupt, readu robotul fizic la pozitia si orientarea initiale.</p><form method='post' action='/reset_pose'><button>Robot readus la start: reseteaza pozitia</button></form><p><a href='/scanare'>Radar</a> | <a href='/harta'>Harta locala</a> | <a href='/calibrare'>Distanta fata</a> | <a href='/calibrareSpate'>Distanta spate</a> | <a href='/'>Actualizeaza</a></p><p>Harta este o scanare locala; marcajul verde nu este o iesire confirmata si nu seteaza tinta AI.</p>");
  html += radarDate + "</body></html>";
  server.send(200, "text/html", html);
}
bool requireIdle() {
  if (running || motionBusy) {
    server.send(409, "text/plain", "Apasa STOP inainte de configurare sau scanare.");
    return false;
  }
  return true;
}
bool readNumber(const char* name, float &value) {
  if (!server.hasArg(name)) return false;
  String input = server.arg(name);
  const char* begin = input.c_str();
  char* end;
  value = strtof(begin, &end);
  return end != begin && *end == '\0' && isfinite(value);
}
void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  analogWriteRange(255);
  stopMotors();
  digitalWrite(ENABLE_PIN, HIGH);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_SPATE, OUTPUT); pinMode(ECHO_PIN_SPATE, INPUT);
  turela.attach(SERVO_PIN, 450, 2450);
  turela.write(ANGLE_FRONT);
  initHarta();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid, apPassword);
  server.on("/", afiseazaPagina);
  server.on("/stop", HTTP_POST, []() { haltRobot("STOP utilizator."); afiseazaPagina(); });
  server.on("/start", HTTP_POST, []() {
    if (!requireIdle()) return;
    if (!poseKnown || !forwardStepMs || !turnLeft90Ms || !turnRight90Ms || targetReached()) {
      server.send(400, "text/plain", "Configureaza timpii, o tinta diferita de pozitia curenta si o pozitie valida.");
      return;
    }
    targetTrimis = false;
    running = true;
    robotStatus = "Pornire AI; astept confirmarea Python.";
    afiseazaPagina();
  });
  server.on("/configure", HTTP_POST, []() {
    if (!requireIdle()) return;
    float x, y, forward, left, right;
    if (!readNumber("x", x) || !readNumber("y", y) ||
        !readNumber("forward", forward) || !readNumber("left", left) || !readNumber("right", right) ||
        fabsf(x) > 1 || fabsf(y) > 1 || forward < 1 || forward > 10000 ||
        left < 1 || left > 10000 || right < 1 || right > 10000) {
      server.send(400, "text/plain", "Coordonate -1..1; timpi 1..10000 ms.");
      return;
    }
    targetX = x; targetY = y;
    forwardStepMs = (unsigned long)forward;
    turnLeft90Ms = (unsigned long)left;
    turnRight90Ms = (unsigned long)right;
    targetTrimis = false;
    robotStatus = "Configuratie salvata in RAM.";
    afiseazaPagina();
  });
  server.on("/reset_pose", HTTP_POST, []() {
    if (!requireIdle()) return;
    robotX = MAP_SIZE / 2; robotY = MAP_SIZE / 2; robotDir = 0;
    poseKnown = true;
    targetTrimis = false;
    robotStatus = "Pozitie initiala confirmata de utilizator.";
    afiseazaPagina();
  });
  server.on("/scanare", []() { if (requireIdle()) { stopMotors(); ruleazaRadar(); afiseazaPagina(); } });
  server.on("/harta", []() { if (requireIdle()) { stopMotors(); genereazaHarta(); trimiteHartaHTML(); } });
  server.on("/calibrare", []() { if (requireIdle()) { citesteLaUnghi(ANGLE_FRONT); afiseazaDistanta(); afiseazaPagina(); } });
  server.on("/calibrareSpate", []() { if (requireIdle()) { citesteLaUnghi(ANGLE_FRONT); afiseazaDistantaSpate(); afiseazaPagina(); } });
  server.begin();
}
void loop() {
  server.handleClient();
  if (!running) { stopMotors(); delay(1); return; }
  if (!connectionReady()) { haltRobot("Niciun client Wi-Fi conectat."); return; }
  if (!targetTrimis) {
    targetTrimis = trimiteTargetLaPython();
    if (!targetTrimis) { delay(100); return; }
  }
  if (targetReached()) { haltRobot("Tinta atinsa conform pozitiei estimate."); return; }
  if (millis() - lastSensorSend >= sensorIntervalMs) {
    trimiteSenzoriLaPython();
    lastSensorSend = millis();
  }
  yield();
}
