#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <NewPing.h>


#define ENABLE_PIN 16
#define IN1_PIN 2
#define IN2_PIN 14
#define IN3_PIN 12
#define IN4_PIN 13


// --- DEFINIREA PINILOR SENZORI ---
#define TRIG_PIN 15
#define ECHO_PIN 5
#define SERVO_PIN 4


// --- CONFIGURATII ---
const int motorSpeed = 600;
const int maxDist = 200;
String radarDate = "Apasa SCANARE pentru a verifica obstacolele.";


// --- OBIECTE ---
Servo turela;
NewPing sonar(TRIG_PIN, ECHO_PIN, maxDist);
ESP8266WebServer server(80);


// --- FUNCTII CONTROL MOTOARE ---
void moveForward() {
    analogWrite(IN1_PIN, motorSpeed);
    analogWrite(IN2_PIN, 0);
    analogWrite(IN3_PIN, motorSpeed);
    analogWrite(IN4_PIN, 0);
}


void moveBackward() {
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, motorSpeed);
    analogWrite(IN3_PIN, 0);
    analogWrite(IN4_PIN, motorSpeed);
}


void stopMotors() {
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, 0);
    analogWrite(IN3_PIN, 0);
    analogWrite(IN4_PIN, 0);
}


// --- FUNCTIE SCANARE RADAR ---
void ruleazaRadar() {
    radarDate = "<h3>Rezultate Radar:</h3><ul>";
    for (int unghi = 0; unghi <= 180; unghi += 45) {
        turela.write(unghi);
        delay(400);
        int distanta = sonar.ping_cm();
        if (distanta == 0) distanta = maxDist;
        radarDate += "<li>Unghi " + String(unghi) + "deg : " + String(distanta) + " cm</li>";
    }
    radarDate += "</ul>";
    turela.write(90);
}


void afiseazaPagina() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<style>body{font-family:Arial;text-align:center;background:#2c3e50;color:white;}";
    html += ".btn{display:block;margin:15px auto;background:#3498db;padding:20px;width:80%;font-size:20px;border-radius:10px;color:white;text-decoration:none;}";
    html += ".btn-stop{background:#e74c3c;}";
    html += ".btn-scan{background:#2ecc71;}";
    html += "</style></head><body>";

    html += "<h1>Control Robot IoT</h1>";

    html += "<a class='btn' href='/inainte'>Mergi Înainte</a>";
    html += "<a class='btn' href='/inapoi'>Mergi Înapoi</a>";
    html += "<a class='btn btn-stop' href='/stop'>STOP</a>";
    html += "<a class='btn btn-scan' href='/scanare'>SCANARE RADAR</a>";

    html += "<div style='margin-top:30px;'>" + radarDate + "</div>";

    html += "</body></html>";

    server.send(200, "text/html", html);
}


void setup() {
    Serial.begin(9600);
    pinMode(ENABLE_PIN, OUTPUT);
    // ... (pini motoare)
    digitalWrite(ENABLE_PIN, HIGH);
 stopMotors();
    turela.attach(SERVO_PIN);
    turela.write(90);
    WiFi.softAP("hotspot_robot<3>", "1234567890");
    Serial.println(WiFi.softAPIP());
    server.on("/", afiseazaPagina);
    server.on("/inainte", [](){ moveForward(); afiseazaPagina(); });
    server.on("/inapoi",  [](){ moveBackward(); afiseazaPagina(); });
    server.on("/stop",    [](){ stopMotors(); afiseazaPagina(); });
    server.on("/scanare", [](){ stopMotors(); ruleazaRadar(); afiseazaPagina(); });
    server.begin();
}


void loop() {
    server.handleClient();
}