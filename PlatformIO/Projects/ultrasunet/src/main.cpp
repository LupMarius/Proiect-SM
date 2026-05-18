#include <Arduino.h>
#include <NewPing.h>

// ---------------- SETARI ----------------
const int motorSpeed = 600;     // viteza motoare
const int distantaStop = 40;    // distanta la care se opreste (cm)

// ---- Senzor ultrasonic ----
#define TRIG_PIN 15  // Acesta este pinul fizic D8 pe Wemos/NodeMCU
#define ECHO_PIN 5   // Acesta este pinul fizic D1 pe Wemos/NodeMCU
#define MAX_DIST 200

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

// ---- Motoare ----
#define ENABLE_PIN 16 // Pinul fizic D0
#define IN1_PIN 2     // Pinul fizic D4
#define IN2_PIN 14    // Pinul fizic D5
#define IN3_PIN 12    // Pinul fizic D6
#define IN4_PIN 13    // Pinul fizic D7

// MODIFICARE: Am mutat butonul pe D3 pentru a nu se suprapune cu TRIG_PIN
#define START_BUTTON_PIN D3

// ---------------- FUNCTII DE MISCARE ----------------

// Merge inainte
void moveForward() {
  analogWrite(IN1_PIN, motorSpeed);
  analogWrite(IN2_PIN, 0);
  analogWrite(IN3_PIN, motorSpeed);
  analogWrite(IN4_PIN, 0);
}

// Rotire dreapta
void rotateRight() {
  analogWrite(IN1_PIN, motorSpeed);
  analogWrite(IN2_PIN, 0);
  analogWrite(IN3_PIN, 0);
  analogWrite(IN4_PIN, motorSpeed);
}

// Oprire motoare
void stopMotors() {
  analogWrite(IN1_PIN, 0);
  analogWrite(IN2_PIN, 0);
  analogWrite(IN3_PIN, 0);
  analogWrite(IN4_PIN, 0);
}

// ---------------- SETUP ----------------
void setup() {

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // MODIFICARE: Utilizarea rezistentei interne pull-up specifice ESP8266
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  // Asteapta pana se apasa butonul (conectat intre pinul D3 si GND)
  while (digitalRead(START_BUTTON_PIN) == HIGH) {
    // asteapta
  }

  digitalWrite(ENABLE_PIN, HIGH);  // porneste motoarele
}

// ---------------- LOOP ----------------
void loop() {

  int distanta = sonar.ping_cm();

  if (distanta > 0 && distanta < distantaStop) {

    stopMotors();
    delay(200);

    rotateRight();
    delay(300);

    stopMotors();
    delay(200);
  }
  else {
    moveForward();
  }

  delay(100);
}