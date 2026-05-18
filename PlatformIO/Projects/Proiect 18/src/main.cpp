#include <Arduino.h>

#define ENABLE_PIN 16
#define IN1_PIN 2
#define IN2_PIN 14
#define IN3_PIN 12
#define IN4_PIN 13
#define START_BUTTON_PIN 0

const int motorSpeed = 600;

void moveForward()
{
    analogWrite(IN1_PIN, motorSpeed);
    analogWrite(IN2_PIN, 0);
    analogWrite(IN3_PIN, motorSpeed);
    analogWrite(IN4_PIN, 0);
}

void stopMotors()
{
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, 0);
    analogWrite(IN3_PIN, 0);
    analogWrite(IN4_PIN, 0);
}

void setup()
{
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    pinMode(START_BUTTON_PIN, INPUT);
    digitalWrite(START_BUTTON_PIN, HIGH);

    digitalWrite(ENABLE_PIN, HIGH);
}



void loop()
{
    if (digitalRead(START_BUTTON_PIN) == LOW)
    {
        moveForward();
        delay(400);
        stopMotors();
    }
}
