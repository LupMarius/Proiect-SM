#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>



#define TRIG_PIN 15
#define ECHO_PIN 5
#define MAX_DIST 200

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

Servo turela;

int myFunction(int, int);


void setup()
{
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  Serial.println("Salut");
  turela.attach(4);
  turela.write(90);
}

void loop()
{
  if (Serial.available() > 0)
  {
    char tastaApasata = Serial.read();

    if (tastaApasata == 'A' || tastaApasata == 'a')
    {
      digitalWrite(2, LOW);
      Serial.println("LED aprins!");
    }

    if (tastaApasata == 'S' || tastaApasata == 's')
    {
      digitalWrite(2, HIGH);
      Serial.println("LED stins!");
    }
      if (tastaApasata == 'M' || tastaApasata == 'm')
      {
      turela.write(120);
      }
       
      if (tastaApasata == 'N' || tastaApasata == 'n')
{ 
  turela.write(60);
}
    if (tastaApasata == 'U' || tastaApasata == 'u')
{
  unsigned int distanta = sonar.ping_cm();
  Serial.print("Distanta: ");
  Serial.print(distanta);
  Serial.println(" cm");
}


  }
}
