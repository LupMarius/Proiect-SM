#include <Arduino.h>
#include <Servo.h>

Servo turela;
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
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


  }
}
