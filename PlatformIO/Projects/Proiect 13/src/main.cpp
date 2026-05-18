#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

  void loop() 
{
       digitalWrite(2, HIGH);
       delay(1000);
       digitalWrite(2, LOW);
       delay(1000);
}



int myFunction(int x, int y) {
  return x + y;
}