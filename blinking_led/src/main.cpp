#include <Arduino.h>

uint8_t ledPin = 13;
uint8_t btnPin = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(btnPin, INPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int btnStatus = digitalRead(btnPin);

  if (btnStatus == HIGH){
    digitalWrite(ledPin, HIGH);
    
  } else {
    digitalWrite(ledPin, LOW);

  }

}

