#include <Arduino.h>

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-relay-module-ac-web-server/
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

/*
  Turns on an Electromagnet on for one second, then off for one second, repeatedly.
  This example code is in the public domain.
*/

int Electromagnet = 25;
int PlaceholderButton = 4;

int buttonState = 0;


// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  // initialize the digital pin as an output.
  pinMode(Electromagnet, OUTPUT);
  pinMode(PlaceholderButton, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  buttonState=digitalRead(PlaceholderButton);
  Serial.println(buttonState);

  if(buttonState != HIGH){
    digitalWrite(Electromagnet, HIGH);   // turn the Electromagnet on (HIGH is the voltage level)
  }
  else {
    digitalWrite(Electromagnet, LOW);    // turn the Electromagnet off by making the voltage LOW
  }
  delay(1000); // wait for a second
}