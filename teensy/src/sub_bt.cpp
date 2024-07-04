/*
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

void setup(){
  Serial.begin(9600);
  Serial1.begin(9600); //HC-05 deafault speed in AT DATA mode
  // Serial1.begin(38400); //HC-05 deafault speed in AT command mode

  //=====================[Wait for connection]========================
  // NOTE as the bluetooth modules take some time to connect, the first
  // few messages are lost. By first waiting untill there is a connection
  // before entering the loop, no messages are lost.

  // MAIN: send a signal, wait for return message
  // while(1){
  //   Serial1.write(1);
  //   if(Serial1.available()) break;
  // }

  // SUB: Wait for an incomming message, if recieved send return message
  while(!Serial){} // Only start the pairing sequence once the sub Serial monitor is open

  while(1){
    if(Serial1.available()){
      Serial1.write(1);
      break;
    }
  }
}

void loop(){
  //MAIN
  // Serial1.println("Hello world");
  // delay(100);

  //SUB
  if(Serial1.available()){
    Serial.write(Serial1.read());
  }
}
*/


// For direct communication with the bluetooth module.
// Used for programming the BT module, such as its
// role, command mode, and bind address.
/*
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

void setup(){
  Serial.begin(9600);
  // Serial1.begin(9600); //HC-05 deafault speed in AT DATA mode
  Serial1.begin(38400); //HC-05 deafault speed in AT command mode
}

void loop(){
  if(Serial.available()){
    Serial1.write(Serial.read());
  }

  if(Serial1.available()){
    Serial.write(Serial1.read());
  }
}
*/