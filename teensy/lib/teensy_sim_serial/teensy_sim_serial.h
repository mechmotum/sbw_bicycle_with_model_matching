/*
___[ teensy_sim_serial ]___
This library contains the functions for sending and receiving 
variables from the Teensy to a PC and back over serial COM
*/

#ifndef TEENSY_SIM_SERIAL_
#define TEENSY_SIM_SERIAL_

#include <Arduino.h>
/* SITES USED
https://forum.arduino.cc/t/how-to-covert-4-bytes-to-float/612320
https://www.tutorialspoint.com/c_standard_library/c_function_memcpy.htm
https://stackoverflow.com/questions/22127041/c-pass-variable-type-to-function
https://stackoverflow.com/questions/648900/c-templates-undefined-reference
*/

//----[Bytes size check
//number of bytes in numpy's types. Check with the 'check_byte_size.py' script
const uint8_t NUMPY_INT8_BYTES = 1;
const uint8_t NUMPY_UINT16_BYTES = 2;
const uint8_t NUMPY_INT32_BYTES = 4;
const uint8_t NUMPY_FLOAT32_BYTES = 4;

static_assert(sizeof(int8_t) == NUMPY_INT8_BYTES, "systems int8_t and numpy int8 have a different byte size");
static_assert(sizeof(uint16_t) == NUMPY_UINT16_BYTES, "systems uint16_t and numpy uint16 have a different byte size");
static_assert(sizeof(int32_t) == NUMPY_INT32_BYTES, "systems int32_t and numpy int32 have a different byte size");
static_assert(sizeof(float) == NUMPY_FLOAT32_BYTES, "systems float and numpy float32 have a different byte size");

//---[Template Functions
// Send variable
template<typename T>
T byte_rx(){
  while(!Serial.available()){} //TODO: throw in a 'time out' timer
  
  uint8_t bytes_rx[sizeof(T)];
  T input;

  for(uint8_t i=0; i<sizeof(T); i++){
    bytes_rx[i] = (uint8_t)Serial.read();
  }

  static_assert(sizeof(input) == sizeof(bytes_rx), "var to be copied has different byte size than destination");
  memcpy(&input, bytes_rx, sizeof(T));
  return input;
}

// Receive variable
template<typename T>
void byte_tx(T* output){
    uint8_t bytes_out[sizeof(T)];
    static_assert(sizeof(bytes_out) == sizeof(*output), "var to be copied has different byte size than destination");
    memcpy(bytes_out, output, sizeof(T));

    for(uint8_t i=0; i<sizeof(T); i++){
      Serial.write(bytes_out[i]);
    }
}

#endif //TEENSY_SIM_SERIAL_




/* FOR DEBUGGING (REPLACE MAIN WITH THIS)
#include <Arduino.h>
#include "teensy_sim_serial.h"

void setup(){
  Serial.begin(9600); // Communication with PC through micro-USB
  while(!Serial){} //Wait with startup untill serial communication has started
}

void loop(){
  int32_t speed_ticks = byte_rx<int32_t>();
  int8_t torque_h = byte_rx<int8_t>();
  float omega_x = byte_rx<float>();
  float omega_y = byte_rx<float>();
  float omega_z = byte_rx<float>();
  uint16_t encoder_h = byte_rx<uint16_t>();
  uint16_t encoder_f = byte_rx<uint16_t>();
  byte_tx<int32_t>(&speed_ticks);
  Serial.println();
  byte_tx<int8_t>(&torque_h);
  Serial.println();
  byte_tx<float>(&omega_x);
  Serial.println();
  byte_tx<float>(&omega_y);
  Serial.println();
  byte_tx<float>(&omega_z);
  Serial.println();
  byte_tx<uint16_t>(&encoder_h);
  Serial.println();
  byte_tx<uint16_t>(&encoder_f);
  Serial.println();
}
*/