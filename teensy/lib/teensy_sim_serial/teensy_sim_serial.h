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
template<typename T>
T byte_rx(){
  if(Serial.available()){
    uint8_t bytes_rx[sizeof(T)];
    T input;

    for(uint8_t i=0; i<sizeof(T); i++){
      bytes_rx[i] = (uint8_t)Serial.read();
    }

    static_assert(sizeof(input) == sizeof(bytes_rx), "var to be copied has different byte size than destination");
    memcpy(&input, bytes_rx, sizeof(T));
    return input;
  }
  T no_serial = -98;
  return no_serial; //TODO: handle the fact that you may call this function while there is nothing in the buffer
}

template<typename T>
void byte_tx(T* output){
    uint8_t bytes_out[sizeof(T)];
    static_assert(sizeof(bytes_out) == sizeof(*output), "var to be copied has different byte size than destination");
    memcpy(bytes_out, output, sizeof(T));

    for(uint8_t i=0; i<sizeof(T); i++){
      Serial.write(bytes_out[i]);
    }
}

//----[Non template functions
// float byte_rx_float32();
// void byte_tx_float32(float* output);

#endif //TEENSY_SIM_SERIAL_