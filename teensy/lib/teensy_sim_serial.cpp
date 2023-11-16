#include "teensy_sim_serial.h"

float byte_rx_float32(){
  if(Serial.available()){
    uint8_t bytes_rx[NUMPY_FLOAT32_BYTES];
    float f_input;

    for(uint8_t i=0; i<NUMPY_FLOAT32_BYTES; i++){
      bytes_rx[i] = (uint8_t)Serial.read();
    }

    static_assert(sizeof(float) == NUMPY_FLOAT32_BYTES, "float size is expected to be 4 bytes");
    memcpy(&f_input, bytes_rx, NUMPY_FLOAT32_BYTES);
    return f_input;
  }
  return -99.0F; //TODO: handle the fact that you may call this function while there is nothing in the buffer
}

void byte_tx_float32(float* output){
    static_assert(sizeof(float) == NUMPY_FLOAT32_BYTES, "float size is expected to be 4 bytes");
    uint8_t bytes_out[NUMPY_FLOAT32_BYTES];
    memcpy(bytes_out, output, NUMPY_FLOAT32_BYTES);

    for(uint8_t i=0; i<NUMPY_FLOAT32_BYTES; i++){
      Serial.write(bytes_out[i]);
    }
}

/* SITES
https://forum.arduino.cc/t/how-to-covert-4-bytes-to-float/612320
https://www.tutorialspoint.com/c_standard_library/c_function_memcpy.htm
*/