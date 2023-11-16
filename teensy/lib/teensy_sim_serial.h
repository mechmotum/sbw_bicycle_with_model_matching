#ifndef TEENSY_SIM_SERIAL_
#define TEENSY_SIM_SERIAL_

#include <Arduino.h>

// Constants
//number of bytes in numpy's float32
const uint8_t NUMPY_FLOAT32_BYTES = 4;


// Functions
float byte_rx_float32();
void byte_tx_float32(float* output);

#endif //TEENSY_SIM_SERIAL_