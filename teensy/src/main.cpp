#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include "SdFat.h"
#include "RingBuf.h" //From sdFat library
#include "mpu9250.h"

float byte_rx_float();
void byte_tx_float(float* output);

void setup(){
  Serial.begin(9600);
  while(!Serial){}
}

void loop(){
  if(Serial.available()){ 
    float w_x, w_y, w_z, delta_h, delta_f, vel, T_h;
    w_x = byte_rx_float();
    w_y = byte_rx_float();
    w_z = byte_rx_float();
    delta_h = byte_rx_float();
    delta_f = byte_rx_float();
    vel = byte_rx_float();
    T_h = byte_rx_float();

    float T_out_h = w_x + w_y + w_z + delta_h + delta_f + vel + T_h;
    float T_out_f = T_out_h - 2;
    
    byte_tx_float(&T_out_h);
    byte_tx_float(&T_out_f);
    Serial.println();
  }
}

float byte_rx_float(){
  if(Serial.available()){
    uint8_t bytes_in[4];
    float f_input;

    for(uint8_t i=0; i<4; i++){
      bytes_in[i] = (uint8_t)Serial.read();
    }
    static_assert(sizeof(float) == 4, "float size is expected to be 4 bytes");

    memcpy(&f_input, bytes_in, 4);
    return f_input;
  }
  return -99.0F; //TODO: handle the fact that you may call this function while there is nothing in the buffer
}

void byte_tx_float(float* output){
    static_assert(sizeof(float) == 4, "float size is expected to be 4 bytes");
    uint8_t bytes_out[4];
    memcpy(bytes_out, output, 4);

    for(uint8_t i=0; i<4; i++){
      Serial.write(bytes_out[i]);
    }
}