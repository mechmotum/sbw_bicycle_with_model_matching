#include <SPI.h>  // include the SPI library:

//pin location

const int ssimu = 10;
const int sshand = 24;
const int ssfork = 25;
const int pwrencoders = 26;


void setup() {
  // set the slaveSelectPin as an output:
  
  pinMode (ssimu, OUTPUT);
  pinMode (sshand, OUTPUT);
  pinMode (ssfork, OUTPUT);
  pinMode (pwrencoders, OUTPUT);
  // initialize SPI:
 
  SPI.begin(); 
   Serial.begin(500000);
}


void loop() {
  digitalWrite(ssimu,HIGH); // setting HIGH slave selection pin to disable IMU
  digitalWrite(sshand,LOW);  // setting LOW slave selection pin to enable handlebar encoder
  digitalWrite(pwrencoders,HIGH); // setting HIGH PA10 pin of external pcb connected to pin 26 of teensie to enable power to my encoders
  digitalWrite(ssfork,HIGH); // setting HIGH slave selection pin to disable fork encoder
  
  SPI.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE3));  // set frequency to 125 Khz-4 Mhz, encoder transmit first the MSB, Clock Idles High Latch on the initial clock edge, sample on the subsequent edge
  
uint16_t mybyte; // declaring variable to unsigned 16-bit integer
mybyte= SPI.transfer16(0); // transfering 16 bits to MOSI and reading what comes back to MISO
uint16_t mybyte1; //declaring variable to unsigned 16-bit integer
//From my byte1 you need to mask out 3 bits the first MSB and the two LSB;
  
  mybyte1 = (mybyte &= 0x7fff)>>2; // First AND bit operator is used bit mask 0111 1111 1111 1111 the 16bit is set to zero. 
//afterwards the bits are shifted two positions to the left. mybyte is now a 14bit with the MSB=0 and the upcoming 13 bit coming from the encoder signal.
  // 
 SPI.endTransaction(); // ending transaction
 delayMicroseconds(20); // delay between 12.5 μs ≤ tm ≤ 20.5 μs for SSI timeout
   Serial.print("Fork(deg)=");
    Serial.print(mybyte1*360/8191);
      Serial.print(",Counts= ");
    Serial.print(mybyte1);
    
   Serial.println(); 
 
}
