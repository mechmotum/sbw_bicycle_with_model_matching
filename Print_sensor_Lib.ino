
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h> // library used to measure encoder counts of handlebars and fork https://www.pjrc.com/teensy/td_libs_Encoder.html
 #define rearencoderA 0
  #define pedalencoderA 2

 unsigned long duration;
 unsigned long duration1;
 unsigned long freq;
 unsigned long freq1;
 unsigned long rpm;
 unsigned long rpm1;
 unsigned long U;
 unsigned long rear;
 long Handcounts, Forkcounts;
Encoder Handlebars(4,5);
Encoder Fork(6,7);

void setup() {
  Serial.begin(19200);
  pinMode (rearencoderA,INPUT);
  pinMode (pedalencoderA,INPUT);
}

void loop() {
  Handcounts = Handlebars.read();
  Forkcounts = Fork.read();
// This program calculates the bicycle forward velocity in km/h
    duration = pulseIn(rearencoderA,HIGH,250000); //  Returns the length of the pulse in microseconds when signal output A is high
    freq=1200000/(duration*2); // The period of the is 2 times the duration of the pulse of output A https://tushev.org/articles/arduino/9/measuring-frequency-with-arduino
rpm = (freq*60)/96; // 96 are the line counts of the encoder http://www.quantumdev.com/finding-the-rpm-of-an-optical-encoder-using-an-oscilloscope/
U=6.28*0.3355*rpm*60/1000; // http://answers.tutorvista.com/489133/what-is-the-formula-for-converting-rpm-to-kph.html#
// This program calculates the pedal cadence speed in rpm
duration1 = pulseIn(pedalencoderA, HIGH,250000); //  Returns the length of the pulse in microseconds when signal output A is high
freq1=1200000/(duration1*2); // The period is 2 times the duration of the pulse of output A. 12e6 is set based on speedometer output comparison https://tushev.org/articles/arduino/9/measuring-frequency-with-arduino
rpm1 = (freq1*60)/96; // 96 are the line counts of the encoder http://www.quantumdev.com/finding-the-rpm-of-an-optical-encoder-using-an-oscilloscope/
    Serial.print("Handlebar(deg) = ");
    Serial.print(Handcounts*360/294908);
    Serial.print(", Fork(deg) = ");
    Serial.print(Forkcounts*360/294908);
    Serial.print(", Speed(km/h) = ");
    Serial.print(U);
    Serial.print(",Pedal(rpm)= ");
    Serial.print(rpm1);
    Serial.println();
  }


