#include <Arduino.h>
#include <SD.h>
#include <TeensyThreads.h> // https://github.com/ftrias/TeensyThreads
#include <SPI.h> 
#include <Encoder.h>
#include "mpu9250.h" // https://github.com/bolderflight/MPU9250


//=========================== Function definitions ===========================//
void setup();
void loop();
void haptics();
float return_scaling(int iteration);
//void write_thread();
//void block_for_us(unsigned long delta_time_us);
//void block_until_us(unsigned long continue_time_us);
//void yield_for_us(unsigned long delta_time_us);
//void yield_until_us(unsigned long continue_time_us);
//float clamp_f32(float value, float minv, float maxv);
//int clamp_i32(int value, int minv, int maxv);

//=================================== Pins ===================================//
// SPI
// Pins 11, 12, and 13 are used as MOSI, MISO, and SCK by default for SPI0.
const int cs_imu = 10; // Chip Select for MPU9250
const int cs_hand = 24; // Chip Select for handlebar encoder
const int cs_fork = 25; // Chip Select for fork encoder
bfs::Mpu9250 IMU(&SPI, cs_imu); // MPU9250 object
// Analog
const int a_force = 20; // Analog output of the force transducer
const int a_torque = 21; // Analog output of the torque sensor
const int a_fork = 40; // Analog output of the fork motor drive
const int a_hand = 41; // Analog output of the handlebar motor drive
// Encoders
Encoder wheel_counter(2, 3); // Rear wheel speed encoder
Encoder pedal_counter(23, 22); // Pedal cadence encoder
// Motor control
const int pwm_pin_hand = 8; // Send PWM signals to the handlebar motor
const int pwm_pin_fork = 9; // Send PWM signals to the fork motor
const int switch_hand = 29; // Used to turn the handlebar motor on or off
const int switch_fork = 30; // Used to turn the fork motor on or off
// Other
const int encoder_power = 26; // Make HIGH to send power to the encoders
const int hand_led = 27; // LED installed on the handlebars
const int hand_switch = 28; // A switch installed on the handlebars

//============================= Global variables =============================//
// Rear wheel encoder
const uint32_t WHEEL_COUNTS_LENGTH = 200;
int32_t wheel_counts[WHEEL_COUNTS_LENGTH] = {0};
uint32_t wheel_counts_index = 0;
// Pedal encoder
const uint32_t PEDAL_COUNTS_LENGTH = 500;
int32_t pedal_counts[PEDAL_COUNTS_LENGTH] = {0};
uint32_t pedal_counts_index = 0;
// Haptics
float error_prev = 0;
float error_time_prev = 0;
unsigned int haptics_iteration_counter = 0;
// Time
elapsedMicros sinceLast; // How long has passed since last loop execution
// IMU
float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float temp;
// Analog
float val_fork = 0;
float val_hand = 0;
float val_torque = 0;
float val_force = 0;
float a_torque_fork;
float a_torque_hand;
float analog_torque_sensor_Nm;
float analog_force_transducer_N;
// Gains
float Kp_f = 2;
float Kd_f = 0.029;
float Kp_h = 0.9;
float Kd_h = 0.012;
// Other
int hand_switch_state = 0;

//============================== Main Setup ==================================//
void setup() {
  SPI.begin();
  Serial.begin(9600);
  while (!Serial); // Wait for Serial port to connect
  // Setup SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Please insert SD card!");
    while (1);
  }
  // Setup OUTPUT pins
  pinMode (cs_imu, OUTPUT);
  pinMode (cs_hand, OUTPUT);
  pinMode (cs_fork, OUTPUT);
  pinMode (encoder_power, OUTPUT);
    digitalWrite(encoder_power, HIGH); // HIGH to enable power to the encoders
  pinMode (hand_led, OUTPUT);
  pinMode (switch_fork, OUTPUT);
    digitalWrite(switch_fork, HIGH); // Set HIGH to enable motor
  pinMode (switch_hand, OUTPUT);
    digitalWrite(switch_hand, HIGH); // Set HIGH to enable motor
  // Setup PWM pins
  analogWriteResolution(15);
  pinMode (pwm_pin_fork, OUTPUT);
    analogWriteFrequency(pwm_pin_fork, 4577.64);
    analogWrite(pwm_pin_fork, 16384);
  pinMode (pwm_pin_hand, OUTPUT);
    analogWriteFrequency(pwm_pin_hand, 4577.64);
    analogWrite(pwm_pin_hand, 16384);
  // Setup INPUT pins
  pinMode (hand_switch, INPUT); // making handlebar switch high changes fork controller gains
  // Disable SPI communication with encoders
  digitalWrite(cs_fork, HIGH);
  digitalWrite(cs_hand, HIGH);
  // Initialize IMU
  if (!IMU.Begin()) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
  }
  IMU.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G); // +- 4g
  IMU.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_250DPS); // +- 250 deg/s
  sinceLast = 0;
}

//============================== Main Loop ===================================//
void loop(){
  if (sinceLast >= 1000){
    sinceLast = sinceLast - 1000;
    haptics();
  }
}

//============================== Haptics =====================================//
void haptics(){
  hand_switch_state = digitalRead(hand_switch);
  //---------------- SPI communication with Handlebar encoder ----------------//
  digitalWrite(cs_imu, HIGH); // HIGH to disable IMU communication
  digitalWrite(cs_fork, HIGH); // HIGH to disable fork encoder communication
  // Set frequency to 125 Khz-4 Mhz 
  // encoder transmit first the MSB
  // Clock Idles High Latch on the initial clock edge
  // sample on the subsequent edge
  SPI.beginTransaction(SPISettings(225000, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_hand, LOW); // LOW to enable handlebar encoder communication
  // Transfer 16 bits to MOSI and read what comes back to MISO
  uint16_t hand_dac = SPI.transfer16(0); 
  // From my byte1 you need to mask out 3 bits the first MSB and the two LSB;
  // First AND bit operator is used bit mask 0111 1111 1111 1111 the 16bit is set to zero.
  hand_dac = (hand_dac & 0x7fff) >> 2;
  digitalWrite(cs_hand, HIGH); // HIGH to disable handlebar encoder communication
  SPI.endTransaction();

  //------------------ SPI communication with Fork encoder -------------------//
  SPI.beginTransaction(SPISettings(225000, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_fork, LOW); // LOW to enable fork encoder communication
  uint16_t fork_dac = SPI.transfer16(0);
  fork_dac = (fork_dac & 0x7fff) >> 2;
  digitalWrite(cs_fork, HIGH); // disable fork encoder
  SPI.endTransaction(); // ending transaction

  //------------------------- Processing encoder data ------------------------//
  /* NOTE: The two encoders are mounted opposite to each other.
    Therefore, we have different counts directions. More specifically, 
    CCW rotation of the handlebar encoder gives 360 deg -> 310 deg,
    whereas, CCW rotation of the fork encoder gives 0-55 degrees.
    The rotational direction must be the same for both encoders, 
    the encoders must give an output 0-180 degrees. 
    Two if statements are used for these reasons.
  */
  // Encoder counts to degrees. 152.20 is the offset (angle_hand-angle_fork).
  // 153.65
  float angle_hand = (float)hand_dac * 360.0f / 8191.0f - 153.65; 
  if (angle_hand > 180.0f) // CCW handlebar rotation gives 360 deg-> 310 deg.
    angle_hand = angle_hand - 360.0f; // Subtract 360 to get 0-180 deg CCW
  /* NOTE: The fork mechanical range is +- 42 degrees.
    Handlebars do not have a mechanical limit.
    Software limits are set below.
    If you do not set this limits the motor folds back.
  */
  angle_hand = constrain(angle_hand, -42.0f, 42.0f); // Software limits

  // Encoder counts to degrees. substracting -99.2 deg to align fork with handlebars and - to get minus values from fork encoder.
  // 100.65
  float angle_fork = -(float)fork_dac * 360.0f / 8191.0f - 100.65; 
  if (angle_fork < -180.0f) // CW fork rotation gives -360 deg-> -310 deg
    angle_fork = angle_fork + 360.0f; // Add 360 to get 0-180 deg CCW

  //---------------------- Calculate error derivative ------------------------//
  // Set the very first handlebar and fork encoder values to 0
  if (haptics_iteration_counter < 10) { 
    angle_hand = 0.0f;
    angle_fork = 0.0f;
  }
  // Calculation of dT in seconds
  unsigned long error_time_curr = micros();
  float error_time_diff = ((float) (error_time_curr - error_time_prev)) / 1000000.0f;
  error_time_prev = error_time_curr;
  // Calculation of dError
  float error_curr = (angle_hand - angle_fork);
  float error = (error_curr - error_prev) / error_time_diff;
  error_prev = error_curr;

  //------------------------ Calculate fork torque ---------------------------//
  float command_fork = (Kp_f*(angle_hand - angle_fork) + Kd_f*error);

  //------------- Reduction of torque in the first 14 seconds ----------------//
  command_fork = command_fork / return_scaling(haptics_iteration_counter);
  
  //---------------------- Find the fork PWM command -------------------------//
  int pwm_command_fork = (command_fork * -842.795 + 16384);
  pwm_command_fork = constrain(pwm_command_fork, 0, 32768);
  analogWrite(pwm_pin_fork, pwm_command_fork);

  //----------------------- Calculate handlebar torque -----------------------//
  float command_hand = (Kp_h*(angle_hand - angle_fork) + Kd_h*error);

  //------------- Reduction of torque in the first 14 seconds ----------------//
  if (haptics_iteration_counter < 13000) {
    command_hand = command_hand / return_scaling(haptics_iteration_counter);
  }
  else if (haptics_iteration_counter >= 13000) {
    digitalWrite(hand_led, HIGH); // Turn on the LED to show that bike is ready
    if (hand_switch_state == 1) { // Turn off the haptics with the switch
      command_hand = 0;
    }
  }

  //-------------------- Find the handlebar PWM command ----------------------//
  int pwm_command_hand = (command_hand * -842.795 + 16384);
  pwm_command_hand = constrain(pwm_command_hand, 0, 32768);
  analogWrite(pwm_pin_hand, pwm_command_hand);

  //------------------------ Printing to serial port -------------------------//
  // Limit the printing rate
  if (haptics_iteration_counter % 300 == 0) {
    Serial.print("State:");
    Serial.print(hand_switch_state);
    // Angles
    Serial.print(",Hand(deg)=");
    Serial.print(angle_hand);
    Serial.print(",Fork(deg)=");
    Serial.print(angle_fork);
    // Torque
    Serial.print(",Torque_handlebar(Nm)=");
    Serial.print(command_hand);
    Serial.print(",Torque_fork(Nm)=");
    Serial.print(command_fork);
    Serial.print(",Hand(Counts)= ");
    Serial.print(hand_dac);
    Serial.print(",Fork(Counts)= ");
    Serial.print(fork_dac);
    Serial.print(",Time Taken= ");
    Serial.print(sinceLast);
    Serial.println();
  }

  //------------------------ Move to the next loop ---------------------------//
  if (sinceLast > 1000) {
    Serial.println("MISSED HAPTICS DEADLINE");
  }
  haptics_iteration_counter += 1;
}

float return_scaling(int iteration) {
  if (iteration <= 1000)
    return 1000;
  if (iteration <= 2000)
    return 500;
  if (iteration <= 3000)
    return 250;
  if (iteration <= 4000)
    return 100;
  if (iteration <= 5000)
    return 50;
  if (iteration <= 6000) 
    return 35;
  if (iteration <= 7000)
    return 30;
  if (iteration <= 8000)
    return 25;
  if (iteration <= 9000)
    return 20;
  if (iteration <= 10000)
    return 15;
  if (iteration <= 11000)
    return 10;
  if (iteration <= 12000)
    return 5;
  if (iteration <= 13000)
    return 2.5;
  
  return 1;
}