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
float return_scaling(uint64_t iteration);
//void write_thread();
//void block_for_us(unsigned long delta_time_us);
//void block_until_us(unsigned long continue_time_us);
//void yield_for_us(unsigned long delta_time_us);
//void yield_until_us(unsigned long continue_time_us);

//=================================== Pins ===================================//
// SPI
// Pins 11, 12, and 13 are used as MOSI, MISO, and SCK by default for SPI0.
const uint8_t cs_imu = 10; // Chip Select for MPU9250
const uint8_t cs_hand = 24; // Chip Select for handlebar encoder
const uint8_t cs_fork = 25; // Chip Select for fork encoder
bfs::Mpu9250 IMU(&SPI, cs_imu); // MPU9250 object
// Analog
const uint8_t a_force = 20; // Analog output of the force transducer
const uint8_t a_torque = 21; // Analog output of the torque sensor
const uint8_t a_fork = 40; // Analog output of the fork motor drive
const uint8_t a_hand = 41; // Analog output of the handlebar motor drive
// Encoders
Encoder wheel_counter(2, 3); // Rear wheel speed encoder
Encoder pedal_counter(23, 22); // Pedal cadence encoder
// Motor control
const uint8_t pwm_pin_hand = 8; // Send PWM signals to the handlebar motor
const uint8_t pwm_pin_fork = 9; // Send PWM signals to the fork motor
const uint8_t switch_hand = 29; // Used to turn the handlebar motor on or off
const uint8_t switch_fork = 30; // Used to turn the fork motor on or off
// Other
const uint8_t encoder_power = 26; // Make HIGH to send power to the encoders
const uint8_t hand_led = 27; // LED installed on the handlebars
const uint8_t hand_switch = 28; // A switch installed on the handlebars

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
float error_prev = 0.0f;
uint32_t error_time_prev = 0.0f;
uint64_t haptics_iteration_counter = 0; // Ensure it never overflows
// Time
elapsedMicros sinceLast; // How long has passed since last loop execution
// IMU
/*float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float temp;*/
// Analog
float val_fork = 0.0f;
float val_hand = 0.0f;
float val_torque = 0.0f;
float val_force = 0.0f;
float a_torque_fork = 0.0f;
float a_torque_hand = 0.0f;
float analog_torque_sensor_Nm = 0.0f;
float analog_force_transducer_N = 0.0f;
// Gains
float Kp_f = 2.0f;
float Kd_f = 0.029f;
float Kp_h = 0.9f;
float Kd_h = 0.012f;
// Other
uint8_t hand_switch_state = 0;

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
    digitalWrite(cs_imu, HIGH);
  pinMode (cs_hand, OUTPUT);
    digitalWrite(cs_hand, HIGH);
  pinMode (cs_fork, OUTPUT);
    digitalWrite(cs_fork, HIGH);
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
  float angle_hand = (float)hand_dac * 360.0f / 8191.0f - 153.65f; 
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
  float angle_fork = -(float)fork_dac * 360.0f / 8191.0f - 100.65f; 
  if (angle_fork < -180.0f) // CW fork rotation gives -360 deg-> -310 deg
    angle_fork = angle_fork + 360.0f; // Add 360 to get 0-180 deg CCW

  //---------------------- Calculate error derivative ------------------------//
  // Set the very first handlebar and fork encoder values to 0
  if (haptics_iteration_counter < 10) { 
    angle_hand = 0.0f;
    angle_fork = 0.0f;
  }
  // Calculation of dT in seconds
  uint32_t error_time_curr = micros();
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
  uint64_t pwm_command_fork = (command_fork * -842.795 + 16384);
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
  uint64_t pwm_command_hand = (command_hand * -842.795 + 16384);
  pwm_command_hand = constrain(pwm_command_hand, 0, 32768);
  analogWrite(pwm_pin_hand, pwm_command_hand);

  //----------------------------- Read IMU data ------------------------------//
  bool status = IMU.Read();
  float accelY = IMU.accel_x_mps2();
  float accelX = IMU.accel_y_mps2();
  float accelZ = IMU.accel_z_mps2();
  float gyroY = IMU.gyro_x_radps();
  float gyroX = IMU.gyro_y_radps();
  float gyroZ = IMU.gyro_z_radps();
  float temp = IMU.die_temp_c();

  //------------------------ Printing to serial port -------------------------//
  // Limit the printing rate
  if (haptics_iteration_counter % 100 == 0) {
    Serial.print("Status: ");
    Serial.print(status);
    // Angles
    //Serial.print(",Hand(deg)=");
    //Serial.print(angle_hand);
    //Serial.print(",Fork(deg)=");
    //Serial.print(angle_fork);
    // Torque
    //Serial.print(",Torque_handlebar(Nm)=");
    //Serial.print(command_hand);
    //Serial.print(",Torque_fork(Nm)=");
    //Serial.print(command_fork);
    //Serial.print(",Hand(Counts)= ");
    //Serial.print(hand_dac);
    //Serial.print(",Fork(Counts)= ");
    //Serial.print(fork_dac);
    //Serial.print(",Time Taken= ");
    //Serial.print(sinceLast);
    // IMU
    Serial.print(",AccelX= ");
    Serial.print(accelX);
    Serial.print(",AccelY= ");
    Serial.print(accelY);
    Serial.print(",AccelZ= ");
    Serial.print(accelZ);
    Serial.print(",GyroX= ");
    Serial.print(gyroX);
    Serial.print(",GyroY= ");
    Serial.print(gyroY);
    Serial.print(",GyroZ= ");
    Serial.print(gyroZ);
    Serial.print(",Temp= ");
    Serial.print(temp);
    // New line
    Serial.println();
  }

  //------------------------ Move to the next loop ---------------------------//
  if (sinceLast > 1000) {
    Serial.println("MISSED HAPTICS DEADLINE");
  }
  haptics_iteration_counter += 1;
}

float return_scaling(uint64_t iteration) {
  if (iteration <= (uint64_t) 6000) 
    return 35.0f;
  if (iteration <= (uint64_t) 7000)
    return 30.0f;
  if (iteration <= (uint64_t) 8000)
    return 25.0f;
  if (iteration <= (uint64_t) 9000)
    return 20.0f;
  if (iteration <= (uint64_t) 10000)
    return 15.0f;
  if (iteration <= (uint64_t) 11000)
    return 10.0f;
  if (iteration <= (uint64_t) 12000)
    return 5.0f;
  if (iteration <= (uint64_t) 13000)
    return 2.5f;
  
  return 1.0f;
}