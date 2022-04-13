#include <Arduino.h>
#include <SD.h>
#include <TeensyThreads.h> // https://github.com/ftrias/TeensyThreads
#include <SPI.h> 
#include <Encoder.h>
#include "mpu9250.h" // https://github.com/bolderflight/MPU9250

//=========================== Function definitions ===========================//
void setup();
void loop();
void haptics_setup();
void haptics_loop();
void write_thread();
void block_for_us(unsigned long delta_time_us);
void block_until_us(unsigned long continue_time_us);
void yield_for_us(unsigned long delta_time_us);
void yield_until_us(unsigned long continue_time_us);
float clamp_f32(float value, float minv, float maxv);
int clamp_i32(int value, int minv, int maxv);

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


//const int ssimu = 10; // slave selection enable is Low
//const int sshand = 24; // pin of logic gate setting LOW is active
//const int ssfork = 25;
//const int pwrencoders = 26; // making High enable power to encoders
//const int hand_led = 27; //hand_led powers up for 30 seconds to show that self alighment is completed
//const int gain_switch = 28; // pin connected to handlebar switch when High fork controller settings are changed
//const int a_fork = 40; // analog output of fork motor drive

//const int a_hand = 41; // analog output of fork motor drive

//const int a_torque = 21; // analog output of torque sensor

//const int a_force = 20; // analog output of force tranducer

//Encoder wheel_counter(2, 3); //declaring wheel encoder pins for encoder library
//Encoder pedal_counter(23, 22); //declaring pedal encoder pins for encoder library
//bfs::Mpu9250 IMU(&SPI, 10); // an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10

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
long current_time;
unsigned long next_run_time;
float errorlast = 0;
float errorTimeLast = 0;
unsigned int haptics_iteration_counter = 0;
struct HapticsMessage {
  uint32_t index;
  uint32_t start_time;
  uint32_t end_time;
  float angle_hand_rad;
  float angle_fork_rad;
  float torque_fork;
  float torque_handlebar;
  float pedal_cadence;
  float velocity;
  int32_t countspedal;
  int32_t countswheel;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float temp;
  float a_torque_fork;
  float a_torque_hand;
  float analog_torque_sensor_Nm;
  float analog_force_transducer_N;
  float Torque_zero;
  float Dac_zero;
};
// Time
unsigned long time_start_program;
unsigned long current_time_program;
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
// Other
int status;
const int HAPTICS_MESSAGE_BUFFER_CAPACITY = 1000;
HapticsMessage haptics_message_buffer_1[HAPTICS_MESSAGE_BUFFER_CAPACITY];
HapticsMessage haptics_message_buffer_2[HAPTICS_MESSAGE_BUFFER_CAPACITY];
Threads::Mutex message_buffer_mutex;
volatile HapticsMessage * haptics_thread_message_buffer;
volatile uint32_t haptics_thread_message_count;

//============================== Main Setup ==================================//
void setup() {
  SPI.begin(); // intializing SPI
  Serial1.begin(9600); // begin serial communication;
  // while (!Serial) {
  //     Wait for serial port to connect. Needed for native USB.
  // }

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Please insert SD card!");
    while (1);
  }

  haptics_setup();

  if (!threads.setMicroTimer(100)) {
    Serial.println("FAILED TO SET MICRO TIMER");
  }
  threads.setDefaultTimeSlice(1);
  threads.addThread(write_thread);

  digitalWrite(cs_fork, HIGH); // setting HIGH slave selection pin to disable IMU
  digitalWrite(cs_hand, HIGH); // setting LOW slave selection pin to enable handlebar encoder
  unsigned long time1;
  unsigned long time2;
  time_start_program = micros();
  time1 = micros();
  status = IMU.Begin();
  IMU.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G); //This function sets the accelerometer full scale range to the given value
  IMU.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_250DPS);//This function sets the gyroscope full scale range to the given value
  time2 = micros();
  Serial.print(time1);
  Serial.print(",");
  Serial.print(time2);
  Serial.print(",");
  Serial.println(time2 - time1);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
  
  }
  randomSeed(analogRead(33));
  
}

//================================ Main Loop =================================//
void loop() {
  haptics_loop();// calling haptic loop thread here is my main thread
}

//=============================== Haptic Setup ===============================//
void haptics_setup() {
  next_run_time = micros();
  // Setup OUTPUT pins
  pinMode (cs_imu, OUTPUT);
  pinMode (cs_hand, OUTPUT);
  pinMode (cs_fork, OUTPUT);
  pinMode (encoder_power, OUTPUT);
    digitalWrite(encoder_power, HIGH); // HIGH to enable power to the encoders
  pinMode (hand_led, OUTPUT);
  pinMode (switch_fork, OUTPUT);
    digitalWrite(switch_fork, HIGH); // Set HIGH to enable motor
  pinMode (pwm_pin_fork, OUTPUT);
  pinMode (switch_hand, OUTPUT);
    digitalWrite(switch_hand, HIGH); // Set HIGH to enable motor
  pinMode (pwm_pin_hand, OUTPUT);
  // Setup INPUT pins
  pinMode (hand_switch, INPUT); // making handlebar switch high changes fork controller gains
  // Setup Teensy Threads
  Threads::Scope scope(message_buffer_mutex);
  haptics_thread_message_buffer = haptics_message_buffer_1;
  haptics_thread_message_count = 0;
  // Only switch every n timeslices.
  threads.setTimeSlice(threads.id(), 10);
}

//================================ Haptic Loop ===============================//
void haptics_loop() {
  unsigned long start_time = micros();
  unsigned long state = digitalRead(hand_switch);
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
  float angle_hand = (float)hand_dac * 360.0f / 8191.0f - 153.65; 
  if (angle_hand > 180.0f) // CCW handlebar rotation gives 360 deg-> 310 deg.
    angle_hand = angle_hand - 360.0f; // Subtract 360 to get 0-180 deg CCW
  /* NOTE: The fork mechanical range is +- 42 degrees.
    Handlebars do not have a mechanical limit.
    Software limits are set below.
    If you do not set this limits the motor folds back.
  */
  angle_hand = clamp_f32(angle_hand, -45.0f, 45.0f); // Software limits
  float angle_hand_rad = angle_hand * 0.0174533; // Convert to radians

  // Encoder counts to degrees. substracting -99.2 deg to align fork with handlebars and - to get minus values from fork encoder.
  float angle_fork = -(float)fork_dac * 360.0f / 8191.0f - 100.65; 
  if (angle_fork < -180.0f) // CW fork rotation gives -360 deg-> -310 deg
    angle_fork = angle_fork + 360.0f; // Add 360 to get 0-180 deg CCW
  float angle_fork_rad = angle_fork * 0.0174533; // Convert to radians
  // -----------------------------------------------Fork feedback tracking controller------------------------------------------------------------------------
  
  // define variables
  float Kp = 2; // Nick recommends 8 Nm/rad=0.139626 Nm/deg 0.61157
  float Kd = 0.029;  // Nick recommends 0.6 Nm/rad=0.010472 Nm/deg
  //    float Tc = 0.0369; // Torque constant  36.9 mNm/A = 0.0369 Nm/A
  //    const int Tdac = 1;
  float command_fork; // Commanded torque for the handlebars


  signed int pwm_command_fork;

  //--------------------------------------------------------------------Calculation of derivative part----------------------------------------------------------------------------------------------------------------------

  if (haptics_iteration_counter < 10) { //setting handlebar and fork encoder first values to 0, we do that because first values seem to be floating values
    angle_hand = 0.0f;
    angle_fork = 0.0f;
  }

  unsigned long errorTimeNew = micros();//calculation of dt
  float errorTime = ((float) (errorTimeNew - errorTimeLast)) / 1000000.0f; /*get time in s, if you put your time in ms you get very small magnitudes error which has as an impact high Kd gain to have high damping torques.
                                                                        It also makes sense to have the error in rad/s since these parameters will be used afterwards to extend the whipple bicycle model, which has its parameters set in sec*/
  errorTimeLast = errorTimeNew;

  float errornew = (angle_hand - angle_fork); //calculation of derror
  float error = (errornew - errorlast) / errorTime;
  errorlast = errornew;
  //-----------------------------------------------reducing the amount of torque applied to the forkmotor for the first 14 seconds allowing the fork to align safely with the handlebars-------------------------------------------

  if (haptics_iteration_counter > 0 && haptics_iteration_counter < 6000) // 1st of reduction
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error) / 35;
  }
  else if (haptics_iteration_counter > 6000 && haptics_iteration_counter < 7000)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error) / 30;
  }
  else if (haptics_iteration_counter > 7000 && haptics_iteration_counter < 8000)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error) / 25;
  }
  else if (haptics_iteration_counter > 8000 && haptics_iteration_counter < 9000)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error) / 20;
  }
  else if (haptics_iteration_counter > 9000 && haptics_iteration_counter < 10000)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error) / 15;
  }
  else if (haptics_iteration_counter > 10000 && haptics_iteration_counter < 11000)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error) / 10;
  }
  else if (haptics_iteration_counter > 11000 && haptics_iteration_counter < 12000)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error) / 5;
  }
  else if (haptics_iteration_counter > 12000 && haptics_iteration_counter < 13000)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error) / 2.5;
  }
  else if ((haptics_iteration_counter) > 13000 && (state) == 0)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error); //9 th step of reduction
    digitalWrite(hand_led, HIGH);
  }
  else if ((haptics_iteration_counter) > 13000 && (state) == 1)
  {
    command_fork = (Kp * (angle_hand - angle_fork) + Kd * error);
    digitalWrite(hand_led, HIGH);
    /*
      command_fork =(2.2* (angle_hand - angle_fork) + 0.025 * error); When the bike is loaded gains should be changed to obtain better performance for this reason an additional switch is added at the handlebars
        (higher gains without loading the fork causes oscillatory motion of the fork)
        digitalWrite(hand_led, HIGH); */
  }


  //--------------------------------------------------------------------Calculation of Torque fork motor after alighment--------------------------------------------------------------------------------------------------
  //You have to add a switch and change 1685 more than 2000
  pwm_command_fork = (command_fork * -1685.59 + 32768); // divide with reduction ratio* torque constant to give a command in Amperes
  // Scale torque amp with a constant to obtain a dac number
  pwm_command_fork = clamp_i32(pwm_command_fork, 0, 65536);

  analogWriteResolution(16);
  analogWrite(pwm_pin_fork, abs(pwm_command_fork));


  //--------------------------------------------------------------------Calculation of Torque fork motor from motor current--------------------------------------------------------------------------------------------------
  val_fork = analogRead(a_fork);// read the fork motordrive analog pin
  float a_volt_fork = val_fork * 3.3 / 1024;
  float a_amp_fork = (a_volt_fork * 15 / 3.3); // 15 Amperes,3.3 volts
  //a_torque_fork = -(a_amp_fork * 0.0369 * 36); // 0.0369 = torque constant, 36 reduction ratio
  if (command_fork < 0)
  {
    a_torque_fork = -(a_amp_fork * 0.0369 * 36);
  }
  else if (command_fork > 0 && command_fork < 1)
  {
    a_torque_fork = command_fork - 0.08; /*the circuit can not read negative voltage so to estimate current torque I substract an error from the actual torque of the controller. The error is estimated based on the error read
  from the positive voltage comparison with actual torque*/
  }
  else if (command_fork > 1 && command_fork < 2)
  {
    a_torque_fork = command_fork - 0.16;
  }
  else
  {
    a_torque_fork = command_fork - 0.75;
  }
  // -----------------------------------------------Handlebar feedback tracking controller------------------------------------------------------------------------------------------------------------------------------
  //declare pin alocation on teensie 3.6 board
  const int switch_hand = 29; // making high enables handlebar motor drive
  const int pwm_pin_hand = 8;
  //declare pin as outputs
  
  // define variables
  float Kp1 = 0.9; // Nick recommends 8 Nm/rad=0.139626 Nm/deg
  float Kd1 = 0.012; // Nick recommends 0.6 Nm/rad=0.010472 Nm/deg
  //    float Tc1 = 0.0369; // Torque constant  36.9 mNm/A = 0.0369 Nm/A
  //    const int Tdac1 = 1;

  float command_hand; // Commanded torque for the handlebars
  signed int pwm_command_hand; // Commanded torque converted to PWM
  //-----------------------------------------------Reducing the amount of torque applied to the handlebar for the first 14 seconds allowing the fork to align safely with the handlebars-------------------------------------------

  if (haptics_iteration_counter > 0 && haptics_iteration_counter < 6000) // 1st of reduction
  {
    command_hand = (Kp1 * (angle_hand - angle_fork) + Kd1 * error) / 35;
  }
  else if (haptics_iteration_counter > 6000 && haptics_iteration_counter < 7000)
  {
    command_hand = (Kp1 * (angle_hand - angle_fork) + Kd1 * error) / 30;
  }
  else if (haptics_iteration_counter > 7000 && haptics_iteration_counter < 8000)
  {
    command_hand = (Kp1 * (angle_hand - angle_fork) + Kd1 * error) / 25;
  }
  else if (haptics_iteration_counter > 8000 && haptics_iteration_counter < 9000)
  {
    command_hand = (Kp1 * (angle_hand - angle_fork) + Kd1 * error) / 20;
  }
  else if (haptics_iteration_counter > 9000 && haptics_iteration_counter < 10000)
  {
    command_hand = (Kp1 * (angle_hand - angle_fork) + Kd1 * error) / 15;
  }
  else if (haptics_iteration_counter > 10000 && haptics_iteration_counter < 11000)
  {
    command_hand = (Kp1 * (angle_hand - angle_fork) + Kd1 * error) / 10;
  }
  else if (haptics_iteration_counter > 11000 && haptics_iteration_counter < 12000)
  {
    command_hand = (Kp1 * (angle_hand - angle_fork) + Kd1 * error) / 5;
  }
  else if (haptics_iteration_counter > 12000 && haptics_iteration_counter < 13000)
  {
    command_hand = (Kp1 * (angle_hand - angle_fork) + Kd1 * error) / 2.5;
  }
  else if (haptics_iteration_counter > 13000 && (state) == 0)
  {
    command_hand = Kp1 * (angle_hand - angle_fork) + Kd1 * error; //9 th step of reduction
    digitalWrite(hand_led, HIGH);
  }
  else if ((haptics_iteration_counter) > 13000 && (state) == 1)
  {
    command_hand = 0;
    digitalWrite(hand_led, HIGH);
  }

  //--------------- Calculate the current for the handlebars -----------------//
  // Convert torque to PWM by multiplying with 1685.59 and adding 32768
  pwm_command_hand = (command_hand * -1685.59 + 32768);
  // Clamp the command
  if (pwm_command_hand > 65536){
    pwm_command_hand = 65536;
  }
  if (pwm_command_hand < -65536){
    pwm_command_hand = -65536;
  }
  // Send PWM command
  analogWriteResolution(16);
  analogWrite(pwm_pin_hand, abs(pwm_command_hand));
  
  //------------ Calculate torque from handlebar motor current ---------------// 
  val_hand = analogRead(a_hand);// read the handlebar motordrive analog pin
  float a_volt_hand = val_hand * 3.3 / 1024; //transform bytes to actual volatage
  float a_amp_hand = (a_volt_hand * 15 / 3.3); // 15 Amperes,3.3 volts
  //a_torque_hand = -(a_amp_hand * 0.0369 * 36); // 0.0369 = torque constant, 36 reduction ratio

  if (command_hand < 0)
  {
    a_torque_hand = -(a_amp_hand * 0.0369 * 36);
  }
  else if (command_hand > 0 && command_hand < 1)
  {
    a_torque_hand = command_hand + 0.08; /*the circuit can not read negative voltage so to estimate current torque I substract an error from the actual torque of the controller. The error is estimated based on the error read
   from the positive voltage comparison with actual torque*/
  }
  else if (command_hand > 1 && command_hand < 2)
  {
    a_torque_hand = command_hand + 0.16; //estimated torque seem to overshoot commanded torque basically because to receive feedback we apply a counter-acting torque stalling the motor
  }
  else
  {
    a_torque_hand = command_hand + 0.75;
  }

  //----------------------- Calculate applied torque -------------------------//
  //float Dac_error=0.0039f*sq(angle_hand) + 0.2419f*angle_hand + 293.73f -285.0;
  //Dac_error= abs(Dac_error);
  val_torque = analogRead(a_torque);// read the output analog pin of torque sensor
  float val_torque_dac = val_torque;
  float Torque_zero;
  float Dac_zero;

  //  if (angle_hand>0)
  //  {
  // Dac_zero=0.359*angle_hand + 293.92;
  Dac_zero = 0.0030 * sq(angle_hand) + 0.2683 * angle_hand + 287.59; // calculate dac error based on handlebar angle check excel file for equation
  //Dac_zero=289.29*exp(0.0009*angle_hand);
  Torque_zero = 0.0792f * Dac_zero - 23.196f; // calculate torque error
  analog_torque_sensor_Nm = (0.0792f * val_torque) - 23.196f - Torque_zero - 1.7f; // 1.7 is the offset with all the motors powered.

  //  }
  //else
  //  {
  //  analog_torque_sensor_Nm =(0.0792f*val_torque)-23.196f+1.0f;
  //  }

  //--------------------- Calculate perturbation force -----------------------//
  float val_force_dac;
  val_force = analogRead(a_force);// read the output analog pin of force transducer
  val_force_dac = val_force;
  float analog_force_transducer_N = -(2.3642 * val_force - 972.77) - 75.0f; // 75 is the offset with all the motors powered.

  //----------------------- Calculate rear wheel speed -----------------------//
  int32_t current_wheel_count = wheel_counter.read();
  int32_t previous_wheel_count = wheel_counts[wheel_counts_index];
  wheel_counts[wheel_counts_index] = current_wheel_count;
  wheel_counts_index += 1;
  if (wheel_counts_index >= WHEEL_COUNTS_LENGTH) {
    wheel_counts_index = 0;
  }
  float rps_wheel = ((float) (current_wheel_count - previous_wheel_count)) / 192.0f /* counts/rev */ * 1000.0f /* ms / s */ / ((float) WHEEL_COUNTS_LENGTH);
  float bike_velocity_ms = -rps_wheel * 6.28f * 0.33f / 1000.0f * 3600.0f /* radius in m */ * 0.277778/*km/h to m/s*/;

  //------------------------ Calculate pedal cadence -------------------------//
  int32_t current_pedal_count = pedal_counter.read();
  int32_t previous_pedal_count = pedal_counts[pedal_counts_index];
  pedal_counts[pedal_counts_index] = current_pedal_count;
  pedal_counts_index += 1;
  if (pedal_counts_index >= PEDAL_COUNTS_LENGTH) {
    pedal_counts_index = 0;
  }
  float pedal_cadence_rads = ((float) (current_pedal_count - previous_pedal_count)) / 192.0f /* counts/rev */ * 60.0f /* sec / min */ * 1000.0f /* ms / s */ / ((float) PEDAL_COUNTS_LENGTH) * 0.10471975511970057; /*rpm/ rad/s */

  //---------------------- Get ready for the next loop -----------------------//
  unsigned long end_time = micros();
  next_run_time = next_run_time + 1000; // Each loop takes 1000 microseconds

  //----------------------------- Read IMU data ------------------------------//
  IMU.Read();
  // Change axes to match Whipple-Carvallo model
  accelY = IMU.accel_x_mps2();
  accelX = -IMU.accel_y_mps2();
  accelZ = -IMU.accel_z_mps2();
  gyroY = -IMU.gyro_x_radps();
  gyroX = IMU.gyro_y_radps();
  gyroZ = IMU.gyro_z_radps();
  temp = IMU.die_temp_c();

  //------------------------ Printing to serial port -------------------------//
  // Limit the printing rate
  if (haptics_iteration_counter % 300 == 0) {
    // Angles
    Serial.print("Hand(deg)=");
    Serial.print(angle_hand);
    Serial.print(",Fork(deg)=");
    Serial.print(angle_fork);
    // Torque
    Serial.print(",Torque_handlebar(Nm)=");
    Serial.print(command_hand);
    Serial.print(",Torque_fork(Nm)=");
    Serial.print(command_fork);
    Serial.print(",Torque_fork(dac)=");
    Serial.print(pwm_command_fork);
    Serial.print(",Torque_handlebar(dac)=");
    Serial.print(pwm_command_hand);
    Serial.print(",analog_torque_fork(Nm)=");
    Serial.print(a_torque_fork);
    Serial.print(",analog_torque_handlebar(Nm)=");
    Serial.print(a_torque_hand);
    Serial.print(",DAC_torque_sensor(Nm)=");
    Serial.print(val_torque);
    Serial.print(",a_torque(Nm)=");
    Serial.print(analog_torque_sensor_Nm);
    // Force
    Serial.print(",DAC_force_sensor(N)=");
    Serial.print(val_force_dac);
    Serial.print(",a_force(N)=");
    Serial.print(analog_force_transducer_N);
    // Speed
    Serial.print(",bike_velocity(m/s)=");
    Serial.print(bike_velocity_ms);
    Serial.print(",pedal_cadence(rad/s)=");
    Serial.print(pedal_cadence_rads);
    // IMU
    Serial.print(",AccelX(m/s^2)=");
    Serial.print(accelX);
    Serial.print(",AccelY(m/s^2)=");
    Serial.print(accelY);
    Serial.print(",AccelZ(m/s^2)=");
    Serial.print(accelZ);
    Serial.print(",GyroX(rad/s)=");
    Serial.print(gyroX);
    Serial.print(",GyroY(rad/s)=");
    Serial.print(gyroY);
    Serial.print(",GyroZ(rad/s)=");
    Serial.print(gyroZ);
    Serial.print(",Temperature(°C)=");
    Serial.print(temp);
    // Other
    Serial.print(",haptic counter= ");
    Serial.print(haptics_iteration_counter);
    Serial.print(",Time = ");
    Serial.print(micros());
    Serial.print(",Hand(Counts)= ");
    Serial.print(hand_dac);
    Serial.print(",Fork(Counts)= ");
    Serial.print(fork_dac);
    Serial.println();
  }
  //----------------------------- Create message -----------------------------//
  if (haptics_thread_message_count < HAPTICS_MESSAGE_BUFFER_CAPACITY) {
    Threads::Scope scope(message_buffer_mutex);
    volatile HapticsMessage * message = &haptics_thread_message_buffer[haptics_thread_message_count];
    haptics_thread_message_count += 1;
    message->index = haptics_iteration_counter;
    message->start_time = start_time;
    message->end_time = end_time;
    message->angle_hand_rad = angle_hand_rad;
    message->angle_fork_rad = angle_fork_rad;
    message->torque_fork = command_fork;
    message->torque_handlebar = command_hand;
    message->pedal_cadence = pedal_cadence_rads;
    message->velocity = bike_velocity_ms;
    message->countspedal = current_pedal_count;
    message->countswheel = current_wheel_count;
    message->accelX = accelX;
    message->accelY = accelY;
    message->accelZ = accelZ;
    message->gyroX = gyroX;
    message->gyroY = gyroY;
    message->gyroZ = gyroZ;
    message->temp = temp;
    message->a_torque_fork = a_torque_fork;
    message->a_torque_hand = a_torque_hand;
    message->analog_torque_sensor_Nm = analog_torque_sensor_Nm;
    message->analog_force_transducer_N = analog_force_transducer_N;
  } 
  else {
    Serial.println("DATA LOGGING BUFFER OVERFLOWED");
  }
  //------------------------ Move to the next loop ---------------------------//
  if (end_time >= next_run_time) {
    Serial.println("MISSED HAPTICS DEADLINE");
  }
  yield_until_us(next_run_time);
  haptics_iteration_counter += 1;
}

//============================== Log data to SD ==============================//
void write_thread() {
  randomSeed(analogRead(33));
  long randNumber = random(300);
  String filename = String(randNumber)+".csv";
  
  File sd_file;

  HapticsMessage * logging_thread_message_buffer = haptics_message_buffer_2;
  uint32_t logging_thread_message_count = 0;
  uint32_t logging_thread_message_index = 0;

  while (1) {
    sd_file = SD.open(filename.c_str(), O_WRITE | O_CREAT | O_TRUNC);
    if (sd_file) {
      sd_file.print("write_time_us, ");
      sd_file.print("index, ");
      sd_file.print("start_time, ");
      sd_file.print("end_time, ");
      sd_file.print("angle_hand_rad, ");
      sd_file.print("angle_fork_rad, ");
      sd_file.print("torque_fork_Nm, ");
      sd_file.print("torque_handlebar_Nm, ");
      sd_file.print("pedal_cadence_rads, ");
      sd_file.print("velocity_ms, ");
      sd_file.print("countspedal, ");
      sd_file.print("countswheel, ");
      sd_file.print("accelX_mss, ");
      sd_file.print("accelY_mss, ");
      sd_file.print("accelZ_mss, ");
      sd_file.print("gyroX_rads, ");
      sd_file.print("gyroY_rads, ");
      sd_file.print("gyroZ_rads, ");
      //sd_file.print("temp, ");
      //sd_file.print("a_torque_fork, ");
      //sd_file.print("a_torque_hand, ");
      sd_file.print("analog_torque_sensor_Nm, ");
      sd_file.print("analog_force_transducer_N, ");
      sd_file.println();
      break;
    }
    threads.yield();
  }

  while (1) {
    while (logging_thread_message_index < logging_thread_message_count) {
      if (sd_file) {
        volatile HapticsMessage * message = &logging_thread_message_buffer[logging_thread_message_index];
        logging_thread_message_index += 1;

        sd_file.print(micros());
        sd_file.print(",");
        sd_file.print(message->index);
        sd_file.print(",");
        sd_file.print(message->start_time);
        sd_file.print(",");
        sd_file.print(message->end_time);
        sd_file.print(",");
        sd_file.print(message->angle_hand_rad, 6);
        sd_file.print(",");
        sd_file.print(message->angle_fork_rad, 6);
        sd_file.print(",");
        sd_file.print(message->torque_fork);
        sd_file.print(",");
        sd_file.print(message->torque_handlebar);
        sd_file.print(",");
        sd_file.print(message->pedal_cadence);
        sd_file.print(",");
        sd_file.print(message->velocity);
        sd_file.print(",");
        sd_file.print(message->countspedal);
        sd_file.print(",");
        sd_file.print(message->countswheel);
        sd_file.print(",");
        sd_file.print(message->accelX);
        sd_file.print(",");
        sd_file.print(message->accelY);
        sd_file.print(",");
        sd_file.print(message->accelZ);
        sd_file.print(",");
        sd_file.print(message->gyroX);
        sd_file.print(",");
        sd_file.print(message->gyroY, 6);
        sd_file.print(",");
        sd_file.print(message->gyroZ, 6);
        sd_file.print(",");
        //sd_file.print(message->temp);
        //sd_file.print(",");
        //sd_file.print(message->a_torque_fork);
        //sd_file.print(",");
        //sd_file.print(message->a_torque_hand);
        //sd_file.print(",");
        sd_file.print(message->analog_torque_sensor_Nm);
        sd_file.print(",");
        sd_file.print(message->analog_force_transducer_N);
        sd_file.print(",");
        sd_file.println();
      }
    }

    if (sd_file) {
      sd_file.flush();
    }

    {
      Threads::Scope scope(message_buffer_mutex);

      // Swap buffers.
      if (haptics_thread_message_buffer == haptics_message_buffer_1) {
        haptics_thread_message_buffer = haptics_message_buffer_2;
        logging_thread_message_buffer = haptics_message_buffer_1;
      } else {
        haptics_thread_message_buffer = haptics_message_buffer_1;
        logging_thread_message_buffer = haptics_message_buffer_2;
      }

      // Update counts.
      logging_thread_message_count = haptics_thread_message_count;
      haptics_thread_message_count = 0;
      logging_thread_message_index = 0;
    }
  }
}

//================ Functions for debugging and for threading =================//
void block_for_us(unsigned long delta_time_us) {
  block_until_us(micros() + delta_time_us);
}

void block_until_us(unsigned long continue_time_us) {
  while (micros() < continue_time_us) {
    // Do nothing but check the condition.
  }
}

void yield_for_us(unsigned long delta_time_us) {
  yield_until_us(micros() + delta_time_us);
}

void yield_until_us(unsigned long continue_time_us) {
  while (micros() < continue_time_us) {
    threads.yield();
  }
}

float clamp_f32(float value, float minv, float maxv) {
  if (value <= minv) {
    return minv;
  } 
  else if (value >= maxv) {
    return maxv;
  }
  return value;
}

int clamp_i32(int value, int minv, int maxv) {
  if (value <= minv) {
    return minv;
  } 
  else if (value >= maxv) {
    return maxv;
  }
  return value;
}