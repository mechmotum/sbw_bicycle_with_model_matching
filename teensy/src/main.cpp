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
const int ssimu = 10; // slave selection enable is Low
const int sshand = 24; // pin of logic gate setting LOW is active
const int ssfork = 25;
const int pwrencoders = 26; // making High enable power to encoders
const int led = 27; //led powers up for 30 seconds to show that self alighment is completed
const int gain_switch = 28; // pin connected to handlebar switch when High fork controller settings are changed
const int analog_fork_motor = 33; // analog output of fork motor drive
float val_fork = 0;
const int analog_handlebar_motor = 34; // analog output of fork motor drive
float val_handlebar = 0;
const int analog_torque_sensor = 21; // analog output of torque sensor
float val_torque_sensor = 0;
const int analog_force_transducer = 20; // analog output of force tranducer
float val_force_transducer = 0;
Encoder wheel_counter(2, 3); //declaring wheel encoder pins for encoder library
Encoder pedal_counter(23, 22); //declaring pedal encoder pins for encoder library
bfs::Mpu9250 IMU(&SPI, 10); // an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10

//============================= Global variables =============================//
//wheel encoder variables
const uint32_t WHEEL_COUNTS_LENGTH = 200;
int32_t wheel_counts[WHEEL_COUNTS_LENGTH] = {0};
uint32_t wheel_counts_index = 0;

//pedal encoder variables
const uint32_t PEDAL_COUNTS_LENGTH = 500;
int32_t pedal_counts[PEDAL_COUNTS_LENGTH] = {0};
uint32_t pedal_counts_index = 0;

long current_time;
unsigned long next_run_time;
float errorlast = 0;
float errorTimeLast = 0;
unsigned int haptics_iteration_counter = 0;
struct HapticsMessage {
  uint32_t index;
  uint32_t start_time;
  uint32_t end_time;
  float handlebar_angle_rad;
  float fork_angle_rad;
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
  float analog_torque_fork_Nm;
  float analog_torque_handlebar_Nm;
  float analog_torque_sensor_Nm;
  float analog_force_transducer_N;
  float Torque_zero;
  float Dac_zero;
};

unsigned long time_start_program;
unsigned long current_time_program;

float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float temp;
float analog_torque_fork_Nm;
float analog_torque_handlebar_Nm;
float analog_torque_sensor_Nm;
float analog_force_transducer_N;

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

  if (!threads.setMicroTimer(100)) { //if it is not able to set it will print the statement below
    Serial.println("FAILED TO SET MICRO TIMER");
  }
  threads.setDefaultTimeSlice(1);
  threads.addThread(write_thread);

  digitalWrite(ssfork, HIGH); // setting HIGH slave selection pin to disable IMU
  digitalWrite(sshand, HIGH); // setting LOW slave selection pin to enable handlebar encoder
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
  // set the slaveSelectpins and power pin of absolute encoders and IMU as outputs:
  pinMode (ssimu, OUTPUT);
  pinMode (sshand, OUTPUT);
  pinMode (ssfork, OUTPUT);
  pinMode (pwrencoders, OUTPUT);
  pinMode (led, OUTPUT);
  pinMode (gain_switch, INPUT); // making handlebar switch high changes fork controller gains
  {
    Threads::Scope scope(message_buffer_mutex);
    haptics_thread_message_buffer = haptics_message_buffer_1;
    haptics_thread_message_count = 0;
  }

  // Only switch every n timeslices.
  threads.setTimeSlice(threads.id(), 10);
}

//================================ Haptic Loop ===============================//
void haptics_loop() {
  unsigned long start_time = micros();
  unsigned long state = digitalRead(gain_switch); // read HIGH and LOW state of handlebat toggle switch
  // enable slave devices
  digitalWrite(ssimu, HIGH); // setting HIGH slave selection pin to disable IMU
  digitalWrite(pwrencoders, HIGH); // setting HIGH PA10 pin of external pcb connected to pin 26 of teensie to enable power to my encoders
  digitalWrite(ssfork, HIGH); // setting HIGH slave selection pin to disable fork encoder

  // set clock signal characteristics
  SPI.beginTransaction(SPISettings(225000, MSBFIRST, SPI_MODE3));  // set frequency to 125 Khz-4 Mhz, encoder transmit first the MSB, Clock Idles High Latch on the initial clock edge, sample on the subsequent edge
  digitalWrite(sshand, LOW); // setting LOW slave selection pin to enable handlebar encoder LOW

  uint16_t handlebar_dac = SPI.transfer16(0); // transfering 16 bits to MOSI and reading what comes back to MISO

  //From my byte1 you need to mask out 3 bits the first MSB and the two LSB;
  // First AND bit operator is used bit mask 0111 1111 1111 1111 the 16bit is set to zero.
  handlebar_dac = (handlebar_dac & 0x7fff) >> 2;

  digitalWrite(sshand, HIGH); // disable handlebar encoder
  SPI.endTransaction(); // ending transaction

  //---------------------------------------------------Reading data from fork encoder----------------------------------------------------------------------------------------
  SPI.beginTransaction(SPISettings(225000, MSBFIRST, SPI_MODE3));
  digitalWrite(ssfork, LOW); // enable fork encoder LOW

  uint16_t fork_dac = SPI.transfer16(0); // transfering 16 bits to MOSI and reading what comes back to MISO
  fork_dac = (fork_dac & 0x7fff) >> 2;

  digitalWrite(ssfork, HIGH); // disable fork encoder
  SPI.endTransaction(); // ending transaction

  // -----------------------------------------------Processing data output of encoders------------------------------------------------------------------------

  //Note: The two encoders are mounted oppose to each other for this reason we have different counts directions.
  // More specific, Anticlockwise rotation of handlebar encoder gives 360 deg-> 310 deg whereas, anticlockwise rotation gives 0-55 degrees.
  // The rotational direction must be the same for both encoders, the encoders must give an output 0+- 180 degrees. Two if statements are used for these reasons.

  float handlebarangle = (float)handlebar_dac * 360.0f / 8191.0f - 153.65; // encoder counts to deg 152.20 is the offset (handangle-forkangle)

  if (handlebarangle > 180.0f) // Anticlockwise rotation of handlebar encoder gives 360 deg-> 310 deg.
    handlebarangle = handlebarangle - 360.0f; //substracting -360 we get 0-180 deg anticlockwise.

  // ---------------------------------------------Set software limits------------------------------------------------------------------------------------------

  // The fork mechanical range is +- 42 degrees since handlebars do bot have a mechanical limit software limits are set below if you do not set this limits the motor foldsback.
  handlebarangle = clamp_f32(handlebarangle, -45.0f, 45.0f);
  float handlebar_angle_rad = handlebarangle * 0.0174533;
  // end of software limits

  float forkangle = -(float)fork_dac * 360.0 / 8191.0 - 100.65; ////CALIBRATE encoder counts to deg, substracting -99.2 deg to align fork with handlebars and - to get minus values from fork encoder.
  //Attention: make sure that the fork encoder head is aligned with actuation magnet if not it will give an offset the handlebars will not follow the exact trajectory of the fork
  if (forkangle < -180.0f) // Clockwise rotation of fork encoder gives -360 deg-> -310 deg
    forkangle = forkangle + 360.0f; //by adding +360 we get 0-180 deg anticlockwise

  float fork_angle_rad = forkangle * 0.0174533;
  // -----------------------------------------------Fork feedback tracking controller------------------------------------------------------------------------

  //declare pin alocation on teensie 3.6 board
  const int forkmotordrive = 30; // making High enables fork motor drive
  const int pwmforkmotor = 9;
  //declare pin as outputs
  pinMode (forkmotordrive, OUTPUT);
  pinMode (pwmforkmotor, OUTPUT);
  // define variables
  float Kp = 2; // Nick recommends 8 Nm/rad=0.139626 Nm/deg 0.61157
  float Kd = 0.029;  // Nick recommends 0.6 Nm/rad=0.010472 Nm/deg
  //    float Tc = 0.0369; // Torque constant  36.9 mNm/A = 0.0369 Nm/A
  //    const int Tdac = 1;
  float Torque_fork_Nm;


  float Torqued;
  signed int Torquedac;

  //--------------------------------------------------------------------Calculation of derivative part----------------------------------------------------------------------------------------------------------------------

  if (haptics_iteration_counter < 10) { //setting handlebar and fork encoder first values to 0, we do that because first values seem to be floating values
    handlebarangle = 0.0f;
    forkangle = 0.0f;
  }

  unsigned long errorTimeNew = micros();//calculation of dt
  float errorTime = ((float) (errorTimeNew - errorTimeLast)) / 1000000.0f; /*get time in s, if you put your time in ms you get very small magnitudes error which has as an impact high Kd gain to have high damping torques.
                                                                        It also makes sense to have the error in rad/s since these parameters will be used afterwards to extend the whipple bicycle model, which has its parameters set in sec*/
  errorTimeLast = errorTimeNew;

  float errornew = (handlebarangle - forkangle); //calculation of derror
  float error = (errornew - errorlast) / errorTime;
  errorlast = errornew;
  //-----------------------------------------------reducing the amount of torque applied to the forkmotor for the first 14 seconds allowing the fork to align safely with the handlebars-------------------------------------------

  if (haptics_iteration_counter > 0 && haptics_iteration_counter < 6000) // 1st of reduction
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error) / 35;
  }
  else if (haptics_iteration_counter > 6000 && haptics_iteration_counter < 7000)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error) / 30;
  }
  else if (haptics_iteration_counter > 7000 && haptics_iteration_counter < 8000)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error) / 25;
  }
  else if (haptics_iteration_counter > 8000 && haptics_iteration_counter < 9000)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error) / 20;
  }
  else if (haptics_iteration_counter > 9000 && haptics_iteration_counter < 10000)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error) / 15;
  }
  else if (haptics_iteration_counter > 10000 && haptics_iteration_counter < 11000)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error) / 10;
  }
  else if (haptics_iteration_counter > 11000 && haptics_iteration_counter < 12000)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error) / 5;
  }
  else if (haptics_iteration_counter > 12000 && haptics_iteration_counter < 13000)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error) / 2.5;
  }
  else if ((haptics_iteration_counter) > 13000 && (state) == 0)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error); //9 th step of reduction
    digitalWrite(led, HIGH);
  }
  else if ((haptics_iteration_counter) > 13000 && (state) == 1)
  {
    Torque_fork_Nm = (Kp * (handlebarangle - forkangle) + Kd * error);
    digitalWrite(led, HIGH);
    /*
      Torque_fork_Nm =(2.2* (handlebarangle - forkangle) + 0.025 * error); When the bike is loaded gains should be changed to obtain better performance for this reason an additional switch is added at the handlebars
        (higher gains without loading the fork causes oscillatory motion of the fork)
        digitalWrite(led, HIGH); */
  }


  //--------------------------------------------------------------------Calculation of Torque fork motor after alighment--------------------------------------------------------------------------------------------------
  Torqued = +Kd * error;
  //You have to add a switch and change 1685 more than 2000
  Torquedac = (Torque_fork_Nm * -1685.59 + 32768); // divide with reduction ratio* torque constant to give a command in Amperes
  // Scale torque amp with a constant to obtain a dac number
  Torquedac = clamp_i32(Torquedac, 0, 65536);

  analogWriteResolution(16);
  analogWrite(pwmforkmotor, abs(Torquedac));
  digitalWrite(forkmotordrive,HIGH); // NOTE: DISABLED MOTOR KLEINWPSALIDI


  //--------------------------------------------------------------------Calculation of Torque fork motor from motor current--------------------------------------------------------------------------------------------------
  val_fork = analogRead(analog_fork_motor);// read the fork motordrive analog pin
  float analog_voltage_fork_motor = val_fork * 3.3 / 1024;
  float analog_current_fork_motor = (analog_voltage_fork_motor * 15 / 3.3); // 15 Amperes,3.3 volts
  //analog_torque_fork_Nm = -(analog_current_fork_motor * 0.0369 * 36); // 0.0369 = torque constant, 36 reduction ratio
  if (Torque_fork_Nm < 0)
  {
    analog_torque_fork_Nm = -(analog_current_fork_motor * 0.0369 * 36);
  }
  else if (Torque_fork_Nm > 0 && Torque_fork_Nm < 1)
  {
    analog_torque_fork_Nm = Torque_fork_Nm - 0.08; /*the circuit can not read negative voltage so to estimate current torque I substract an error from the actual torque of the controller. The error is estimated based on the error read
  from the positive voltage comparison with actual torque*/
  }
  else if (Torque_fork_Nm > 1 && Torque_fork_Nm < 2)
  {
    analog_torque_fork_Nm = Torque_fork_Nm - 0.16;
  }
  else
  {
    analog_torque_fork_Nm = Torque_fork_Nm - 0.75;
  }
  // -----------------------------------------------Handlebar feedback tracking controller------------------------------------------------------------------------------------------------------------------------------
  //declare pin alocation on teensie 3.6 board
  const int handmotordrive = 29; // making high enables handlebar motor drive
  const int pwmhandmotor = 8;
  //declare pin as outputs
  pinMode (handmotordrive, OUTPUT);
  pinMode (pwmhandmotor, OUTPUT);
  // define variables
  float Kp1 = 0.9; // Nick recommends 8 Nm/rad=0.139626 Nm/deg
  float Kd1 = 0.012; // Nick recommends 0.6 Nm/rad=0.010472 Nm/deg
  //    float Tc1 = 0.0369; // Torque constant  36.9 mNm/A = 0.0369 Nm/A
  //    const int Tdac1 = 1;

  float Torque_handlebar_Nm;
  signed int Torquedac1;
  //-----------------------------------------------Reducing the amount of torque applied to the handlebar for the first 14 seconds allowing the fork to align safely with the handlebars-------------------------------------------

  if (haptics_iteration_counter > 0 && haptics_iteration_counter < 6000) // 1st of reduction
  {
    Torque_handlebar_Nm = (Kp1 * (handlebarangle - forkangle) + Kd1 * error) / 35;
  }
  else if (haptics_iteration_counter > 6000 && haptics_iteration_counter < 7000)
  {
    Torque_handlebar_Nm = (Kp1 * (handlebarangle - forkangle) + Kd1 * error) / 30;
  }
  else if (haptics_iteration_counter > 7000 && haptics_iteration_counter < 8000)
  {
    Torque_handlebar_Nm = (Kp1 * (handlebarangle - forkangle) + Kd1 * error) / 25;
  }
  else if (haptics_iteration_counter > 8000 && haptics_iteration_counter < 9000)
  {
    Torque_handlebar_Nm = (Kp1 * (handlebarangle - forkangle) + Kd1 * error) / 20;
  }
  else if (haptics_iteration_counter > 9000 && haptics_iteration_counter < 10000)
  {
    Torque_handlebar_Nm = (Kp1 * (handlebarangle - forkangle) + Kd1 * error) / 15;
  }
  else if (haptics_iteration_counter > 10000 && haptics_iteration_counter < 11000)
  {
    Torque_handlebar_Nm = (Kp1 * (handlebarangle - forkangle) + Kd1 * error) / 10;
  }
  else if (haptics_iteration_counter > 11000 && haptics_iteration_counter < 12000)
  {
    Torque_handlebar_Nm = (Kp1 * (handlebarangle - forkangle) + Kd1 * error) / 5;
  }
  else if (haptics_iteration_counter > 12000 && haptics_iteration_counter < 13000)
  {
    Torque_handlebar_Nm = (Kp1 * (handlebarangle - forkangle) + Kd1 * error) / 2.5;
  }
  else if (haptics_iteration_counter > 13000 && (state) == 0)
  {
    Torque_handlebar_Nm = Kp1 * (handlebarangle - forkangle) + Kd1 * error; //9 th step of reduction
    digitalWrite(led, HIGH);
  }
  else if ((haptics_iteration_counter) > 13000 && (state) == 1)
  {
    Torque_handlebar_Nm = 0;
    digitalWrite(led, HIGH);
  }


  //--------------------------------------------------------------------calculation of Torque handlebar motor--------------------------------------------------------

  Torquedac1 = (Torque_handlebar_Nm * -1685.59 + 32768); // divide with reduction ratio* torque constant to give a command in Amperes 1685.59
  // Scale torque amp with a constant to obtain a dac number
  if (Torquedac1 > 65536) // Clockwise rotation of fork encoder gives -360 deg-> -310 deg
    Torquedac1 = 65536; //by adding +360 we get 0-180 deg anticlockwise
  if (Torquedac1 < -65536) // Clockwise rotation of fork encoder gives -360 deg-> -310 deg
    Torquedac1 = -65536; //by adding +360 we get 0-180 deg anticlockwise
  analogWriteResolution(16);
  analogWrite(pwmhandmotor, abs(Torquedac1));
  digitalWrite(handmotordrive, HIGH); // NOTE: DISABLED HANDLEBAR MOTOR KLEINWXEROULIA
  //--------------------------------------------------------------------Calculation of Torque handlebar motor from motor current--------------------------------------------------------------------------------------------------
  val_handlebar = analogRead(analog_handlebar_motor);// read the handlebar motordrive analog pin
  float analog_voltage_handlebar_motor = val_handlebar * 3.3 / 1024; //transform bytes to actual volatage
  float analog_current_handlebar_motor = (analog_voltage_handlebar_motor * 15 / 3.3); // 15 Amperes,3.3 volts
  //analog_torque_handlebar_Nm = -(analog_current_handlebar_motor * 0.0369 * 36); // 0.0369 = torque constant, 36 reduction ratio

  if (Torque_handlebar_Nm < 0)
  {
    analog_torque_handlebar_Nm = -(analog_current_handlebar_motor * 0.0369 * 36);
  }
  else if (Torque_handlebar_Nm > 0 && Torque_handlebar_Nm < 1)
  {
    analog_torque_handlebar_Nm = Torque_handlebar_Nm + 0.08; /*the circuit can not read negative voltage so to estimate current torque I substract an error from the actual torque of the controller. The error is estimated based on the error read
   from the positive voltage comparison with actual torque*/
  }
  else if (Torque_handlebar_Nm > 1 && Torque_handlebar_Nm < 2)
  {
    analog_torque_handlebar_Nm = Torque_handlebar_Nm + 0.16; //estimated torque seem to overshoot commanded torque basically because to receive feedback we apply a counter-acting torque stalling the motor
  }
  else
  {
    analog_torque_handlebar_Nm = Torque_handlebar_Nm + 0.75;
  }
  //--------------------------------------------------------------------Calculation of applied torque from torque sensor---------------------------------------------------------------------------------------------------------------


  //float Dac_error=0.0039f*sq(handlebarangle) + 0.2419f*handlebarangle + 293.73f -285.0;
  //Dac_error= abs(Dac_error);
  val_torque_sensor = analogRead(analog_torque_sensor);// read the output analog pin of torque sensor
  float val_torque_dac = val_torque_sensor;
  float Torque_zero;
  float Dac_zero;

  //  if (handlebarangle>0)
  //  {
  // Dac_zero=0.359*handlebarangle + 293.92;
  Dac_zero = 0.0030 * sq(handlebarangle) + 0.2683 * handlebarangle + 287.59; // calculate dac error based on handlebar angle check excel file for equation
  //Dac_zero=289.29*exp(0.0009*handlebarangle);
  Torque_zero = 0.0792f * Dac_zero - 23.196f; // calculate torque error
  analog_torque_sensor_Nm = (0.0792f * val_torque_sensor) - 23.196f - Torque_zero - 1.7f; // 1.7 is the offset with all the motors powered.

  //  }
  //else
  //  {
  //  analog_torque_sensor_Nm =(0.0792f*val_torque_sensor)-23.196f+1.0f;
  //  }

  //--------------------------------------------------------------------Calculation of applied translational force from force transducer---------------------------------------------------------------------------------------------------------------
  float val_force_dac;
  val_force_transducer = analogRead(analog_force_transducer);// read the output analog pin of force transducer
  val_force_dac = val_force_transducer;
  float analog_force_transducer_N = -(2.3642 * val_force_transducer - 972.77) - 75.0f; // 75 is the offset with all the motors powered.

  //--------------------------------------------------------------------Calculation of rear wheel speed------------------------------------------------------------------------------------------------------------------------------
  int32_t current_wheel_count = wheel_counter.read();
  int32_t previous_wheel_count = wheel_counts[wheel_counts_index];
  wheel_counts[wheel_counts_index] = current_wheel_count;
  wheel_counts_index += 1;
  if (wheel_counts_index >= WHEEL_COUNTS_LENGTH) {
    wheel_counts_index = 0;
  }
  float rps_wheel = ((float) (current_wheel_count - previous_wheel_count)) / 192.0f /* counts/rev */ * 1000.0f /* ms / s */ / ((float) WHEEL_COUNTS_LENGTH);
  float bike_velocity_ms = -rps_wheel * 6.28f * 0.33f / 1000.0f * 3600.0f /* radius in m */ * 0.277778/*km/h to m/s*/;

  //--------------------------------------------------------------------Calculation of pedal rpm --------------------------------------------------------
  int32_t current_pedal_count = pedal_counter.read();
  int32_t previous_pedal_count = pedal_counts[pedal_counts_index];
  pedal_counts[pedal_counts_index] = current_pedal_count;
  pedal_counts_index += 1;
  if (pedal_counts_index >= PEDAL_COUNTS_LENGTH) {
    pedal_counts_index = 0;
  }
  float pedal_cadence_rads = ((float) (current_pedal_count - previous_pedal_count)) / 192.0f /* counts/rev */ * 60.0f /* sec / min */ * 1000.0f /* ms / s */ / ((float) PEDAL_COUNTS_LENGTH) * 0.10471975511970057; /*rpm/ rad/s */

  unsigned long end_time = micros();



  next_run_time = next_run_time + 1000;// this must be set to 500 however it is not possible to print if you set the sampling frequency too high

  // -----------------------------------------------Reading Imu------------------------------------------------------------------------

  IMU.Read();
  // display the data

  accelY = IMU.accel_x_mps2(); //change the axis to match the orientation according to the Whipple Carvallo Model.
  accelX = -IMU.accel_y_mps2();//change the axis and direction to match the orientation according to the Whipple Carvallo Model.
  accelZ = -IMU.accel_z_mps2();//change the axis and direction to match the orientation according to the Whipple Carvallo Model.
  gyroY = -IMU.gyro_x_radps();//change the axis and direction to match the orientation according to the Whipple Carvallo Model.
  gyroX = IMU.gyro_y_radps();//change the axis to match the orientation according to the Whipple Carvallo Model.
  gyroZ = IMU.gyro_z_radps();////change the axis to match the orientation according to the Whipple Carvallo Model.
  temp = IMU.die_temp_c();

  // -----------------------------------------------Printing to serial port------------------------------------------------------------------------
  // Limit the printing rate.
  if (haptics_iteration_counter % 300 == 0) {

    //----------------------------------------Serial print on the phone through the bluetooth module
    
    Serial1.println(bike_velocity_ms);
    

    // -----------------------------------------------Printing to serial port------------------------------------------------------------------------
//    Serial.print("DAC_force_sensor(Nm)=");
//    Serial.print(val_force_dac);
//    Serial.print(",analog_force_transducer(N)=");
//    Serial.print(analog_force_transducer_N);
//    Serial.print(",DAC_torque_sensor(Nm)=");
//    Serial.print(val_torque_sensor);
//    Serial.print("analog_torque_sensor(Nm)=");
//    Serial.print(analog_torque_sensor_Nm);
    Serial.print(",Hand(deg)=");
    Serial.print(handlebarangle);
    Serial.print(",Fork(deg)=");
    Serial.print(forkangle);
    Serial.print(",Torque_handlebar(Nm)=");
    Serial.print(Torque_handlebar_Nm);
    Serial.print(",Torque_fork(Nm)=");
    Serial.print(Torque_fork_Nm);
    Serial.print(",Torque_fork(dac)=");
    Serial.print(Torquedac);
    Serial.print(",Torque_handlebar(dac)=");
    Serial.print(Torquedac1);
    Serial.print(",bike_velocity(m/s)=");
    Serial.print(bike_velocity_ms);

    Serial.print(",pedal_cadence(rad/s)=");
    Serial.print(pedal_cadence_rads);
    Serial.print(",analog_torque_fork(Nm)=");
    Serial.print(analog_torque_fork_Nm);
    Serial.print(",analog_torque_handlebar(Nm)=");
    Serial.print(analog_torque_handlebar_Nm);
    Serial.print(",haptic counter= ");
    Serial.print(haptics_iteration_counter);
    Serial.print(",Time = ");
    Serial.print(micros());
    //Serial.print(",Hand(Counts)= ");
    //Serial.print(handlebar_dac);
    //Serial.print(",Fork(Counts)= ");
    //Serial.print(fork_dac);
    //Serial.print(",AccelX(m/s^2)=");
    //Serial.print(accelX);//this is the longitudinal axis of the bicycle
    Serial.print(",AccelY(m/s^2)=");
    Serial.print(accelY);//this is the lateral axis of the bicycle
    //Serial.print(",AccelZ(m/s^2)=");
    //Serial.print(accelZ);//this is the vertical acceleration axis
    Serial.print(",GyroX(rad/s)=");
    Serial.print(gyroX);//this is the roll axis of the bicycle
    //Serial.print(",GyroY(rad/s)=");
    //Serial.print(gyroY);//this is the pitch axis of the bicycle
    //Serial.print(",GyroZ(rad/s)=");
    //Serial.print(gyroZ);//this is the yaw axis of the bicycle
    //Serial.print(",Temperature(°C)=");
    //Serial.print(temp);
    Serial.println();
  }

  if (haptics_thread_message_count < HAPTICS_MESSAGE_BUFFER_CAPACITY) {
    Threads::Scope scope(message_buffer_mutex);
    volatile HapticsMessage * message = &haptics_thread_message_buffer[haptics_thread_message_count];
    haptics_thread_message_count += 1;
    message->index = haptics_iteration_counter;
    message->start_time = start_time;
    message->end_time = end_time;
    message->handlebar_angle_rad = handlebar_angle_rad;
    message->fork_angle_rad = fork_angle_rad;
    message->torque_fork = Torque_fork_Nm;
    message->torque_handlebar = Torque_handlebar_Nm;
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
    //message->temp = temp;
    //message->analog_torque_fork_Nm = analog_torque_fork_Nm;
    //message->analog_torque_handlebar_Nm = analog_torque_handlebar_Nm;
    message->analog_torque_sensor_Nm = analog_torque_sensor_Nm;
    message->analog_force_transducer_N = analog_force_transducer_N;


  } else {
    Serial.println("DATA LOGGING BUFFER OVERFLOWED");
  }

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
      sd_file.print("handlebar_angle_rad, ");
      sd_file.print("fork_angle_rad, ");
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
      //sd_file.print("analog_torque_fork_Nm, ");
      //sd_file.print("analog_torque_handlebar_Nm, ");
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
        sd_file.print(message->handlebar_angle_rad, 6);
        sd_file.print(",");
        sd_file.print(message->fork_angle_rad, 6);
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
        //sd_file.print(message->analog_torque_fork_Nm);
        //sd_file.print(",");
        //sd_file.print(message->analog_torque_handlebar_Nm);
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