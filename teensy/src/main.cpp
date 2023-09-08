#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include "SdFat.h"
#include "RingBuf.h" //From sdFat library
#include "mpu9250.h" //https://github.com/bolderflight/MPU9250

/*TODO: include other measurement functions into the BikeMeasurements class, like the IMU*/


/* LEFT FOR DOCUMENTATION PURPOSE ONLY [transfer to more appropriate location and remove]
a_force = 20; // Analog output pin of the force transducer
a_torque = 21; // Analog output pin of the torque sensor
a_fork = 40; // Analog output pin of the fork motor drive
a_hand = 41; // Analog output pin of the handlebar motor drive
*/

//============================== Compile modes ===============================//
#define USE_IMU 1
#define USE_SD 0
#define USE_BIKE_ENCODERS 0
#define SERIAL_DEBUG 0

//================================= Classes ==================================//
class BikeMeasurements{
  private:
    // state and input variables
    float m_hand_angle;
    float m_fork_angle;
    float m_lean_angle;
    float m_fork_rate;
    float m_lean_rate;
    float m_hand_torque; // Measurement of the torque on the handlebar applied by the human 
    
    // variables needed for derivarion/intergration calculation
    uint32_t m_dt_steer_meas; //Time between two consecutive measurements of the steer angle in microseconds
    uint32_t m_dt_IMU_meas; //Time between two consecutive measurements of the IMU values in microseconds
    float m_fork_angle_prev;
    
  public:
    BikeMeasurements(){
      m_hand_angle = 0;
      m_fork_angle = 0;
      m_lean_angle = 0;
      m_fork_rate = 0;
      m_lean_rate = 0;
      m_hand_torque = 0;

      m_dt_steer_meas = 0;
      m_fork_angle_prev = 0;
    }

    // getters
    float get_hand_angle(){return m_hand_angle;}
    float get_fork_angle(){return m_fork_angle;}
    float get_lean_angle(){return m_lean_angle;}
    float get_fork_rate(){return m_fork_rate;}
    float get_lean_rate(){return m_lean_rate;}
    float get_hand_torque(){return m_hand_torque;}
    uint32_t get_dt_steer_meas(){return m_dt_steer_meas;}

    // setters
    void set_hand_angle(float angle){m_hand_angle = angle;}
    void set_fork_angle(float angle){m_fork_angle = angle;}

    // Retreive measurements
    void measure_steer_angles();
    void measure_hand_torque();
    void calculate_fork_rate();
    void calculate_roll_states();
};

//=========================== Function declarations ===========================//
void calc_pd_errors(BikeMeasurements& bike, float& error, float& derror_dt);
void calc_pd_control(float error, float derror_dt, double& command_fork, double& command_hand);
void calc_mm_control(BikeMeasurements& bike, double& command_fork);
void actuate_steer_motors(double command_fork, double command_hand);
uint16_t read_motor_encoder(const uint8_t cs_pin);
void update_dtime(uint32_t& dtime, elapsedMicros& timer);
float riemann_integrate(float value, uint32_t dt);
float calc_bckwrd_derivative(float val_cur, float& val_prev, uint32_t dt);
float return_scaling(uint64_t iteration);
uint8_t check_switch(uint8_t curr_value, uint8_t *val_array, uint8_t array_size);
// void print_to_serial();
float steer_moving_avg(float new_value);
#if USE_BIKE_ENCODERS
float calc_bike_speed();
float calc_cadance();
#endif
#if USE_IMU
void get_IMU_data();
#endif
#if USE_SD
void open_file();
void print_to_SD();
#endif

//============================= Global Variables =============================//
//-------------------------------- Constants ---------------------------------//
// PWM
const uint8_t PWM_RESOLUTION = 15;
const float PWM_FREQUENCY = 4577.64;
const uint32_t PWM_MAX_VAL = (1 << PWM_RESOLUTION) - 1;
const uint32_t HAND_PWM_MAX = PWM_MAX_VAL;
const uint32_t HAND_PWM_MIN = 0;
const uint32_t FORK_PWM_MAX = PWM_MAX_VAL;
const uint32_t FORK_PWM_MIN = 0;
const uint16_t INITIAL_FORK_PWM = 16384; //K: I think that the middle is a zero command. That 0 and 32,768 are both maximum torque but in opposite directions.
const uint16_t INITIAL_STEER_PWM = 16384;

// Timing
const uint16_t MIN_LOOP_LENGTH_MU = 1000; // target minimum loop length in microseconds.
const float MIN_LOOP_LENGTH_S = MIN_LOOP_LENGTH_MU*1e-6; //
const uint16_t CTRL_STARTUP_ITTERATIONS = 13000; //#itterations in which the steer torques are slowly scaled to unity.

// Motor encoders
const uint32_t ENCODER_CLK_FREQ = 225000; //clock frequency for the encoders SPI protocol
const float HAND_ENC_BIAS = 153.65;
const float FORK_ENC_BIAS = 100.65;
const float HAND_ENC_MAX_VAL = 8191.0;
const float FORK_ENC_MAX_VAL = 8191.0;

// Pedal and wheel encoders
const uint16_t WHEEL_COUNTS_LENGTH = 200;
const uint16_t PEDAL_COUNTS_LENGTH = 500;
const float WHEEL_RADIUS = 0.33f;
const uint8_t WHEEL_COUNTS_PER_REV = 192;
const uint8_t PEDAL_COUNTS_PER_REV = 192;

// Angles
const float FULL_ROTATION_DEG = 360.0;
const float HALF_ROTATION_DEG = 180.0;
const float SOFTWARE_LIMIT = 42.0;

// SD card
const uint16_t SD_SAMPLING_FREQ = 1000; //Sampling frequency of the SD card
const uint64_t MESSAGE_LENGTH = 100; // The maximum expected legth of one sample of data. In bytes
const uint16_t EXPERIMENT_TIME = 600; // In seconds
const uint8_t  BUFFER_HOLD_TIME = 2; // In seconds
const uint64_t LOG_FILE_SIZE = MESSAGE_LENGTH*EXPERIMENT_TIME*SD_SAMPLING_FREQ; // Log file should hold 10 minutes of data sampled at 1kHz
const uint64_t RING_BUF_CAPACITY = MESSAGE_LENGTH*BUFFER_HOLD_TIME*SD_SAMPLING_FREQ; // Buffer should hold 2 seconds of data sampled at 1kHz

// PD Gains
const float KP_F = 2.0f; // Fork
const float KD_F = 0.029f; // Fork
const float KP_H = 0.9f; // Handlebar
const float KD_H = 0.012f; // Handlebar

// Model matching gains
const float K_MM1 = 0; // lean angle
const float K_MM2 = 0; // steer/fork angle
const float K_MM3 = 0; // lean rate
const float K_MM4 = 0; // steer/fork rate
const float K_MM5 = 0; // lean torque
const float K_MM6 = 0; // steer/hand torque

//----------------------- Steering rate calculation --------------------------//
const uint8_t STEER_MVING_AVG_SMPL_LEN = 10;

//-------------------------------- Pins --------------------------------------//
const uint8_t cs_hand = 24; // SPI Chip Select for handlebar encoder
const uint8_t cs_fork = 25; // SPI Chip Select for fork encoder
#if USE_IMU
const uint8_t cs_imu = 10; // SPI Chip Select for MPU9250
#endif

const uint8_t pwm_pin_hand = 8; // Send PWM signals to the handlebar motor
const uint8_t pwm_pin_fork = 9; // Send PWM signals to the fork motor
const uint8_t enable_hand = 29; // Turn the handlebar motor on or off
const uint8_t enable_fork = 30; // Turn the fork motor on or off
const uint8_t enable_motor_enc = 31; // HIGH to send power to the motor encoders

const uint8_t hand_led = 32; // LED installed on the handlebars
const uint8_t hand_switch = 28; // Switch installed on the handlebars

#if USE_BIKE_ENCODERS
const uint8_t encdr_pin1_wheel = 2; //1 of 2 pins to read out the wheel encoder
const uint8_t encdr_pin2_wheel = 3; //1 of 2 pins to read out the wheel encoder
const uint8_t encdr_pin1_pedal = 23; //1 of 2 pins to read out the pedal encoder
const uint8_t encdr_pin2_pedal = 22; //1 of 2 pins to read out the pedal encoder
#endif

//------------------------------ PD Control ----------------------------------//
float error_prev = 0.0f; // Variable to store the previos mismatch between handlebar and fork
uint64_t control_iteration_counter = 0; // TODO: Ensure it never overflows!

//-------------------------- Switch debouncing -------------------------------//
// uint8_t hand_switch_value = 0;
// uint8_t hand_switch_array[10] = {0};
// uint8_t hand_switch_state = 0;
// uint8_t hand_switch_state_prev = 0;

//--------------------------------- Time -------------------------------------//
elapsedMicros since_last_loop; // How long has passed since last loop execution
elapsedMicros since_last_steer_meas; // How long since last handlebar and fork measurement
elapsedMicros since_last_IMU_meas; // How long since last IMU measurement

//------------------- Wheel Speed and Cadence Encoders -----------------------//
#if USE_BIKE_ENCODERS
  Encoder wheel_counter(encdr_pin1_wheel, encdr_pin2_wheel); // Initialize Rear wheel speed encoder
  Encoder pedal_counter(encdr_pin1_pedal, encdr_pin2_pedal); // Initialize Pedal cadence encoder
  int32_t wheel_counts[WHEEL_COUNTS_LENGTH] = {0};
  int32_t pedal_counts[PEDAL_COUNTS_LENGTH] = {0};
  uint32_t wheel_counts_index = 0;
  uint32_t pedal_counts_index = 0;
#endif

//--------------------------------- IMU --------------------------------------//
#if USE_IMU
  bfs::Mpu9250 IMU(&SPI, cs_imu); // MPU9250 object
#endif

//--------------------------- SD Card Logging --------------------------------//
#if USE_SD
  SdExFat sd;
  ExFile root; // Used to count the number of files
  ExFile countFile; // Used to count the number of files
  ExFile logFile; // Used for logging
  String fileName = "sbw_log_";
  String fileExt = ".csv";
  RingBuf<ExFile, RING_BUF_CAPACITY> rb; // Set up the buffer
  int fileCount = 0; // Number of files already on the SD
  bool isOpen = false; // Is file already open?
  bool isFull = false; // Is the file full?
#endif



//============================== [Main Setup] ==================================//
void setup(){
  //------[Initialize communications
  SPI.begin(); // IMU, fork and steer encoders
  #if SERIAL_DEBUG
  Serial.begin(9600); // Communication with PC through micro-USB
  #endif
  
  //------[Setup INPUT pins
  pinMode(hand_switch,      INPUT);

  //------[Setup OUTPUT pins
  pinMode(enable_motor_enc, OUTPUT);
  pinMode(hand_led,         OUTPUT);
  pinMode(enable_fork,      OUTPUT);
  pinMode(enable_hand,      OUTPUT);
  pinMode(pwm_pin_fork,     OUTPUT);
  pinMode(pwm_pin_hand,     OUTPUT);
  pinMode(cs_hand,          OUTPUT);
  pinMode(cs_fork,          OUTPUT);
  #if USE_IMU
  pinMode(cs_imu,           OUTPUT);
  #endif

  //------[Setup PWM pins
  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(pwm_pin_fork, PWM_FREQUENCY);
  analogWriteFrequency(pwm_pin_hand, PWM_FREQUENCY);

  //------[Give output pins initial values
  //Disconnect all sub-modules from the SPI bus. (pull up CS)
  digitalWrite(cs_fork, HIGH);
  digitalWrite(cs_hand, HIGH);
  #if USE_IMU
  digitalWrite(cs_imu,  HIGH);
  #endif
  
  digitalWrite(enable_motor_enc, HIGH); // Set HIGH to enable power to the encoders
  digitalWrite(enable_fork,      HIGH); // Set HIGH to enable motor
  digitalWrite(enable_hand,      HIGH); // Set HIGH to enable motor
  digitalWrite(hand_led,         LOW);
  analogWrite(pwm_pin_fork,      INITIAL_FORK_PWM);
  analogWrite(pwm_pin_hand,      INITIAL_STEER_PWM);


  //------[Setup IMU
  #if USE_IMU
    digitalWrite(cs_imu, LOW);
    if(!IMU.Begin()){ //Initialize communication with the sensor
      #if SERIAL_DEBUG
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      #endif
    }
    IMU.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G); // +- 4g
    IMU.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_250DPS); // +- 250 deg/s
    digitalWrite(cs_imu, HIGH);
  #endif


  #if USE_SD
    //------[Setup SD card
    if(!sd.begin(SdioConfig(FIFO_SDIO))){ //Initialize SD card and file system for SDIO mode. Here: FIFO
      #if SERIAL_DEBUG
      Serial.println("SD card initialization unsuccessful");
      Serial.println("Please check SD card!");
      #endif
      // Short-short error code (..)
      while(1) {
        digitalWrite(hand_led, HIGH);
        delay(500);
        digitalWrite(hand_led, LOW);
        delay(500);
      }
    }

    //------[Count files on SD card
    root.open("/");
    while (countFile.openNext(&root, O_RDONLY)) {
      if (!countFile.isHidden()) fileCount++; // Count only the log files
      countFile.close();
    }
  #endif //USE_SD
  

  //------[Time stuff
  delay(1); //give time for sensors to initialize
  since_last_loop = 0;
  since_last_steer_meas = 0;
  since_last_IMU_meas = 0;
}



//============================== [Main Loop] ===================================//
void loop(){
  #if USE_SD // may be moved to setup with a while loop
    if (!isOpen) open_file();
  #endif
  
  
  if (since_last_loop >= MIN_LOOP_LENGTH_MU){ //K: Sort of have a max freq? (cause that is not garanteed in this way)    
    since_last_loop = since_last_loop - MIN_LOOP_LENGTH_MU; //reset counter

    if (control_iteration_counter >= CTRL_STARTUP_ITTERATIONS) // Turn on LED when bike is ready
      digitalWrite(hand_led, HIGH);

    // // Read the switch state
    // hand_switch_state_prev = hand_switch_state;
    // hand_switch_value = digitalRead(hand_switch);
    // hand_switch_state = check_switch(hand_switch_value, 
    //                                 hand_switch_array,
    //                                 sizeof(hand_switch_array)/sizeof(hand_switch_array[0])
    //                                 );
    
    //------[initialize local variables
    static BikeMeasurements sbw_bike{};
    float error, derror_dt;
    double command_fork = 0;
    double command_hand = 0;

    //------[measure bike states and inputs
    sbw_bike.measure_steer_angles();
    // sbw_bike.measure_hand_torque();
    sbw_bike.calculate_fork_rate();
    sbw_bike.calculate_roll_states();

    //------[Perform steering control
    calc_pd_errors(sbw_bike, error, derror_dt);
    calc_pd_control(error, derror_dt, command_fork, command_hand); //add pd_control to the hand and fork torques
    calc_mm_control(sbw_bike, command_fork); // add model matching torque to fork torque
    
    actuate_steer_motors(command_fork, command_hand);

    //------[Increase counters
    control_iteration_counter++;
  }
}



/*==============================================================================*\
 |                               Helper Functions                               |
\*==============================================================================*/

//=========================== [Get steer angles] ===============================//
void BikeMeasurements::measure_steer_angles(){
  //------[Read encoder values
  uint16_t enc_counts_hand = read_motor_encoder(cs_hand); //SPI communication with Handlebar encoder
  uint16_t enc_counts_fork = read_motor_encoder(cs_fork); //SPI communication with Fork encoder

  //------[Time since last measurement
  update_dtime(m_dt_steer_meas, since_last_steer_meas); // time between to calls to the motor encoder value, used for calculating derivatives

  //------[Translate encoder counts to angle in degree
  /* NOTE: The two encoders are mounted opposite to each other.
  Therefore, CCW rotation of the handlebar encoder gives 360 to 310 deg,
  whereas, CCW rotation of the fork encoder gives 0 to 55 degrees.
  The rotational direction must be the same for both encoders, and
  the encoders must give an output int the 0-180 degrees range. 
  */
  // Handlebar
  m_hand_angle = ((float)enc_counts_hand / HAND_ENC_MAX_VAL) * FULL_ROTATION_DEG - HAND_ENC_BIAS;
  if (m_hand_angle > HALF_ROTATION_DEG) 
    m_hand_angle = m_hand_angle - FULL_ROTATION_DEG; // CCW handlebar rotation gives 360 deg-> 310 deg. Subtract 360 to get 0-180 deg CCW
  // Fork
  m_fork_angle = -(((float)enc_counts_fork / FORK_ENC_MAX_VAL) * FULL_ROTATION_DEG + FORK_ENC_BIAS); //Minus sign to get minus values from fork encoder.
  if (m_fork_angle < -HALF_ROTATION_DEG) 
    m_fork_angle = m_fork_angle + FULL_ROTATION_DEG; // CW fork rotation gives -360 deg-> -310 deg. Add 360 to get 0-180 deg CCW


  //------[Compensate for difference in range of motion of handelbar and fork
  /* NOTE: The fork mechanical range is +- 42 degrees. Handlebars do not 
  have a mechanical limit, however. To relay the mechanical limit of 
  the fork to the steer, software limits are set below. If you do not set
  this limits, the motor folds back.
  */
  m_hand_angle = constrain(m_hand_angle, -SOFTWARE_LIMIT, SOFTWARE_LIMIT); //K: This may cause the oscillations
}


//=========================== [Get handlebar torque] ===============================//
void BikeMeasurements::measure_hand_torque(){

}


//======================= [calculate steer derivatives] ============================//
void BikeMeasurements::calculate_fork_rate(){
  m_fork_angle = steer_moving_avg(m_fork_angle);
  m_fork_rate = calc_bckwrd_derivative(m_fork_angle, m_fork_angle_prev, m_dt_steer_meas);
  return;
}


//======================= [calculate roll rate and angle] ==========================//
void BikeMeasurements::calculate_roll_states(){
  // TODO: make sure that gyrox is indeed the roll rate!
  // TODO: Use the gravitational acceleration and a kalman filter 
  //       or Maximum Likelyhood Estimator to more acurately 
  //       predict attitude

  /*NOTE: We assume that the current measured value is constant
  untill the next measurement. The time between the current 
  measurement and the next measurement is given by m_dt_IMU_meas,
  while the current measurement is given by m_lean_rate.*/

  get_IMU_data(); // get next measurement and time between current and next
  m_lean_angle += riemann_integrate(m_lean_rate, m_dt_IMU_meas); //use current measurement and time till next to calculate integral
  m_lean_rate = IMU.gyro_x_radps(); //next measurement becomes current measurement
}


//============================ [Calculate PD error] ============================//
void calc_pd_errors(BikeMeasurements& bike, float& error, float& derror_dt){
  //------[Calculate error derivative in seconds^-1
  // S: Set the very first handlebar and fork encoder values to 0
  // D: setting handlebar and fork encoder first values to 0, we do that because first values seem to be floating values
  // K: most likely due to encoder just haven got power, and still initializing. A delay in the setup might fix this
  if (control_iteration_counter < 10){ 
    bike.set_hand_angle(0.0f);
    bike.set_fork_angle(0.0f);
  }
  error = (bike.get_hand_angle() - bike.get_fork_angle());
  derror_dt = calc_bckwrd_derivative(error, error_prev, bike.get_dt_steer_meas());
  return;
}


//=========================== [Calculate PD Control] ===========================//
void calc_pd_control(float error, float derror_dt, double& command_fork, double& command_hand){
  //------[Calculate PID torques
  command_fork += (KP_F*error + KD_F*derror_dt) / return_scaling(control_iteration_counter); //Scaling is done to prevent a jerk of the motors at startup
  command_hand += (KP_H*error + KD_H*derror_dt) / return_scaling(control_iteration_counter); // , as the fork and handlebar can be misaligned
  return;
}

//=========================== [Calculate model matching Control] ===========================//
void calc_mm_control(BikeMeasurements& bike, double& command_fork){

  //TODO: look into if it can be maded angle controlled?

  //------[Calculate model matching torques
  /* NOTE: The torque is only applied to the fork, as the driver should not notice 
  that the fork is moving differently than the handlebar. Furthermore, we assume the 
  lean torque is zero, aka there is no external torque in lean direction applied
  */
  command_fork +=   K_MM1*bike.get_lean_angle()
                  + K_MM2*bike.get_fork_angle() 
                  + K_MM3*bike.get_lean_rate() 
                  + K_MM4*bike.get_fork_rate() 
                  // + K_MM5*bike.get_lean_torque() //we cannot measure lean torque, so we assume it is zero. 
                  + K_MM6*bike.get_hand_torque();
}


//=========================== [Actuate steer motors] ===========================//
void actuate_steer_motors(double command_fork, double command_hand){
  //------[Find the PWM command
  uint64_t pwm_command_fork = (command_fork * -842.795 + 16384); //K: magic numbers, what do they mean!!!
  uint64_t pwm_command_hand = (command_hand * -842.795 + 16384); 
  pwm_command_fork = constrain(pwm_command_fork, FORK_PWM_MIN, FORK_PWM_MAX);
  pwm_command_hand = constrain(pwm_command_hand, HAND_PWM_MIN, HAND_PWM_MAX);

  //------[Send motor command
  analogWrite(pwm_pin_hand, pwm_command_hand);
  analogWrite(pwm_pin_fork, pwm_command_fork);
  return;
}


#if USE_BIKE_ENCODERS
//========================= [Calculate bicycle speed] ==========================//
float calc_bike_speed(){
  /*K: NOTE Since the microcontroller operating frequency is much higher than the
  frequency at which encoder ticks pass the reading head, the difference between
  the tick count of the current and previous loop will most of the time be zero.
  To have a meaningfull value, the value of this loop and that of 
  WHEEL_COUNTS_LENGTH ago are compared. WARNING: It is assumed that the loop has
  a constant frequency, which is not the case.*/
  //TODO: Use timers instead of loops
  int32_t current_wheel_count = wheel_counter.read();
  int32_t previous_wheel_count = wheel_counts[wheel_counts_index];
  wheel_counts[wheel_counts_index] = current_wheel_count;
  wheel_counts_index += 1;
  if (wheel_counts_index >= WHEEL_COUNTS_LENGTH) wheel_counts_index = 0;

  /*NOTE #rounds = count_diff/WHEEL_COUNTS_PER_REV. The count_diff is measured 
  WHEEL_COUNTS_LENGTH loops away from each other. A loop (should) take
  MIN_LOOP_LENGTH_S sec. So time_diff = WHEEL_COUNTS_LENGTH * 
  MIN_LOOP_LENGTH_S. Than rounds per second = #rounds/time_diff
  */ 
  float rps_wheel = ((float)(current_wheel_count - previous_wheel_count)) / 
  ((float)WHEEL_COUNTS_PER_REV * (float)WHEEL_COUNTS_LENGTH * MIN_LOOP_LENGTH_S);
  float velocity_ms = -rps_wheel * 2*PI * WHEEL_RADIUS;

  return velocity_ms;
} 



//============================ [Calculate cadance] =============================//
float calc_cadance(){
  /*K: NOTE Since the microcontroller operating frequency is much higher than the
  frequency at which encoder ticks pass the reading head, the difference between
  the tick count of the current and previous loop will most of the time be zero.
  To have a meaningfull value, the value of this loop and that of 
  WHEEL_COUNTS_LENGTH ago are compared. WARNING: It is assumed that the loop has
  a constant frequency, which is not the case.*/
  //TODO: Use timers instead of loops
  int32_t current_pedal_count = pedal_counter.read();
  int32_t previous_pedal_count = pedal_counts[pedal_counts_index];
  pedal_counts[pedal_counts_index] = current_pedal_count;
  pedal_counts_index += 1;
  if (pedal_counts_index >= PEDAL_COUNTS_LENGTH) pedal_counts_index = 0;

  /*NOTE #rounds = count_diff/PEDAL_COUNTS_PER_REV. The count_diff is measured 
  PEDAL_COUNTS_LENGTH loops away from each other. A loop (should) take
  MIN_LOOP_LENGTH_S sec. So time_diff = PEDAL_COUNTS_LENGTH * 
  MIN_LOOP_LENGTH_S. Than rps = #rounds/time_diff. Then rad/s = rps*2pi
  */
  float cadence_rads = ((float) (current_pedal_count - previous_pedal_count)) / 
  ((float)PEDAL_COUNTS_PER_REV *(float) PEDAL_COUNTS_LENGTH * MIN_LOOP_LENGTH_S)
  * 2*PI;
    
  return cadence_rads;
}
#endif //USE_BIKE_ENCODERS



//=============================== [Read the IMU] ===============================//
#if USE_IMU
void get_IMU_data(){
  //------[Read out data via SPI
  digitalWrite(cs_imu, LOW);
  IMU.Read(); // load IMU data into IMU object
  digitalWrite(cs_imu, HIGH);

  //------[Time since last measurement
  update_dtime(m_dt_IMU_meas, since_last_IMU_meas); // time between to calls to the IMU

  /* Only left for documentation reasons */
  // float accelY = IMU.accel_x_mps2();
  // float accelX = IMU.accel_y_mps2();
  // float accelZ = IMU.accel_z_mps2();
  // float gyroY = IMU.gyro_x_radps();
  // float gyroX = IMU.gyro_y_radps();
  // float gyroZ = IMU.gyro_z_radps();
  // float temp = IMU.die_temp_c();
}
#endif
  


//=========================== [Print to serial] ===========================//
#if SERIAL_DEBUG
// void print_to_serial(){
//   if (control_iteration_counter % 100 == 0){ // Limit the printing rate
//     // Serial.print("Switch: ");
//     // Serial.print(hand_switch_state);
//     // Angles
//     Serial.print(",Hand(deg)=");
//     Serial.print(hand_angle);
//     Serial.print(",Fork(deg)=");
//     Serial.print(fork_angle);
//     // Torque
//     Serial.print(",Torque_handlebar(Nm)=");
//     Serial.print(command_hand);
//     Serial.print(",Torque_fork(Nm)=");
//     Serial.print(command_fork);
//     Serial.print(",Hand(Counts)= ");
//     Serial.print(enc_counts_hand);
//     Serial.print(",Fork(Counts)= ");
//     Serial.print(enc_counts_fork);
//     Serial.print(",Time Taken= ");
//     Serial.print(since_last_loop);
//     // IMU
//     #if USE_IMU 
//       Serial.print(",AccelX= ");
//       Serial.print(accelX);
//       Serial.print(",AccelY= ");
//       Serial.print(accelY);
//       Serial.print(",AccelZ= ");
//       Serial.print(accelZ);
//       Serial.print(",GyroX= ");
//       Serial.print(gyroX);
//       Serial.print(",GyroY= ");
//       Serial.print(gyroY);
//       Serial.print(",GyroZ= ");
//       Serial.print(gyroZ);
//       Serial.print(",Temp= ");
//       Serial.print(temp);
//     #endif
//     // Encoders
//     #if USE_BIKE_ENCODERS
//       Serial.print(",Velocity= ");
//       Serial.print(velocity_ms);
//       Serial.print(",Cadence= ");
//       Serial.print(cadence_rads);
//     #endif
//     // New line
//     Serial.println();
//   }
// }
#endif //SERIAL_DEBUG



//============================= [Print to SD] =============================//
#if USE_SD
void print_to_SD(){
  size_t n = rb.bytesUsed();

  // Check if there is free space
  if ((logFile.curPosition() + n) > (LOG_FILE_SIZE - 100)) {
    digitalWrite(hand_led, LOW); //notify the user via LED
    isFull = true; //TODO: open new file
  }

  if (!isFull) {
    // If the file is not busy, write out one sector of data
    if (n >= 512 && !logFile.isBusy()) 
      rb.writeOut(512);

    // // Write data to buffer
    // rb.print(control_iteration_counter); //format must match that of 'open_file()'
    // rb.write(',');
    // rb.print(since_last_loop);
    // // rb.write(',');
    // // rb.print(hand_switch_state);
    // rb.write(',');
    // rb.print(hand_angle,2);
    // rb.write(',');
    // rb.print(fork_angle,2);
    // rb.write(',');
    // // rb.print(angle_rate,2);
    // // rb.write(',');
    // // rb.print(filtered_angle_rate,2);
    // // rb.write(',');
    // rb.print(error,2);
    // rb.write(',');
    // rb.print(command_fork,2);
    // rb.write(',');
    // rb.print(command_hand,2);
    // #if USE_IMU 
    // rb.write(',');
    // rb.print(gyroX,3);
    // rb.write(',');
    // rb.print(gyroY,3);
    // rb.write(',');
    // rb.print(gyroZ,3);
    // #endif
    // rb.write('\n');
  }

  // Flush the data from buffer to file. Try to do it at rarely as possible.
  // Flushing takes a couple of milliseconds (around 3-4), which makes the
  // next 3 or 4 PID controller iterations slightly out-of-time.
  // Since the normal flush-less iteration takes significantly less than 1ms, 
  // the controller gets back in time quickly.
  // CAN SOMETIMES CAUSE A LAG SPIKE OF MULTIPLE SECONDS
  // TODO: Find a workaround
  if (control_iteration_counter % 1500 == 0) logFile.flush();
}
#endif //USE_SD



//========================== [Read Motor Encoder] =========================//
uint16_t read_motor_encoder(const uint8_t cs_pin){
  /* Communication with encoder goes through SSI protecol variant.
  |  But we read it out via SPI protecol. Since the encoder uses 
  |  13 bits, we need to select those out of the message we recieve
  |  from the sub(/slave). These 13 bits are placed as follows.
  |  .*** **** **** **.. Messages sent to the sub from the main (MOSI)
  |  will be disregarded as SSI is not duplex.
  |  For more info see:   
  |  > data sheet of RMB20SC.
  |  > https://en.wikipedia.org/wiki/Synchronous_Serial_Interface
  |  > https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
  |  > https://arduino.stackexchange.com/questions/55470/interfacing-with-an-ssi-sensor
  |  
  |  FROM ORIGINAL COMMENT: 
  |    Set frequency to 125 Khz-4 Mhz <--(from the datasheet)
  |    encoder transmit first the MSB <--(from the datasheet)
  |    Clock Idles High Latch on the initial clock edge
  |    sample on the subsequent edge
  |
  |  Sampling on the subsequent edge is probably why we need to disregard the first bit 
 */

  SPI.beginTransaction(SPISettings(ENCODER_CLK_FREQ, MSBFIRST, SPI_MODE3)); //Open transactions on SPI BUS
  digitalWrite(cs_pin, LOW); // LOW to connect to SPI bus
  uint16_t enc_val = SPI.transfer16(0); // Transfer to MOSI (0) and read what comes back to MISO (enc_val)
  enc_val = (enc_val & 0x7fff) >> 2; // select the 13 bits
  digitalWrite(cs_pin, HIGH); // HIGH to disconnect from SPI bus. Also stop clock and thus communicate EOT to ssi interface.
  SPI.endTransaction(); // Close transactions on SPI bus
  return enc_val;
}


//================= [Update time between measurments of the steer angels] =================//
void update_dtime(uint32_t& dtime, elapsedMicros& timer){
  // TODO: make 'dtime' and 'timer' linked to each other. As in dtime and 
  // timer are a pair. If not, either timer is reset by another dtime 
  // messing up this dtime. Or dtime is set by the wrong timer.
  // maybe make it a class on its own?
  dtime = timer;
  timer = 0;
  return;
}

float riemann_integrate(float value, uint32_t dt){
  return value*dt;
}

//================= [Calculate Derivative Backward Euler] =================//
float calc_bckwrd_derivative(float val_cur, float& val_prev, uint32_t dt){
  /* Calculate the derivative with backward Euler. The previous value is updated 
  to the current after that values derivative is calulated. WARNING: This should 
  be the only place where the value of 'previous' is altered.
  */
  float derivative = (val_cur - val_prev)/(dt * 1e-6);
  val_prev = val_cur;
  return derivative;
}
// As a replacement of:
  // uint32_t error_time_curr = micros();
  // float error_time_diff = ((float) (error_time_curr - error_time_prev)) / 1000000.0f;
  // error_time_prev = error_time_curr;
  // // Calculation of dError
  // float error_curr = (hand_angle - fork_angle);
  // float error = (error_curr - error_prev) / error_time_diff;
  // error_prev = error_curr;



//=========================== [Return Scaling] ============================//
float return_scaling(uint64_t iteration){
  // Slowly ramps up the torque over 13 seconds. Used to avoid commanding high 
  // torques when turning on the bicycle, if the handlebars and the wheel are
  // not aligned.
  if (iteration <= (uint64_t) 6000) 
    return 35.0f;
  else if (iteration <= (uint64_t) 7000)
    return 30.0f;
  else if (iteration <= (uint64_t) 8000)
    return 25.0f;
  else if (iteration <= (uint64_t) 9000)
    return 20.0f;
  else if (iteration <= (uint64_t) 10000)
    return 15.0f;
  else if (iteration <= (uint64_t) 11000)
    return 10.0f;
  else if (iteration <= (uint64_t) 12000)
    return 5.0f;
  else if (iteration <= (uint64_t) 13000)
    return 2.5f;
  
  return 1.0f;
}



//=========================== [Moving Average] ============================//
float steer_moving_avg(float new_value){
  // TODO: At this point the moving average will break if it is used 
  // for two separate signals. Make the function generic
  static float avg_sum = 0;
  static uint8_t avg_idx = 0;
  static float avg_array[STEER_MVING_AVG_SMPL_LEN] = {0};

  avg_sum = avg_sum - avg_array[avg_idx];
  avg_array[avg_idx] = new_value;
  avg_sum = avg_sum + new_value;
  avg_idx = (avg_idx+1) % STEER_MVING_AVG_SMPL_LEN;
  return (float) (avg_sum / STEER_MVING_AVG_SMPL_LEN);
}


//============================ [Check Switch] =============================//
uint8_t check_switch(uint8_t curr_value, uint8_t *val_array, uint8_t array_size){
  // Taking raw switch value is not reliable as the value can sometimes jump to
  // 0 even if the switch is on. This function tracks the last array_size
  // switch values and outputs 1 if at least 70% (rounded down) of them were 
  // 1. Otherwise, returns 0.
  for (uint8_t i = 0; i < array_size - 1; i++) val_array[i] = val_array[i+1];
  val_array[array_size - 1] = curr_value;

  uint16_t val_sum = 0;
  for (uint8_t i = 0; i < array_size; i++) val_sum += val_array[i];

  if (val_sum >= (uint8_t)(array_size*0.7)) return 1;
  return 0;
}



//============================== [Open File] ==============================//
#if USE_SD
void open_file() {
  String fullFileName = fileName + String(fileCount) + fileExt;

  // Open file
  if (!logFile.open(fullFileName.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("Open failed");
    while (1) { // Long-long error code
      digitalWrite(hand_led, HIGH);
      delay(1000);
      digitalWrite(hand_led, LOW);
      delay(500);
    }
  }
  
  //Prealocate space
  if (!logFile.preAllocate(LOG_FILE_SIZE)) {
    Serial.println("Preallocate failed");
    while (1) { // Long-long error code
      digitalWrite(hand_led, HIGH);
      delay(1000);
      digitalWrite(hand_led, LOW);
      delay(500);
    }
  }

  // Start Writing to buffer
  rb.begin(&logFile);

  // rb.print("control_iteration_counter"); //This should equal the format used in 'print_to_SD'
  // rb.write(',');
  // rb.print("mpc_iteration_counter");
  // rb.write(',');
  // rb.print("since_last_loop");
  // // rb.write(',');
  // // rb.print("hand_switch_state");
  // rb.write(',');
  // rb.print("hand_angle");
  // rb.write(',');
  // rb.print("fork_angle");
  // rb.write(',');
  // // rb.print("angle_rate");
  // // rb.write(',');
  // // rb.print("filtered_angle_rate");
  // // rb.write(',');
  // rb.print("error");
  // rb.write(',');
  // rb.print("command_fork");
  // rb.write(',');
  // rb.print("command_hand");
  // #if USE_IMU
  //   rb.write(',');
  //   rb.print("gyroX");
  //   rb.write(',');
  //   rb.print("gyroY");
  //   rb.write(',');
  //   rb.print("gyroZ");
  // #endif
  // rb.write('\n');

  // Write to SD card
  size_t n = rb.bytesUsed();
  rb.writeOut(n);
  logFile.flush();

  isOpen = true; // Set to TRUE such that the open file function is only called once
  return;
}
#endif