// Note: include the "eigen.h" library first, otherewise there will be compile errors.
#include "eigen.h" //https://github.com/bolderflight/eigen
#include "simpleKalman.h"
#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include "SdFat.h"
#include "RingBuf.h" //From sdFat library
#include "mpu9250.h" //https://github.com/bolderflight/MPU9250
/*TODO: include other measurement functions into the BikeMeasurements class, like the IMU*/
// TODO: make all measurement units into SI units

/* LEFT FOR DOCUMENTATION PURPOSE ONLY [transfer to more appropriate location and remove]
a_force = 20; // Analog output pin of the force transducer
a_torque = 21; // Analog output pin of the torque sensor
a_fork = 40; // Analog output pin of the fork motor drive
a_hand = 41; // Analog output pin of the handlebar motor drive
*/

//============================== Compile modes ===============================//
#define USE_IMU 1
#define USE_BT 0
#define USE_SD 0
#define USE_PEDAL_CADANCE 0
#define SERIAL_DEBUG 1

//================================= Classes ==================================//
class BikeMeasurements{
  private:
    // state, input, and bicycle variables
    float m_hand_angle;    // [rad]
    float m_fork_angle;    // [rad]
    float m_lean_angle;    // [rad]
    float m_fork_rate;     // [rad/s]
    float m_lean_rate;     // [rad/s]
    float m_hand_torque;   // Measurement of the torque on the handlebar applied by the human 
    float m_lean_torque;    // [Nm] Measurement of the input impulse on the force trancducer beneath the seat.
    float m_bike_speed;    // [m/s]
    #if USE_PEDAL_CADANCE
    float m_pedal_cadance; // [rad/s]
    #endif
    
    float m_lean_angle_meas; // [rad]
    float m_omega_x_old;     // [rad/s]

    // variables needed for derivarion/intergration calculation
    uint32_t m_dt_bike_speed_meas; //Time between two consecutive measurements of the bike speed in microseconds
    uint32_t m_dt_steer_meas; //Time between two consecutive measurements of the steer angle in microseconds
    uint32_t m_dt_IMU_meas; //Time between two consecutive measurements of the IMU values in microseconds
    float m_fork_angle_prev; // [rad]
    
  public:
    BikeMeasurements(){
      m_hand_angle = 0;
      m_fork_angle = 0;
      m_lean_angle = 0;
      m_fork_rate = 0;
      m_lean_rate = 0;
      m_hand_torque = 0;
      m_lean_torque = 0;
      m_bike_speed = 0;
      #if USE_PEDAL_CADANCE
      m_pedal_cadance = 0;
      #endif

      m_lean_angle_meas = 0;
      m_omega_x_old = 0;

      m_dt_bike_speed_meas = 0;
      m_dt_IMU_meas = 0;
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
    float get_lean_torque(){return m_lean_torque;}
    uint32_t get_dt_steer_meas(){return m_dt_steer_meas;}
    float get_bike_speed(){return m_bike_speed;}
    #if USE_PEDAL_CADANCE
    float get_pedal_cadance(){return m_pedal_cadance;}
    #endif

    // setters
    void set_hand_angle(float angle){m_hand_angle = angle;}
    void set_fork_angle(float angle){m_fork_angle = angle;}

    // Retreive measurements
    void measure_steer_angles();
    void measure_hand_torque();
    void measure_lat_perturbation();
    void calculate_fork_rate();
    void calculate_roll_states();
    void calc_lean_angle_meas(float omega_x, float omega_y, float omega_z);
    void calculate_bike_speed();
    #if USE_PEDAL_CADANCE
    void calculate_pedal_cadance();
    #endif
};

//=========================== Function declarations ===========================//
void calc_pd_errors(BikeMeasurements& bike, float& error, float& derror_dt);
void calc_pd_control(float error, float derror_dt, double& command_fork, double& command_hand);
void calc_mm_control(BikeMeasurements& bike, double& command_fork);
void calc_sil_control(BikeMeasurements& bike, double& command_fork, double& command_hand);
void calc_mm_sil_control(BikeMeasurements& bike, double& command_fork, double& command_hand);
void actuate_steer_motors(double command_fork, double command_hand);
void calc_mm_gains(float& k_phi, float& k_delta, float& k_dphi, float& k_ddelta, float& k_tphi, float& k_tdelta, float speed);
uint16_t read_motor_encoder(const uint8_t cs_pin);
void update_dtime(uint32_t& dtime, elapsedMicros& timer);
float riemann_integrate(float value, uint32_t dt);
float calc_bckwrd_derivative(float val_cur, float& val_prev, uint32_t dt);
float return_scaling(uint64_t iteration);
uint8_t check_switch(uint8_t curr_value, uint8_t *val_array, uint8_t array_size);
float steer_moving_avg(float new_value);
void calc_friction_callibration_control(uint64_t loop_iter, double& command);
void calc_directional_bias_callibration(uint64_t loop_iter, double& fork_command);
void apply_friction_compensation(double& fork_command);
#if USE_BT
void bt_setup();
void print_to_bt(BikeMeasurements& sbw_bike, double command_fork, double command_hand);
#endif
#if SERIAL_DEBUG
void serial_setup();
void print_to_serial(BikeMeasurements& bike, double command_fork, double command_hand);
#endif
#if USE_IMU
void imu_setup();
void get_IMU_data(uint32_t& dt_IMU_meas);
#endif
#if USE_SD
void sd_setup();
void open_file();
void print_to_SD(BikeMeasurements bike, float command_fork, float command_hand);
#endif

/*copied from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c 
made by user79758 and stef, altered to fit current implementation.
Under https://creativecommons.org/licenses/by-sa/2.5/ (original post),
and https://creativecommons.org/licenses/by-sa/4.0/ (edited post)*/
template <typename T> 
int sgmd(T val) { return (T(0) < val) - (val < T(0));}

template <typename T> 
int sgn(T val) { return (T(0) <= val) - (val < T(0));} //altered to give 1 at zero. (so no longer sigmund function) This was required for the implementation regarding of Sanjurjo

//============================= Global Variables =============================//
//-------------------------------- Constants ---------------------------------//
// Bluetooth
#if USE_BT
const uint32_t BT_BAUDRATE = 9600;
#endif

// physical quantities
const float GRAVITY = 9.81;

// Conversion
const float MICRO_TO_UNIT = 1e-6;

// PWM Setup
const uint8_t PWM_RESOLUTION_BITS = 15;
const float PWM_FREQUENCY = 4577.64;

// Commanded torque
// const int8_t MAX_FORK_TORQUE_RATE = 20;
// const int8_t MAX_HAND_TORQUE_RATE = 5;
const uint32_t HAND_PWM_MAX = 24812;
const uint32_t HAND_PWM_MIN = 7956;
const uint32_t FORK_PWM_MAX = 24812;
const uint32_t FORK_PWM_MIN = 7956;
const uint16_t INITIAL_FORK_PWM = 16384; //K: I think that the middle is a zero command. That 0 and 32,768 are both maximum torque but in opposite directions.
const uint16_t INITIAL_STEER_PWM = 16384;

// Steer Torque
const float TEENSY_ANALOG_VOLTAGE = 3.3;
const uint16_t HAND_TORQUE_RESOLUTION = 1023;
const uint8_t TORQUE_SLOPE = 1;
const uint8_t TORQUE_BIAS  = 0;

// Lateral force
float LAT_FORCE2TORQUE = 0.95; // Height of the force sensor attachment point, measured from the ground in meters. (wheels at 4bar)
float MEAS2LAT_FORCE = (1/29.382639222963817)*9.81; // [kg/-]*[N/kg]calibration has been done in [kg](independend) vs [-](dependend){no unit as it is a mapping from 0-3,3V to 0-1023}
uint8_t FORCE_BIAS_AVGING_WINDOW = 200;

// Timing
const uint16_t MIN_LOOP_LENGTH_MU = 10000; // target minimum loop length in microseconds. (Currently set at dt = 0.01sec as this corrosponsds with the current Kalman gains)
const float MIN_LOOP_LENGTH_S = MIN_LOOP_LENGTH_MU*MICRO_TO_UNIT; //
const float LOOP_TIME_SCALING = 1000.0F/(float)MIN_LOOP_LENGTH_MU; // The old code was written for startup time of 13 seconds having a loop time of 1 milisec. Since I chanced the loop time, the number of itterations needed for 13 sec also has to scale appropriately
const uint16_t CTRL_STARTUP_ITTERATIONS = 13000*LOOP_TIME_SCALING; // itterations in which the steer torques are slowly scaled to unity.

// Motor encoders
const uint32_t ENCODER_CLK_FREQ = 225000; //clock frequency for the encoders SPI protocol
const float HAND_ENC_BIAS = 153.65 * DEG_TO_RAD + 0.0203;
const float FORK_ENC_BIAS = 100.65 * DEG_TO_RAD - 0.0564;
const float HAND_ENC_MAX_VAL = 8192.0; //ticks go from 0 to 8191. At the 8192th tick encoder_tick/HAND_ENC_MAX_VAL = 1 --> 2*pi == 0
const float FORK_ENC_MAX_VAL = 8192.0;

// Pedal and wheel encoders
/*NOTE: A WHEEL_COUNTS_LENGTH of 500 gives an approximate 45 counts per calculation 
for a bike going 1 m/s. It then also calculates the average speed of the last 
0.5 seconds. So it will have trouble with high frequency sinosoidal translations. 
We assume here that the speed is always in the forward direction. Unfortunetely, 
with this size (500) the resolution is noticably lower for lower speeds.*/
const uint16_t WHEEL_COUNTS_LENGTH = 500; 
const uint8_t WHEEL_COUNTS_PER_REV = 192;
const float WHEEL_RADIUS = 0.33f; //[m]
#if USE_PEDAL_CADANCE
/*NOTE: there is currently no sensor to measure the encoder ticks*/
const uint16_t PEDAL_COUNTS_LENGTH = 500;
const uint8_t PEDAL_COUNTS_PER_REV = 192;
#endif

// Angles
const float SOFTWARE_LIMIT = 42.0 * DEG_TO_RAD;

// SD card
#if USE_SD
const uint16_t SD_BLOCK_SIZE = 512;
const uint16_t SD_SAMPLING_FREQ = 1000; // [Hz] Sampling frequency of the SD card
const uint64_t MESSAGE_LENGTH = 100; // The maximum expected legth of one sample of data. In bytes
const uint16_t EXPERIMENT_TIME = 600; // In seconds
const uint8_t  BUFFER_HOLD_TIME = 2; // In seconds
const uint64_t LOG_FILE_SIZE = MESSAGE_LENGTH*EXPERIMENT_TIME*SD_SAMPLING_FREQ; // Log file should hold 10 minutes of data sampled at 1kHz
const uint64_t RING_BUF_CAPACITY = MESSAGE_LENGTH*BUFFER_HOLD_TIME*SD_SAMPLING_FREQ // Buffer should hold 2 seconds of data sampled at 1kHz
                                   + (SD_BLOCK_SIZE - ((MESSAGE_LENGTH*BUFFER_HOLD_TIME*SD_SAMPLING_FREQ) % SD_BLOCK_SIZE)); // make it a multiple of 512 for better SD writting (512 is one block)
#endif

// PD Gains
/*In the original implementation by georgias and simonas, the angles 
were in degree. The gains were tuned for degrees as well. Now that all 
angles are converted to radians, the gain needs to be compensated to give 
the same value
former: Kp_old*error_deg
New Kp_new*error_rad = Kp_old*rad2deg * error_deg*deg2rad 
                     = Kp_old*error_deg */
const float KP_F = 0.5*2.0f * RAD_TO_DEG; // Fork
const float KD_F = 0.029f * RAD_TO_DEG; // Fork
const float KP_H = 0.5*0.9f * RAD_TO_DEG; // Handlebar
const float KD_H = 0.012f * RAD_TO_DEG; // Handlebar

// Steer into lean gains (see 'Some recent developments in bicycle dynamics and control', A. L. Schwab et al., 2008)
const uint8_t K_SIL1 = 2; // [Ns^2/rad] gain for the steer into lean controller when below stable speed range
const float K_SIL2 = 0.7; // [Ns/rad] gain for the steer into lean controller when above stable speed range
const float V_AVERAGE = 5.5; // [m/s] value somewhere in the stable speed range. (take the average of min and max stable speed)
const float FORK_FRICTION_COMP_RATIO = 0.3;

// Model matching gains: The "_Vx" indicates that the coefficient
//  is multiplied with speed to the power of x.
const float K_MM_PHI_V0 = 0.840841241252; // lean angle
const float K_MM_DELT_V0 = 0.120254094656; // steer/fork angle
const float K_MM_DELT_V2 = -0.183092924965; // steer/fork angle
const float K_MM_DPHI_V1 = 0.373289545485; // lean rate
const float K_MM_DPHI_VMIN1 = -2.53819533238e-13; // lean rate
const float K_MM_DDELT_V1 = -0.041905112053; // steer/fork rate
const float K_MM_DDELT_VMIN1 = -6.82478414984e-14; // steer/fork rate
const float K_MM_TPHI_V0 = 0.00203761694971; // lean torque
const float K_MM_TDELT_V0 = 0.940050621145; // steer/hand torque

//----------------------- Steering rate calculation --------------------------//
const uint8_t STEER_MVING_AVG_SMPL_LEN = 10;

//---------------------------------- IMU -------------------------------------//
#if USE_IMU
const uint32_t WIRE_FREQ = 400000; // [Hz] Frequency set by bolderflight example. It seems to work so I did not alter it.
#endif

//--------------------------- Kalman filtering -------------------------------//
const Eigen::Matrix<float,2,1> X0 {0,0}; //initial state vector
const double T0 = 0; //initial time

// roll angle measurement estimation
float PHI_METHOD_WEIGHT = 0.05;

//--------------------------- Fork friction compensation -------------------------------//
const uint16_t LOOPS_PER_SEC = (1E6/MIN_LOOP_LENGTH_MU);
const uint16_t LOOPS_PER_HALF_SEC = LOOPS_PER_SEC/2;
const float FRIC_COMPENSATION_BIAS = 0.2;
const float FRICT_CAL_CTRL_BOUND = 1;
const float FRIC_CAL_STEP_INCREASE = 0.1;
const float DIR_BIAS_INCREASE_STEP = 0.01;
const float BASE_STEER_TORQUE_DIR_BIAS = 1.5;

//-------------------------------- Pins --------------------------------------//
const uint8_t cs_hand = 24; // SPI Chip Select for handlebar encoder
const uint8_t cs_fork = 25; // SPI Chip Select for fork encoder

const uint8_t pwm_pin_hand = 8; // Send PWM signals to the handlebar motor
const uint8_t pwm_pin_fork = 9; // Send PWM signals to the fork motor
const uint8_t enable_hand = 29; // Turn the handlebar motor on or off
const uint8_t enable_fork = 30; // Turn the fork motor on or off
const uint8_t enable_motor_enc = 31; // HIGH to send power to the motor encoders

const uint8_t hand_led = 32; // LED installed on the handlebars
const uint8_t hand_switch = 28; // Switch installed on the handlebars

const uint8_t transducer_pin = 20; // Analog output of the force transducer
const uint8_t a_torque = 21; // Analog output pin of the torque sensor

const uint8_t encdr_pin1_wheel = 2; //1 of 2 pins to read out the wheel encoder
const uint8_t encdr_pin2_wheel = 3; //1 of 2 pins to read out the wheel encoder
#if USE_PEDAL_CADANCE
const uint8_t encdr_pin1_pedal = 23; //1 of 2 pins to read out the pedal encoder
const uint8_t encdr_pin2_pedal = 22; //1 of 2 pins to read out the pedal encoder
#endif

//---------------------------- Torque command --------------------------------//
uint32_t dt_torque_command = 0;
float command_fork_prev = 0;
float command_hand_prev = 0;

//---------------------------- fork friction callibration --------------------------------//
int itteration_step_friction = 0;
int8_t steer_direction_friction = 1;
int itteration_step_dir_bias = 0;
bool steer_direction_dir_bias = true;

//----------------------- lateral force measurement --------------------------//
float force_bias = 0;
bool haveSampledBias = false;


//------------------------------ PD Control ----------------------------------//
float error_prev = 0.0f; // [rad] Variable to store the previos mismatch between handlebar and fork
uint64_t control_iteration_counter = 0; // TODO: Ensure it never overflows!

//-------------------------- Switch debouncing -------------------------------//
// uint8_t hand_switch_value = 0;
// uint8_t hand_switch_array[10] = {0};
// uint8_t hand_switch_state = 0;
// uint8_t hand_switch_state_prev = 0;

//--------------------------------- Time -------------------------------------//
elapsedMicros since_last_loop; // How long has passed since last loop execution
elapsedMicros since_last_steer_meas; // How long since last handlebar and fork measurement
elapsedMicros since_last_bike_speed; // How long since last bike speed measurement
elapsedMicros since_last_torque_command; // How long since last torque command
#if USE_IMU
elapsedMicros since_last_IMU_meas; // How long since last IMU measurement
#endif

//------------------- Wheel Speed and Cadence Encoders -----------------------//
  Encoder wheel_counter(encdr_pin1_wheel, encdr_pin2_wheel); // Initialize Rear wheel speed encoder
  int32_t wheel_counts[WHEEL_COUNTS_LENGTH] = {0};
  uint32_t wheel_counts_index = 0;
  int32_t end_of_array_storage = 0; //to store the value of the encoder at the end of the wheel_counts array before resetting.
  uint32_t bike_speed_dt_sum_mu = 0;
  uint32_t bike_speed_dt_array[WHEEL_COUNTS_LENGTH] = {0};
#if USE_PEDAL_CADANCE
  Encoder pedal_counter(encdr_pin1_pedal, encdr_pin2_pedal); // Initialize Pedal cadence encoder
  int32_t pedal_counts[PEDAL_COUNTS_LENGTH] = {0};
  uint32_t pedal_counts_index = 0;
#endif

//--------------------------------- IMU --------------------------------------//
#if USE_IMU
  bfs::Mpu9250 IMU; // MPU9250 object
#endif
// Calibration matrix. Translate coordinates expressed IMU frame to the Body fixed frame
Eigen::Matrix<float,3,3> B_ROT_IMU {{-0.10964618,  0.07170863, -0.9928662 },
                                    {-0.00522393, -1.00139095, -0.02117246},
                                    { 0.99607305, -0.03168316, -0.08985829}};
// starting biasses remaining after rotation matrix.
float OMEGA_Y_BIAS = -0.02;
float OMEGA_Z_BIAS = -0.01;

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

//--------------------------- Kalman filtering -------------------------------//
// System matrices
Eigen::Matrix<float,2,2> F {{1,0},{0,1}};
Eigen::Matrix<float,2,1> B {{0},{0}}; 
Eigen::Matrix<float,1,2> H {{1,0}};
Eigen::Matrix<float,2,2> Q {{5e-7,0},{0,1e-8}};
Eigen::Matrix<float,1,1> R {0.1};
Eigen::Matrix<float,2,2> P_post {{0,0},{0,0}};

SimpleKalman gyro_kalman(F, B, H, Q, R, P_post);

//------------------------------ Loop timing ----------------------------------//
// elapsedMicros looptime = 0;

//------------------------- switching controllers -----------------------------//
bool isSwitchControl = false;

//============================== [Main Setup] ==================================//
void setup(){
  //------[Initialize communications
  SPI.begin(); // IMU, fork and steer encoders
  #if USE_BT
  bt_setup(); //initialize bluetooth connection and write log header
  #endif
  #if SERIAL_DEBUG
  Serial.begin(115200); // Communication with PC through micro-USB
  /*NOTE: Wait with startup procedure untill one opens a serial monitor.
  Since the power to the microcontroller and to the rest of the bicycle
  is now not synchronus anymore.
  > First start the motors, then open a serial monitor.
  > If the motors are turned off, also turn of the microcontroller*/
  while(!Serial){}
  serial_setup();
  #endif
  
  //------[Setup INPUT pins
  pinMode(hand_switch,      INPUT);
  pinMode(a_torque,         INPUT);

  //------[Setup OUTPUT pins
  pinMode(enable_motor_enc, OUTPUT);
  pinMode(hand_led,         OUTPUT);
  pinMode(enable_fork,      OUTPUT);
  pinMode(enable_hand,      OUTPUT);
  pinMode(pwm_pin_fork,     OUTPUT);
  pinMode(pwm_pin_hand,     OUTPUT);
  pinMode(cs_hand,          OUTPUT);
  pinMode(cs_fork,          OUTPUT);

  //------[Setup PWM pins
  analogWriteResolution(PWM_RESOLUTION_BITS);
  analogWriteFrequency(pwm_pin_fork, PWM_FREQUENCY);
  analogWriteFrequency(pwm_pin_hand, PWM_FREQUENCY);

  //------[Give output pins initial values
  //Disconnect all sub-modules from the SPI bus. (pull up CS)
  digitalWrite(cs_fork, HIGH);
  digitalWrite(cs_hand, HIGH);
  
  digitalWrite(enable_motor_enc, HIGH); // Set HIGH to enable power to the encoders
  digitalWrite(enable_fork,      HIGH); // Set HIGH to enable motor
  digitalWrite(enable_hand,      HIGH); // Set HIGH to enable motor
  digitalWrite(hand_led,         HIGH);
  analogWrite(pwm_pin_fork,      INITIAL_FORK_PWM);
  analogWrite(pwm_pin_hand,      INITIAL_STEER_PWM);


  //------[Setup IMU
  #if USE_IMU
    imu_setup();
  #endif

  //------[Setup SD card
  #if USE_SD
    sd_setup();
  #endif //USE_SD

  //------[Setup Kalman filter
  gyro_kalman.init(X0,T0);

  //------[reset counting variables
  delay(1); //give time for sensors to initialize
  since_last_loop = 0;
  since_last_steer_meas = 0;
  since_last_bike_speed = 0;
  #if USE_IMU
  since_last_IMU_meas = 0;
  #endif
  wheel_counter.write(0);
}



//============================== [Main Loop] ===================================//
void loop(){
  #if USE_SD // may be moved to setup with a while loop
  if (!isOpen) open_file();
  #endif
  
  
  if (since_last_loop >= MIN_LOOP_LENGTH_MU){ //K: Sort of have a max freq? (cause that is not garanteed in this way)
    if(Serial.available()){ //check if user inputted a character in the serial
      isSwitchControl = true; //if true, switch controller
    }
    // #if SERIAL_DEBUG
    // Serial.print(since_last_loop);
    // Serial.print("\n");
    // #elif USE_BT
    // Serial1.print(since_last_loop);
    // Serial1.print("\n");
    // #endif
    // looptime = 0;
    since_last_loop = since_last_loop - MIN_LOOP_LENGTH_MU; //reset counter

    if (control_iteration_counter >= CTRL_STARTUP_ITTERATIONS) // Turn off LED when bike is ready (does effecet the transducer value)
      digitalWrite(hand_led, LOW);

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
    sbw_bike.calculate_bike_speed();
    sbw_bike.calculate_roll_states();
    sbw_bike.measure_steer_angles();
    sbw_bike.calculate_fork_rate(); //also calculates moving average of fork angle and sets it
    sbw_bike.measure_hand_torque();
    sbw_bike.measure_lat_perturbation();

    //------[Perform steering control
    if(!isSwitchControl){
      // calc_pd_errors(sbw_bike, error, derror_dt);
      // calc_pd_control(error, derror_dt, command_fork, command_hand); //add pd_control to the hand and fork torques
      // calc_friction_callibration_control(control_iteration_counter,command_fork); //used for the fork friction callibration
      // calc_directional_bias_callibration(control_iteration_counter,command_fork);
      calc_friction_callibration_control(control_iteration_counter,command_hand); //used for the steer torque callibration
      Serial.print(",");
      // Serial.print(",,,,,,,");
    } else {
      // calc_mm_control(sbw_bike, command_fork); // add model matching torque to fork torque
      // calc_sil_control(sbw_bike, command_fork, command_hand);
      // calc_mm_sil_control(sbw_bike, command_fork, command_hand);
    }

    apply_friction_compensation(command_fork);
    actuate_steer_motors(command_fork, command_hand);
    Serial.print(",");
    Serial.print(command_hand);
    Serial.print('\n');
    //------[Increase counters
    control_iteration_counter++;

    //------[Data monitoring
    #if USE_BT
    print_to_bt(sbw_bike,command_fork,command_hand);
    #endif
    #if SERIAL_DEBUG
    // print_to_serial(sbw_bike,command_fork,command_hand);
    #endif
    #if USE_SD
    print_to_SD(sbw_bike,command_fork,command_hand);
    #endif
    // #if SERIAL_DEBUG
    // Serial.print(looptime);
    // Serial.print("\n");
    // #elif USE_BT
    // Serial1.print(looptime);
    // Serial1.print("\n");
    // #endif
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
  m_hand_angle = ((float)enc_counts_hand / HAND_ENC_MAX_VAL) * 2*PI - HAND_ENC_BIAS;
  if (m_hand_angle > PI) 
    m_hand_angle = m_hand_angle - 2*PI; // CCW handlebar rotation gives 360 deg-> 310 deg. Subtract 2 pi to get 0-180 deg CCW
  // Fork
  m_fork_angle = -(((float)enc_counts_fork / FORK_ENC_MAX_VAL) * 2*PI + FORK_ENC_BIAS); //Minus sign to get minus values from fork encoder.
  if (m_fork_angle < -PI) 
    m_fork_angle = m_fork_angle + 2*PI; // CW fork rotation gives -360 deg-> -310 deg. Add 2 pi to get 0-180 deg CCW


  //------[Compensate for difference in range of motion of handelbar and fork
  /* NOTE: The fork mechanical range is +- 42 degrees. Handlebars do not 
  have a mechanical limit, however. To relay the mechanical limit of 
  the fork to the steer, software limits are set below. If you do not set
  this limits, the motor folds back.
  */
  m_hand_angle = constrain(m_hand_angle, -SOFTWARE_LIMIT, SOFTWARE_LIMIT); //K: This may cause the oscillations

  Serial.print(enc_counts_hand);
  Serial.print(",");
  Serial.print(enc_counts_fork);
  Serial.print(",");
  Serial.print(m_hand_angle,5);
  Serial.print(",");
  Serial.print(m_fork_angle,5);
  Serial.print(",");
}


//=========================== [Get handlebar torque] ===============================//
void BikeMeasurements::measure_hand_torque(){
  // float voltage = TEENSY_ANALOG_VOLTAGE * analogRead(a_torque)/HAND_TORQUE_RESOLUTION;
  // m_hand_torque = TORQUE_SLOPE*voltage + TORQUE_BIAS;
}

void BikeMeasurements::measure_lat_perturbation(){
  int lat_force_readout = analogRead(transducer_pin);

  if(control_iteration_counter > CTRL_STARTUP_ITTERATIONS){// Wait untill the led light is off
    if(haveSampledBias){ //check if already taken a measurement of the bias
      m_lean_torque = max(0,lat_force_readout-force_bias)*MEAS2LAT_FORCE*LAT_FORCE2TORQUE; //'max()' since the force transducer can only sense pulling forces
    }
    else{ //perform bias measurement
      force_bias += lat_force_readout;
      if(control_iteration_counter >= CTRL_STARTUP_ITTERATIONS + FORCE_BIAS_AVGING_WINDOW){ //
        force_bias /= (control_iteration_counter - CTRL_STARTUP_ITTERATIONS);
        haveSampledBias = true;
      }
    }
  }
  Serial.print(m_lean_torque);
  Serial.print(",");
}

//======================= [calculate steer derivatives] ============================//
void BikeMeasurements::calculate_fork_rate(){
  /*
  No moving average needed. Steer encoder sensor data is very clean (noiseless) so
  so direct numerical derivation gives good results.
  */
  m_fork_rate = calc_bckwrd_derivative(m_fork_angle, m_fork_angle_prev, m_dt_steer_meas);
  
  Serial.print(m_fork_rate,5);
  Serial.print(",");
  return;
}


//======================= [calculate roll rate and angle] ==========================//
void BikeMeasurements::calculate_roll_states(){
  // TODO: make sure that gyrox is indeed the roll rate!
  // TODO: include the IMU class into the BikeMeasurement class

  //Get gyro measurements
  Eigen::Matrix<float,3,1> omega_vec;
  
  get_IMU_data(m_dt_IMU_meas);
  omega_vec(0,0) = IMU.gyro_x_radps();
  omega_vec(1,0) = IMU.gyro_y_radps();
  omega_vec(2,0) = IMU.gyro_z_radps();
  omega_vec = -B_ROT_IMU*omega_vec; //Minus to get the correct angular positive rotation

  //having dt, change the propegation model and input model according to Sanjurjo
  F << 1, -((float)m_dt_IMU_meas)*MICRO_TO_UNIT,
       0, 1;
  B << ((float)m_dt_IMU_meas)*MICRO_TO_UNIT,
       0;

  gyro_kalman.set_F(F);
  gyro_kalman.set_B(B);
  // calculate the lean angle measurement according to Sanjuro
  calc_lean_angle_meas(omega_vec(0,0), omega_vec(1,0), omega_vec(2,0));
  
  // perform the kalman step
  Eigen::Matrix<float,1,1> u{m_omega_x_old};
  Eigen::Matrix<float,1,1> z{m_lean_angle_meas};
  gyro_kalman.next_step(u, z, (double)m_dt_IMU_meas);

  //update lean states
  m_lean_rate = omega_vec(0,0) - gyro_kalman.bias(); // [rad/s]
  m_lean_angle = gyro_kalman.phi(); // [rad]

  // store current roll rate for next itteration
  m_omega_x_old = omega_vec(0,0); // [rad/s] You need u_k-1 to calculate x_k. There is one step difference

  Serial.print(omega_vec(0,0));
  Serial.print(",");
  Serial.print(omega_vec(1,0));
  Serial.print(",");
  Serial.print(omega_vec(2,0));
  Serial.print(",");
  Serial.print(m_lean_angle_meas);
  Serial.print(",");
  Serial.print(m_lean_angle,5);
  Serial.print(",");
  Serial.print(gyro_kalman.bias());
  Serial.print(",");
  Serial.print(m_lean_rate,5);
  Serial.print(",");
}

void BikeMeasurements::calc_lean_angle_meas(float omega_x, float omega_y, float omega_z){
  /*See Sanjurjo e.a. 2019 "Roll angle estimator based on angular 
    rate measurements for bicycles" for explanation on why these 
    formulas are used.*/
  float phi_d, phi_w, W;

  // Remove bias
  omega_y -= OMEGA_Y_BIAS;
  omega_z -= OMEGA_Z_BIAS;

  // Lean angle estimate based on constant cornering (good for small lean angles)
  phi_d = std::atan((omega_z*m_bike_speed)/GRAVITY); // [rad]

  // Lean angle astimate based on zero tilt rate (good for large lean angles)
    if (/*omega_y > -1e-5 && omega_y < 1e-5
      &&*/ omega_z > -1e-5 && omega_z < 1e-5){
        omega_z += 0.001;
      }
  phi_w = std::atan(omega_y/omega_z);//sgn(omega_z)*std::asin(omega_y/std::sqrt(omega_y*omega_y + omega_z*omega_z)); // [rad]
  
  // Use the best method based on lean angle size
  W = std::exp(-m_lean_angle*m_lean_angle/PHI_METHOD_WEIGHT); // weight

  m_lean_angle_meas = W*phi_d + (1-W)*phi_w; // [rad]
}


//========================= [Calculate bicycle speed] ==========================//
void BikeMeasurements::calculate_bike_speed(){
  // TODO: bring the approprate global variables inside of the BikeMeasurement class

  /*K: NOTE Since the microcontroller operating frequency is much higher than the
  frequency at which encoder ticks pass the reading head, the difference between
  the tick count of the current and previous loop will most of the time be zero.
  To have a meaningfull value, the value of this loop and that of 
  WHEEL_COUNTS_LENGTH ago are compared.*/
  int32_t current_wheel_count = wheel_counter.read();
  int32_t previous_wheel_count = wheel_counts[wheel_counts_index];
  wheel_counts[wheel_counts_index] = current_wheel_count;

  //Update time measurement
  update_dtime(m_dt_bike_speed_meas, since_last_bike_speed);
  bike_speed_dt_sum_mu -= bike_speed_dt_array[wheel_counts_index];
  bike_speed_dt_sum_mu += m_dt_bike_speed_meas;
  bike_speed_dt_array[wheel_counts_index] = m_dt_bike_speed_meas;

  /*NOTE #rounds = count_diff/WHEEL_COUNTS_PER_REV. The count_diff is measured 
  WHEEL_COUNTS_LENGTH loops away from each other. The length of such a loop is
  stored in bike_speed_dt_sum_mu
  */ 
  float rps_wheel = ((float)((current_wheel_count + end_of_array_storage) - previous_wheel_count )) 
  / ((float)WHEEL_COUNTS_PER_REV * ((float)bike_speed_dt_sum_mu * MICRO_TO_UNIT));
  m_bike_speed = rps_wheel * 2*PI * WHEEL_RADIUS;
  
  // update index
  wheel_counts_index += 1;

  /*NOTE: To protect from an overflow (both from the wheel_counts_index and 
  the wheel_counter.read() value) both reset if wheel_counts_index 
  reaches the end of the storage register/array. To compensate for the reset
  all subsequent new-old difference calculations have to be offset by 
  end_of_round_storage */
  if (wheel_counts_index >= WHEEL_COUNTS_LENGTH){
    wheel_counts_index = 0;
    end_of_array_storage = wheel_counter.read();
    wheel_counter.write(0);
  }

  Serial.print(current_wheel_count);
  Serial.print(",");
  Serial.print(m_bike_speed);
  Serial.print(",");
  return;
} 


#if USE_PEDAL_CADANCE
//============================ [Calculate cadance] =============================//
void BikeMeasurements::calculate_pedal_cadance(){
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
  m_pedal_cadance = ((float) (current_pedal_count - previous_pedal_count)) / 
  ((float)PEDAL_COUNTS_PER_REV *(float) PEDAL_COUNTS_LENGTH * MIN_LOOP_LENGTH_S)
  * 2*PI;
    
  return;
}
#endif //USE_PEDAL_CADANCE

#if USE_IMU
void imu_setup(){
// Start the I2C bus
    Wire.begin();
    Wire.setClock(WIRE_FREQ);
    // I2C bus,  0x68 address
    IMU.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
    // TODO: the mpu9250 class has a built in digital low pass filter. 
    // default is at 184Hz. look into it if it needs to be lower.
    IMU.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G); // +- 4g
    IMU.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_250DPS); // +- 250 deg/s
    // Initialize and configure IMU
    if(!IMU.Begin()){
      #if SERIAL_DEBUG
      Serial.println("Error initializing communication with IMU");
      Serial.println("Check IMU wiring or try cycling power");
      #endif
      while(1) {}
    }
}
#endif

#if USE_SD
void sd_setup(){
  //------[Initialize SD card and file system for SDIO mode. Here: FIFO
  if(!sd.begin(SdioConfig(FIFO_SDIO))){
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
}
#endif

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

void calc_friction_callibration_control(uint64_t loop_iter, double& command){
    if(loop_iter%LOOPS_PER_SEC == 0) //increase torque every second
        itteration_step_friction++;
    if(loop_iter%LOOPS_PER_HALF_SEC == 0) //switch direction every half seccond
        steer_direction_friction *= -1;
    
    command = max(-FRICT_CAL_CTRL_BOUND, min(FRICT_CAL_CTRL_BOUND, (itteration_step_friction*FRIC_CAL_STEP_INCREASE)*steer_direction_friction)); //from the first data 0.4 seemed like a safe number to not cross for a stationairy bicycle.
    return;
}

void calc_directional_bias_callibration(uint64_t loop_iter, double& fork_command){
  if(loop_iter%LOOPS_PER_SEC == 0) //increase bias torque every second
        itteration_step_dir_bias++;

  if(loop_iter%LOOPS_PER_HALF_SEC == 0) //switch direction every half second
    steer_direction_dir_bias = !steer_direction_dir_bias;
  
  if(steer_direction_dir_bias)
    fork_command = min(2,(BASE_STEER_TORQUE_DIR_BIAS+(itteration_step_dir_bias*DIR_BIAS_INCREASE_STEP)));
  else
    fork_command = -BASE_STEER_TORQUE_DIR_BIAS;

  return;
}

void apply_friction_compensation(double& fork_command){
  fork_command += sgmd(fork_command)*FRIC_COMPENSATION_BIAS;
}

//=========================== [Calculate model matching Control] ===========================//
void calc_mm_control(BikeMeasurements& bike, double& command_fork){

  //TODO: look into if it can be maded angle controlled?(I have no clue what I ment with this...)

  //------[Calculate model matching torques
  /* NOTE: The torque is only applied to the fork, as the driver should not notice 
  that the fork is moving differently than the handlebar. Furthermore, we assume the 
  lean torque is zero, aka there is no external torque in lean direction applied
  */

  float k_phi, k_delta, k_dphi, k_ddelta, k_tphi, k_tdelta;
  calc_mm_gains(k_phi, k_delta, k_dphi, k_ddelta, k_tphi, k_tdelta, bike.get_bike_speed());

  if(bike.get_bike_speed() < 1E-3 && bike.get_bike_speed() > -1E-3){
    command_fork += 0;
  } else {
    command_fork += k_phi*bike.get_lean_angle()
                  + k_delta*bike.get_fork_angle() 
                  + k_dphi*bike.get_lean_rate() 
                  + k_ddelta*bike.get_fork_rate() 
                  // + k_tphi*bike.get_lean_torque() //we cannot measure lean torque, so we assume it is zero. 
                  + k_tdelta*bike.get_hand_torque();
  }


  Serial.print(k_phi);
  Serial.print(",");
  Serial.print(k_delta);
  Serial.print(",");
  Serial.print(k_dphi);
  Serial.print(",");
  Serial.print(k_ddelta);
  Serial.print(",");
  Serial.print(k_tphi);
  Serial.print(",");
  Serial.print(k_tdelta);
  Serial.print(",");
  Serial.print(command_fork);
  Serial.print(",");
}

void calc_mm_gains(float& k_phi, float& k_delta, float& k_dphi, float& k_ddelta, float& k_tphi, float& k_tdelta, float speed){
  k_phi = K_MM_PHI_V0;
  k_delta = K_MM_DELT_V2*(speed*speed) + K_MM_DELT_V0;
  k_dphi = K_MM_DPHI_V1*speed + K_MM_DPHI_VMIN1/speed;
  k_ddelta = K_MM_DDELT_V1*speed + K_MM_DDELT_VMIN1/speed;
  k_tphi = K_MM_TPHI_V0;
  k_tdelta = K_MM_TDELT_V0;
}

void calc_sil_control(BikeMeasurements& bike, double& command_fork, double& command_hand){
  double sil_command;

  if (bike.get_bike_speed() < V_AVERAGE)
    sil_command = K_SIL1 * (V_AVERAGE - bike.get_bike_speed())*bike.get_lean_rate();
  else
    sil_command = K_SIL2 * (bike.get_bike_speed() - V_AVERAGE)*bike.get_lean_angle();

  command_fork += sil_command;
  command_hand -= sil_command * FORK_FRICTION_COMP_RATIO;

  Serial.print(sil_command);
  Serial.print(",");
  return;
}

void calc_mm_sil_control(BikeMeasurements& bike, double& command_fork, double& command_hand){

  //TODO: look into if it can be maded angle controlled? (I have no clue what I ment with this...)

  //------[Calculate model matching torques
  /* NOTE: The torque is only applied to the fork, as the driver should not notice 
  that the fork is moving differently than the handlebar. Furthermore, we assume the 
  lean torque is zero, aka there is no external torque in lean direction applied
  */

  float k_phi, k_delta, k_dphi, k_ddelta, k_tphi, k_tdelta;
  double sil_command;
  calc_mm_gains(k_phi, k_delta, k_dphi, k_ddelta, k_tphi, k_tdelta, bike.get_bike_speed());
  
  if (bike.get_bike_speed() < V_AVERAGE)
  {
    sil_command = K_SIL1 * (V_AVERAGE - bike.get_bike_speed())*bike.get_lean_rate();
    if (bike.get_bike_speed() < 1E-3 && bike.get_bike_speed() > -1E-3){
      command_fork = sil_command;
    }else{
      command_fork =  k_phi*bike.get_lean_angle()
                    + k_delta*bike.get_fork_angle()
                    + (k_dphi + k_tdelta*(K_SIL1 * (V_AVERAGE - bike.get_bike_speed())))*bike.get_lean_rate() 
                    + k_ddelta*bike.get_fork_rate()
                    // + k_tphi*bike.get_lean_torque() //we cannot measure lean torque, so we assume it is zero. 
                    + k_tdelta*bike.get_hand_torque();
    }
  }
  else
  {
    sil_command = K_SIL2 * (bike.get_bike_speed() - V_AVERAGE)*bike.get_lean_angle();
    if (bike.get_bike_speed() < 1E-3 && bike.get_bike_speed() > -1E-3){
      command_fork = sil_command;
    }else{
      command_fork =  (k_phi + k_tdelta*(K_SIL2 * (bike.get_bike_speed() - V_AVERAGE)))*bike.get_lean_angle() 
                    + k_delta*bike.get_fork_angle() 
                    + k_dphi*bike.get_lean_rate() 
                    + k_ddelta*bike.get_fork_rate() 
                    // + k_tphi*bike.get_lean_torque() //we cannot measure lean torque, so we assume it is zero. 
                    + k_tdelta*bike.get_hand_torque();
    }
  }

  command_hand = -sil_command * FORK_FRICTION_COMP_RATIO;

  Serial.print(k_phi);
  Serial.print(",");
  Serial.print(k_delta);
  Serial.print(",");
  Serial.print(k_dphi);
  Serial.print(",");
  Serial.print(k_ddelta);
  Serial.print(",");
  Serial.print(k_tphi);
  Serial.print(",");
  Serial.print(k_tdelta);
  Serial.print(",");
  Serial.print(command_fork);
  Serial.print(",");
  Serial.print(sil_command);
  Serial.print(",");
  return;
}

//=========================== [Actuate steer motors] ===========================//
void actuate_steer_motors(double command_fork, double command_hand){
  //------[Constrain max torque rate
  /*update_dtime(dt_torque_command, since_last_torque_command);
  float command_fork_rate = (command_fork - command_fork_prev)/(dt_torque_command * MICRO_TO_UNIT);
  float command_hand_rate = (command_hand - command_hand_prev)/(dt_torque_command * MICRO_TO_UNIT);

  Serial.print(",");
  Serial.print(command_fork);
  Serial.print(',');
  Serial.print(command_hand);
  Serial.print(',');
  Serial.print(command_fork_rate);
  Serial.print(',');
  Serial.print(command_hand_rate);
  Serial.print(',');

  float dt = dt_torque_command; // may be unnecessary but should be tested, and there is currently no time for that
  if(command_fork_rate < -MAX_FORK_TORQUE_RATE || command_fork_rate > MAX_FORK_TORQUE_RATE ){
    Serial.print("here: ");d
    Serial.print(command_fork);
    Serial.print(command_fork_prev);
    Serial.print(" ");
    command_fork = command_fork_prev + sgmd(command_fork_rate)*(float)MAX_FORK_TORQUE_RATE*(dt*MICRO_TO_UNIT);
  }
  if(command_hand_rate < -MAX_HAND_TORQUE_RATE || command_hand_rate > MAX_HAND_TORQUE_RATE ){
    command_hand = command_hand_prev + sgmd(command_hand_rate)*MAX_HAND_TORQUE_RATE*(dt*MICRO_TO_UNIT);
  }
  Serial.print(command_fork);
  Serial.print(',');
  Serial.print(command_hand);
  Serial.print(',');
  */

  //------[Find the PWM command
  uint64_t pwm_command_fork = (command_fork * -842.795 + 16384); //K: Sends signal in 0-3.3V range out, which then corresonds to some (unkonwn by me) range of current/torque
  uint64_t pwm_command_hand = (command_hand * -842.795 + 16384); //  This video (https://www.youtube.com/watch?v=-TC_ccQnk-Y&list=PLmklAQtFT_ZJzWOa9O6507qA0NiSU8hzN) shows that one can set the ratio between input voltage to output current on the motor driver.
  /*NOTE
  So, pwm_resolution on the command torques is PWM_RESOLUTION BITS = 15 
  --> 0 till 32767. From the formula above: 
  > 16384 equals zero torque. Above and below 16384 is a different sign. 
  So you have a range of 0 to 16384. for torque. 
  > commanded torque is multiplied by -842.795, so commanded torque ranges 
  from 0 till 19.44. 
  From experiments we know that at a instant commanded torque of around 10
  the belt begins to slip. To prevent this: 
  -10*842.795 + 16384 < commanded torque < 10*842.795 + 16384
  7956 < commanded torque < 24812
  */
  //Constrain max torque
  // Serial.print(pwm_command_fork);
  // Serial.print(',');
  // Serial.print(pwm_command_hand);
  // Serial.print(',');
  pwm_command_fork = constrain(pwm_command_fork, FORK_PWM_MIN, FORK_PWM_MAX);
  pwm_command_hand = constrain(pwm_command_hand, HAND_PWM_MIN, HAND_PWM_MAX);
  Serial.print(pwm_command_fork);
  Serial.print(',');
  Serial.print(pwm_command_hand);
  //------[Send motor command
  analogWrite(pwm_pin_hand, pwm_command_hand);
  analogWrite(pwm_pin_fork, pwm_command_fork);

  // command_fork_prev = (float)command_fork;
  // command_hand_prev = (float)command_hand;
  return;
}



//=============================== [Read the IMU] ===============================//
#if USE_IMU
void get_IMU_data(uint32_t& dt_IMU_meas){
  // TODO: Error handling in the case the die temperature becomes to high (in general: error handling)
  //------[Read out data via I2C
  if(!IMU.Read()){
    #if SERIAL_DEBUG
    Serial.println("IMU read out error");
    #endif
  }

  //------[Time since last measurement
  update_dtime(dt_IMU_meas, since_last_IMU_meas); // time between to calls to the IMU
}
#endif

#if USE_BT
//======================== [initialize Bluetooth] ========================//
void bt_setup(){
  /*NOTE: Wait for the main and sub bluetooth modules to connect. 
  The main (this code) will send out a byte untill the sub has 
  received it and has send a response. Then the main will continue.
  */
  Serial1.begin(BT_BAUDRATE);
  while(1){
    Serial1.write(1);
    if(Serial1.available()) break;
  }

  //print header for log file.
  Serial1.print("Phi,");
  Serial1.print("Delta,");
  Serial1.print("d_Phi,");
  Serial1.print("d_Delta,");
  Serial1.print("Torque_hand");
  Serial1.print("\n");
}

//========================== [Print to Bluetooth] =========================//
void print_to_bt(BikeMeasurements& bike, double command_fork, double command_hand){
  if (control_iteration_counter % 10 == 0){ // Limit the printing rate
    Serial1.print(bike.get_lean_angle());
    Serial1.print(",");
    Serial1.print(bike.get_fork_angle());
    Serial1.print(",");
    Serial1.print(bike.get_lean_rate());
    Serial1.print(",");
    Serial1.print(bike.get_fork_rate());
    Serial1.print(",");
    Serial1.print(bike.get_hand_torque());
    Serial1.print("\n");
  }
}
#endif

#if SERIAL_DEBUG
//========================== [initialize Serial] ==========================//
void serial_setup(){
  //Print header for log file
  Serial.print("speed_ticks,");
  Serial.print("speed,");
  Serial.print("Kalman_gain,");
  Serial.print("posterior_x_phi,");
  Serial.print("omega_x,");
  Serial.print("omega_y,");
  Serial.print("omega_z,");
  Serial.print("m_lean_angle_meas,");
  Serial.print("m_lean_angle,");
  Serial.print("bias,");
  Serial.print("lean_rate,");
  Serial.print("enc_counts_hand,");
  Serial.print("enc_counts_fork,");
  Serial.print("m_hand_angle,");
  Serial.print("m_fork_angle,");
  Serial.print("steer_rate,");
  // Serial.print("k_phi,");
  // Serial.print("k_delta,");
  // Serial.print("k_dphi,");
  // Serial.print("k_ddelta,");
  // Serial.print("k_tphi,");
  // Serial.print("k_tdelta,");
  // Serial.print("command_fork,");
  Serial.print("m_lean_torque,");
  Serial.print("sil_command,");
  // Serial.print("fork_com,");
  // Serial.print("hand_com,");
  // Serial.print("fork_com_rate,");
  // Serial.print("hand_com_rate,");
  // Serial.print("post_fork_com,");
  // Serial.print("post_hand_com,");
  // Serial.print("fork_pwm,");
  // Serial.print("hand_pwm,");
  Serial.print("post_fork_pwm,");
  Serial.print("post_hand_pwm");
  Serial.print(",command_hand");

  Serial.print("\n");
}
//=========================== [Print to serial] ===========================//
void print_to_serial(BikeMeasurements& bike, double command_fork, double command_hand){
  // if (control_iteration_counter % 10 == 0){ // Limit the printing rate
    // Serial.print(bike.get_lean_angle());
    // Serial.print(",");
    // Serial.print(bike.get_fork_angle());
    // Serial.print(",");
    // Serial.print(bike.get_lean_rate());
    // Serial.print(",");
    // Serial.print(bike.get_fork_rate());
    // Serial.print(",");
    // Serial.print(bike.get_hand_torque());
    // Serial.print(",");
    // Serial.print(command_fork);
    // Serial.print(",");
    // Serial.print(command_hand);
    // Serial.print(",");
    // Serial.print(bike.get_bike_speed());
    // Serial.print("\n");
  // }
}
#endif //SERIAL_DEBUG



#if USE_SD
//============================== [Open File] ==============================//
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
  
  rb.print("fork_angle");
  rb.write(",");
  rb.print("lean_angle");
  rb.write(",");
  rb.print("fork_rate");
  rb.write(",");
  rb.print("lean_rate");
  rb.write(",");
  rb.print("hand_torque");
  rb.write(",");
  rb.print("bike_speed");
  rb.write(",");
  rb.print("command_fork");
  rb.write(",");
  rb.print("command_hand");
  rb.write("\n");

  // Write to SD card
  size_t n = rb.bytesUsed();
  rb.writeOut(n);
  logFile.flush();

  isOpen = true; // Set to TRUE such that the open file function is only called once
  return;
}



//============================= [Print to SD] =============================//
void print_to_SD(BikeMeasurements bike, float command_fork, float command_hand){
  size_t n = rb.bytesUsed();
  
  elapsedMicros dt_SD;
  dt_SD = 0;

  // Check if there is free space
  if ((logFile.curPosition() + n) > (LOG_FILE_SIZE - 100)) {
    digitalWrite(hand_led, LOW); //notify the user via LED
    isFull = true; //TODO: open new file
  }

  if (!isFull) {
    // If the file is not busy, write out one sector of data
    if (n >= 512 && !logFile.isBusy()) 
      rb.writeOut(512);

      rb.print(",");
      rb.write(bike.get_hand_angle());
      rb.print(",");
      rb.write(bike.get_fork_angle());
      rb.print(",");
      rb.write(bike.get_lean_angle());
      rb.print(",");
      rb.write(bike.get_fork_rate());
      rb.print(",");
      rb.write(bike.get_lean_rate());
      rb.print(",");
      rb.write(bike.get_hand_torque());
      rb.print(",");
      rb.write(bike.get_bike_speed());
      rb.print(",");
      rb.write(command_fork);
      rb.print(",");
      rb.write(command_hand);
      rb.println();
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
  if (control_iteration_counter % 1500 == 0){
    Serial.println("Writing to SD...");
    logFile.flush();
  } 

  #if SERIAL_DEBUG
  Serial.print("sec inside write_to_sd: ");
  Serial.print(dt_SD*MICRO_TO_UNIT);
  #endif
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


//================= [Update time between measurements of the steer angels] =================//
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
  //TODO: Make sure the iput 'dt' is always in microseconds
  /* Calculate the derivative with backward Euler. The previous value is updated 
  to the current after that values derivative is calulated. WARNING: This should 
  be the only place where the value of 'previous' is altered.
  */
  float derivative = (val_cur - val_prev)/(dt * MICRO_TO_UNIT);
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
  if (iteration > (uint64_t) 13000*LOOP_TIME_SCALING)
    return 1.0f;
  else if(iteration <= (uint64_t) 6000*LOOP_TIME_SCALING) 
    return 35.0f;
  else if (iteration <= (uint64_t) 7000*LOOP_TIME_SCALING)
    return 30.0f;
  else if (iteration <= (uint64_t) 8000*LOOP_TIME_SCALING)
    return 25.0f;
  else if (iteration <= (uint64_t) 9000*LOOP_TIME_SCALING)
    return 20.0f;
  else if (iteration <= (uint64_t) 10000*LOOP_TIME_SCALING)
    return 15.0f;
  else if (iteration <= (uint64_t) 11000*LOOP_TIME_SCALING)
    return 10.0f;
  else if (iteration <= (uint64_t) 12000*LOOP_TIME_SCALING)
    return 5.0f;
  else if (iteration <= (uint64_t) 13000*LOOP_TIME_SCALING)
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