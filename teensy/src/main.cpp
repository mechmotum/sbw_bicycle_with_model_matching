//================================= Headers ==================================//
// Note: include the "eigen.h" library first, otherewise there will be compile errors.
#include "eigen.h" //https://github.com/bolderflight/eigen
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include <Filters/IIRFilter.hpp>
#include "simpleKalman.h"
#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include "SdFat.h"
#include "RingBuf.h" //From sdFat library
#include "mpu9250.h" //https://github.com/bolderflight/MPU9250
/*TODO: include other measurement functions into the BikeMeasurements class, like the IMU*/

/* LEFT FOR DOCUMENTATION PURPOSE ONLY [transfer to more appropriate location and remove]
a_force = 20; // Analog output pin of the force transducer
a_torque = 21; // Analog output pin of the self made torque sensor (broken)
a_fork = 40; // Analog output pin of the fork motor drive (not initialized on the motor driver)
a_hand = 41; // Analog output pin of the handlebar motor drive
hand_switch = 28; // Switch installed on the handlebars
*/

// NOTE: More old but maybe still useful functions at the end of the file.


//============================== Compile modes ===============================//
#define USE_IMU 1
#define USE_BT 0
#define USE_SD 0
#define USE_PEDAL_CADANCE 0
#define SERIAL_DEBUG 1
#define TRANSDUCER_ON_STEER 1
#define TRANSDUCER_ON_SADLE 0



//================================= Classes ==================================//
class BikeMeasurements{
  /* This class measures/calculates and stores
  the states, inputs, and other necessary variables
  describing the bicycle dynamics.
  */
  private:
    //States
    float m_hand_angle;    // [rad]
    float m_fork_angle;    // [rad]
    float m_lean_angle;    // [rad]
    float m_fork_rate;     // [rad/s]
    float m_lean_rate;     // [rad/s]
    //Inputs
    float m_hand_torque;   // [Nm] Measurement of the steer torque. Calculated by measureing with a force transducer the force applied on the handlebar 
    float m_lean_torque;   // [Nm] Measurement of the lean torque. Calculated by measureing with a force transducer the force applied beneath the seat
    //Variable bicycle parameters
    float m_bike_speed;    // [m/s]
    //Other measurements
    #if USE_PEDAL_CADANCE
    float m_pedal_cadance; // [rad/s]
    #endif
    
    // Sanjurjo kalman filter variables
    float m_lean_angle_meas; // [rad]
    float m_omega_x_old;     // [rad/s]

    // Variables that will be used as data markers during testing
    float m_accel_x;         // [m/s^2]
    float m_accel_y;         // [m/s^2]

    // Variables needed for derivarion/intergration calculation
    uint32_t m_dt_bike_speed_meas; // [us] Time between two consecutive measurements of the bike speed in microseconds
    uint32_t m_dt_steer_meas; // [us] Time between two consecutive measurements of the steer angle in microseconds
    uint32_t m_dt_IMU_meas; // [us] Time between two consecutive measurements of the IMU values in microseconds
    float m_fork_angle_prev; // [rad]
    
  public:
    //--[Constructors
    BikeMeasurements(){
      //States
      m_hand_angle = 0;
      m_fork_angle = 0;
      m_lean_angle = 0;
      m_fork_rate = 0;
      m_lean_rate = 0;
      //Inputs
      m_hand_torque = 0;
      m_lean_torque = 0;
      //Variable bicycle parameters
      m_bike_speed = 0;
      #if USE_PEDAL_CADANCE
      m_pedal_cadance = 0;
      #endif

      // Sanjurjo kalman filter variables
      m_lean_angle_meas = 0;
      m_omega_x_old = 0;

      // Variables that will be used as data markers during testing
      m_accel_x = 0;
      m_accel_y = 0;

      // Variables needed for derivarion/intergration calculation
      m_dt_bike_speed_meas = 0;
      m_dt_IMU_meas = 0;
      m_dt_steer_meas = 0;
      m_fork_angle_prev = 0;
    }

    //--[Getters
    //States
    float get_hand_angle(){return m_hand_angle;}
    float get_fork_angle(){return m_fork_angle;}
    float get_lean_angle(){return m_lean_angle;}
    float get_fork_rate(){return m_fork_rate;}
    float get_lean_rate(){return m_lean_rate;}
    //Inputs
    float get_hand_torque(){return m_hand_torque;}
    float get_lean_torque(){return m_lean_torque;}
    //Variable bicycle parameters
    float get_bike_speed(){return m_bike_speed;}
    //Other measurements
    #if USE_PEDAL_CADANCE
    float get_pedal_cadance(){return m_pedal_cadance;}
    #endif
    // Variables that will be used as data markers during testing
    float get_accel_x(){return m_accel_x;}
    float get_accel_y(){return m_accel_y;}
    // Variables needed for derivarion/intergration calculation
    uint32_t get_dt_steer_meas(){return m_dt_steer_meas;}

    //--[Setters
    void set_hand_angle(float angle){m_hand_angle = angle;}
    void set_fork_angle(float angle){m_fork_angle = angle;}

    //--[Measurements or calculation of member variables
    //States
    void measure_steer_angles();
    void calculate_roll_states();
    void calculate_fork_rate();
    //Input
    void measure_hand_torque();
    void measure_lat_perturbation();
    //Variable bicycle parameters
    void calculate_bike_speed();
    //Other variables
    #if USE_PEDAL_CADANCE
    void calculate_pedal_cadance();
    #endif
    // Variables that will be used as data markers during testing
    void calculate_accelarations();
    // Sanjurjo kalman filter variables
    void calc_lean_angle_meas(float omega_x, float omega_y, float omega_z);
};



//=========================== Function declarations ===========================//
//---[Controllers
void calc_pd_control(float error, float derror_dt, double& command_fork, double& command_hand);
void calc_sil_control(BikeMeasurements& bike, double& command_fork, double& command_hand);
void calc_mm_sil_control(BikeMeasurements& bike, double& command_fork, double& command_hand);
void calc_mm_gains(float& k_phi, float& k_delta, float& k_dphi, float& k_ddelta, float& k_tphi, float& k_tdelta, float speed);

//--[Control Helper functions
void calc_pd_errors(BikeMeasurements& bike, float& error, float& derror_dt);
float relay_measured_hand_torque(float meas_force);
void reset_force_bias(uint64_t current_itteration);
void apply_friction_compensation(double& fork_command);

//--[Calculation Helper functions
float return_scaling(uint64_t iteration);
float calc_bckwrd_derivative(float val_cur, float& val_prev, uint32_t dt);
void update_dtime(uint32_t& dtime, elapsedMicros& timer);

//--[Low level Bicycle operations
void actuate_steer_motors(double command_fork, double command_hand);
uint16_t read_motor_encoder(const uint8_t cs_pin);

//--[Mode dependent functions
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
void update_IMU_data(uint32_t& dt_IMU_meas);
#endif
#if USE_SD
void sd_setup();
void open_file();
void print_to_SD(BikeMeasurements bike, float command_fork, float command_hand);
#endif

//--[Super small helper functions
template <typename T> // copied from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c || made by user79758 and stef, altered to fit current implementation. || Under https://creativecommons.org/licenses/by-sa/2.5/ (original post), and https://creativecommons.org/licenses/by-sa/4.0/ (edited post)
int sgmd(T val) { return (T(0) < val) - (val < T(0));}

template <typename T>
int sgn(T val) { return (T(0) <= val) - (val < T(0));} //altered to give 1 at zero. (so no longer sigmund function) This was required for the implementation regarding of Sanjurjo



//============================= Global Variables =============================//
//-------------------------------- Constants ---------------------------------//
//---[Quantities
// physical quantities
const float GRAVITY = 9.81;

// Conversion quantities
const float MICRO_TO_UNIT = 1e-6;

//---[Mode dependent
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

// Bluetooth
#if USE_BT
const uint32_t BT_BAUDRATE = 9600;
#endif

// IMU
#if USE_IMU
const uint32_t WIRE_FREQ = 400000; // [Hz] Frequency set by bolderflight example. It seems to work so I did not alter it.
#endif

//---[Physical bicycle objects
// Angular rate measurements
const float OMEGA_Y_BIAS = -0.02;
const float OMEGA_Z_BIAS = -0.01;

// PWM Setup
const uint8_t PWM_RESOLUTION_BITS = 15;
const float PWM_FREQUENCY = 4577.64;

// Commanded torque
const uint32_t HAND_PWM_MAX = 24812;
const uint32_t HAND_PWM_MIN = 7956;
const uint32_t FORK_PWM_MAX = 24812;
const uint32_t FORK_PWM_MIN = 7956;
const uint16_t INITIAL_FORK_PWM = 16384; //K: I think that the middle is a zero command. That 0 and 32,768 are both maximum torque but in opposite directions.
const uint16_t INITIAL_STEER_PWM = 16384;

// Motor encoders
const uint32_t ENCODER_CLK_FREQ = 225000; //clock frequency for the encoders SPI protocol
const float HAND_ENC_BIAS = 153.65 * DEG_TO_RAD + 0.0203;
const float FORK_ENC_BIAS = 100.65 * DEG_TO_RAD - 0.0564;
const float HAND_ENC_MAX_VAL = 8192.0; //ticks go from 0 to 8191. At the 8192th tick encoder_tick/HAND_ENC_MAX_VAL = 1 --> 2*pi == 0
const float FORK_ENC_MAX_VAL = 8192.0;

// Steer Angles
const float SOFTWARE_LIMIT = 42.0 * DEG_TO_RAD;

// Lateral force
float FORCE2LEAN_TORQUE = 0.95; // Height of the force sensor attachment point, measured from the ground in meters. (wheels at 4bar)
float FORCE2STEER_TORQUE = -0.32; // [m] Negative as the push is positive in the coordinate system, while pull is measured as positive. Arm on the handlebar. Line perpendicular to the line of application and the steer pin.
float TRANSDUCER_MEAS2FORCE = (1/29.689376936164084)*9.81; // [kg/-]*[N/kg]calibration has been done in [kg](independend) vs [-](dependend){no unit as it is a mapping from 0-3,3V to 0-1023}
uint16_t FORCE_BIAS_AVGING_WINDOW = 500; //Transducer seems to have a different offset every new code start. So take a sample of FORCE_BIAS_AVGING_WINDOW long, to figure out the offset.

// Pedal and wheel encoders
/*NOTE: A WHEEL_COUNTS_LENGTH of 500 gives an approximate 45 counts per calculation 
for a bike going 1 m/s (counts per rev * speed/radius). It then also calculates the 
average speed of the last 0.5 seconds. So it will have trouble with high frequency
sinosoidal translations. We assume here that the speed is always in the forward 
direction. Unfortunetely, with this size (500) the resolution is noticably lower 
for lower speeds. In short: Lower WHEEL_COUNTS_LENGTH means faster respons to speed 
change, but higher WHEEL_COUNTS_LENGTH means better resolution*/
const uint16_t WHEEL_COUNTS_LENGTH = 500; 
const uint8_t WHEEL_COUNTS_PER_REV = 192;
const float WHEEL_RADIUS = 0.3498f; //[m]
#if USE_PEDAL_CADANCE
/*NOTE: there is currently no sensor to measure the encoder ticks*/
const uint16_t PEDAL_COUNTS_LENGTH = 500;
const uint8_t PEDAL_COUNTS_PER_REV = 192;
#endif

//---[Loop timing
const uint16_t MIN_LOOP_LENGTH_MU = 10000; // target minimum loop length in microseconds. (Currently set at dt = 0.01sec as this corrosponsds with the current Kalman gains)
const float MIN_LOOP_LENGTH_S = MIN_LOOP_LENGTH_MU*MICRO_TO_UNIT; //
const float LOOP_TIME_SCALING = 1000.0F/(float)MIN_LOOP_LENGTH_MU; // The old code was written for startup time of 13 seconds having a loop time of 1 milisec. When changing the loop time, the number of itterations needed for 13 sec also has to scale appropriately
const uint16_t CTRL_STARTUP_ITTERATIONS = 13000*LOOP_TIME_SCALING; // itterations in which the steer torques are slowly scaled to unity.
uint64_t wait_itterations = CTRL_STARTUP_ITTERATIONS; //amount of itterations the code waits until measureing the bias on the force transducer. initially set to CTRL_STARTUP_ITTERATIONS, as the led messes up the sensor reading.

//---[Fork friction compensation
const float FRIC_COMPENSATION_BIAS = 0.2;

//---[Kalman filtering
const Eigen::Matrix<float,2,1> X0 {0,0}; //initial state vector
const double T0 = 0; //initial time

//---[Butterworth Filtering
const float SAMPLING_FREQ = 1/MIN_LOOP_LENGTH_S; //[Hz]
const float BUTTER_CUT_OFF = 5; //[Hz]
const float BUTTER_NATURAL_FREQ = 2 * BUTTER_CUT_OFF/SAMPLING_FREQ;
const uint8_t BUTTER_ORDER = 2;

// roll angle measurement estimation
float PHI_METHOD_WEIGHT = 0.05;

//---[Controller Gains
// PD Gains
/*NOTE: In the original implementation by georgias and simonas, the angles 
were in degree. The gains were tuned for degrees as well. Now that all 
angles are converted to radians, the gain needs to be compensated to give 
the same value. Furthermore the KP gains have been multiplied by 0.5, 
to make the control more stable. (It oscillated out of control sometimes)
>Former: Kp_old*error_deg
>New Kp_new*error_rad = Kp_old*rad2deg * error_deg*deg2rad 
                      = Kp_old*error_deg */
const float KP_F = 0.5*2.0f * RAD_TO_DEG; // Fork
const float KD_F = 0.029f * RAD_TO_DEG; // Fork
const float KP_H = 0.5*0.9f * RAD_TO_DEG; // Handlebar
const float KD_H = 0.012f * RAD_TO_DEG; // Handlebar

// Steer into lean gains (see 'Some recent developments in bicycle dynamics and control', A. L. Schwab et al., 2008)
const uint8_t K_SIL1 = 2.5; // [Ns^2/rad] gain for the steer into lean controller when below stable speed range
const float K_SIL2 = 0.7; // [Ns/rad] gain for the steer into lean controller when above stable speed range
const float V_AVERAGE = 4.3; // [m/s] value somewhere in the stable speed range. (take the average of min and max stable speed)
const float FORK_TRQ_REDUCTION_RATIO = 0.3; //The fork is free to rotate -> no friction. So it will rotate much harder.

// Model matching gains: The "_Vx" indicates that the coefficient
//  is multiplied with speed to the power of x.
const float K_MM_PHI_V0 = 0.211492437333; // lean angle
const float K_MM_DELT_V0 = 0.24058007042; // steer/fork angle
const float K_MM_DELT_V2 = -0.0864766087461; // steer/fork angle
const float K_MM_DPHI_V1 = 0.20410815964; // lean rate
const float K_MM_DPHI_VMIN1 = 1.51052073059e-13; // lean rate
const float K_MM_DDELT_V1 = 0.00383946633565; // steer/fork rate
const float K_MM_DDELT_VMIN1 = 3.31138184392e-14; // steer/fork rate
const float K_MM_TPHI_V0 = 0.00269562893980; // lean torque
const float K_MM_TDELT_V0 = 0.944830638808; // steer/hand torque


//-------------------------------- Pins --------------------------------------//
const uint8_t cs_hand = 24; // SPI Chip Select for handlebar encoder
const uint8_t cs_fork = 25; // SPI Chip Select for fork encoder

const uint8_t pwm_pin_hand = 8; // Send PWM signals to the handlebar motor
const uint8_t pwm_pin_fork = 9; // Send PWM signals to the fork motor
const uint8_t enable_hand = 29; // Turn the handlebar motor on or off
const uint8_t enable_fork = 30; // Turn the fork motor on or off
const uint8_t enable_motor_enc = 31; // HIGH to send power to the motor encoders

const uint8_t hand_led = 32; // LED installed on the handlebars

const uint8_t transducer_pin = 20; // Analog output of the force transducer

const uint8_t encdr_pin1_wheel = 2; //1 of 2 pins to read out the wheel encoder
const uint8_t encdr_pin2_wheel = 3; //1 of 2 pins to read out the wheel encoder
#if USE_PEDAL_CADANCE
const uint8_t encdr_pin1_pedal = 23; //1 of 2 pins to read out the pedal encoder
const uint8_t encdr_pin2_pedal = 22; //1 of 2 pins to read out the pedal encoder
#endif

//----------------------- Force transducer measurement --------------------------//
float force_bias = 0;
bool haveSampledBias = false;

//------------------------------ PD Control ----------------------------------//
float error_prev = 0.0f; // [rad] Variable to store the previos mismatch between handlebar and fork
uint64_t control_iteration_counter = 0; // TODO: Ensure it never overflows!

//--------------------------------- Time -------------------------------------//
elapsedMicros since_last_loop; // How long has passed since last loop execution
elapsedMicros since_last_steer_meas; // How long since last handlebar and fork measurement
elapsedMicros since_last_bike_speed; // How long since last bike speed measurement
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

//------------------------------- Filtering -----------------------------------//
//Butterworth
auto butter_filt = butter<BUTTER_ORDER>(BUTTER_NATURAL_FREQ);
//Highpass (first order, cutoff at 0.1Hz)
AH::Array<float, 2> b_coefs {0.99968594, -0.99968594};
AH::Array<float, 2> a_coefs {1.        , -0.99937188};
auto high_pass_filt = IIRFilter<2,2,float>(b_coefs,a_coefs);

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
  pinMode(transducer_pin, INPUT);

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
      Serial.read(); //such that you only go in here once
    }
    // #if SERIAL_DEBUG
    // Serial.print(since_last_loop);
    // Serial.print("\n");
    // #endif
    // looptime = 0;
    since_last_loop = since_last_loop - MIN_LOOP_LENGTH_MU; //'reset' counter

    if (control_iteration_counter >= CTRL_STARTUP_ITTERATIONS) // Turn off LED when bike is ready (does effecet the transducer value)
      digitalWrite(hand_led, LOW);
    
    //------[initialize local variables
    static BikeMeasurements sbw_bike{}; //Actually global due to static. But makes more sense to show up here.
    float error, derror_dt;
    double command_fork = 0;
    double command_hand = 0;

    //------[measure bike states and inputs
    sbw_bike.calculate_bike_speed();
    sbw_bike.calculate_roll_states();
    sbw_bike.measure_steer_angles();
    sbw_bike.calculate_fork_rate(); //also calculates moving average of fork angle and sets it
    sbw_bike.calculate_accelarations();
    sbw_bike.measure_hand_torque();
    sbw_bike.measure_lat_perturbation();

    //------[Perform steering control
    if(!isSwitchControl){
      calc_pd_errors(sbw_bike, error, derror_dt);
      calc_pd_control(error, derror_dt, command_fork, command_hand); //add pd_control to the hand and fork torques
    } else {
      calc_sil_control(sbw_bike, command_fork, command_hand);
      command_fork += relay_measured_hand_torque(sbw_bike.get_hand_torque());
      // calc_mm_sil_control(sbw_bike, command_fork, command_hand);
    }
    // apply_friction_compensation(command_fork);
    actuate_steer_motors(command_fork, command_hand);

    //------[Increase counters
    control_iteration_counter++;

    //------[Data monitoring
    #if USE_BT
    print_to_bt(sbw_bike,command_fork,command_hand);
    #endif
    #if SERIAL_DEBUG
    print_to_serial(sbw_bike,command_fork,command_hand);
    #endif
    #if USE_SD
    print_to_SD(sbw_bike,command_fork,command_hand);
    #endif
    // #if SERIAL_DEBUG
    // Serial.print(looptime);
    // Serial.print("\n");
    // #endif
  }
}



/*==================================================================================*\
 |                                 Helper Functions                                 |
\*==================================================================================*/

//=============================== [Get steer angles] ===============================//
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
}


//=========================== [Get handlebar torque] ===============================//
void BikeMeasurements::measure_hand_torque(){
  #if TRANSDUCER_ON_STEER
  int transducer_readout = analogRead(transducer_pin); //int as it is used in calculation later on, where it can get negative. 
  if(control_iteration_counter > wait_itterations){// Wait untill the led light is off
    if(haveSampledBias){ //check if already taken a measurement of the bias
      m_hand_torque = (transducer_readout-force_bias)*TRANSDUCER_MEAS2FORCE*FORCE2STEER_TORQUE;
      // Serial.print(m_hand_torque,5);
      // Serial.print(",");
      m_hand_torque = butter_filt(m_hand_torque);
      m_hand_torque = high_pass_filt(m_hand_torque);
    }
    else{ //perform bias measurement
      force_bias += transducer_readout;
      if(control_iteration_counter >= (uint16_t)(wait_itterations + FORCE_BIAS_AVGING_WINDOW)){
        force_bias /= (control_iteration_counter - wait_itterations);
        haveSampledBias = true;
      }
      // Serial.print(",");
    }
  }else{
    // Serial.print(",");
  }
  // Serial.print(transducer_readout);
  // Serial.print(",");
  #endif
}


//============================= [Get lean torque] ==================================//
void BikeMeasurements::measure_lat_perturbation(){
  #if TRANSDUCER_ON_SADLE
  int lat_force_readout = analogRead(transducer_pin);
  if(control_iteration_counter > wait_itterations){// Wait untill the led light is off
    if(haveSampledBias){ //check if already taken a measurement of the bias
      m_lean_torque = (lat_force_readout-force_bias)*TRANSDUCER_MEAS2FORCE*FORCE2LEAN_TORQUE;
    }
    else{ //perform bias measurement
      force_bias += lat_force_readout;
      if(control_iteration_counter >= (uint16_t)(wait_itterations + FORCE_BIAS_AVGING_WINDOW)){ //
        force_bias /= (control_iteration_counter - wait_itterations);
        haveSampledBias = true;
      }
    }
  }
  // Serial.print(lat_force_readout);
  // Serial.print(",");
  #endif
}


//========================= [calculate steer derivatives] ==========================//
void BikeMeasurements::calculate_fork_rate(){
  /*
  No moving average needed. Steer encoder sensor data is very clean (noiseless) so
  so direct numerical derivation gives good results.
  */
  m_fork_rate = calc_bckwrd_derivative(m_fork_angle, m_fork_angle_prev, m_dt_steer_meas);
  return;
}


//======================= [calculate roll rate and angle] ==========================//
void BikeMeasurements::calculate_roll_states(){
  //Get gyro measurements
  Eigen::Matrix<float,3,1> omega_vec;
  
  update_IMU_data(m_dt_IMU_meas);
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

  // Serial.print(omega_vec(0,0));
  // Serial.print(",");
  // Serial.print(omega_vec(1,0));
  // Serial.print(",");
  // Serial.print(omega_vec(2,0));
  // Serial.print(",");
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
  phi_w = std::atan(omega_y/omega_z); // used to better deal with singularity... but did not really work ---> sgn(omega_z)*std::asin(omega_y/std::sqrt(omega_y*omega_y + omega_z*omega_z)); // [rad]
  
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

void BikeMeasurements::calculate_accelarations(){
  Vector3f IMU_acc = {IMU.accel_x_mps2(),IMU.accel_y_mps2(),IMU.accel_z_mps2()};
  Vector3f bike_acc = B_ROT_IMU*IMU_acc;
  m_accel_x = bike_acc(0);
  m_accel_y = bike_acc(1);
  
}

//=======================-========= [IMU setup] ================================//
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


//================================= [SD setup] =================================//
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


//===================== [Calculate friction compensation] ======================//
void apply_friction_compensation(double& fork_command){
  static uint8_t pos_cntr = 0;
  static uint8_t neg_cntr = 0;
  static int8_t force_dir = 0;

  if(sgmd(fork_command) >= 0){
    if(pos_cntr > 5){
      force_dir = 1;
      neg_cntr = 0;
    }else{
      force_dir = -1;
      pos_cntr++;
    }
  }else{
    if(neg_cntr > 5){
      force_dir = -1;
      pos_cntr = 0;
    }else{
      force_dir = 1;
      neg_cntr++;
    }
  }

  fork_command += force_dir*FRIC_COMPENSATION_BIAS;
}


//======================== [Relay measured hand torque] ========================//
float relay_measured_hand_torque(float meas_force){
  float force_relayed = 0;
  static uint8_t in_bounds_cntr = 0;
  static uint8_t out_bounds_cntr = 5; //start out of bounds

  if(-0.5<meas_force && meas_force<0.5){      // If in bounds
    if(in_bounds_cntr >= 50){                 //   for at least 50 loop cycles
      out_bounds_cntr = 0;                    //     then ignore the measured hand torque --> most likely noise
    }else{
      in_bounds_cntr++;
      force_relayed = meas_force;             // else, relay the measured torque --> most likely a passing through bounds
    }
  }else{                                      // If out of bounds
    if(out_bounds_cntr >= 5){                 //   for longer than 5 cycles (aka not a spike of noise that became out of bounds)
      force_relayed = meas_force;             //      relay the measured torque
      in_bounds_cntr = 0;
    }else{
      out_bounds_cntr++;                      // else, ignore measured hand torque --> most likely still noise.
    }
  }
  return force_relayed;
}


//=========================== [Calculate Sil control] ==========================//
void calc_sil_control(BikeMeasurements& bike, double& command_fork, double& command_hand){
  double sil_command;

  if (bike.get_bike_speed() < V_AVERAGE)
    sil_command = K_SIL1 * (V_AVERAGE - bike.get_bike_speed())*bike.get_lean_rate();
  else
    sil_command = K_SIL2 * (bike.get_bike_speed() - V_AVERAGE)*bike.get_lean_angle();

  command_fork += sil_command; //Different signs as the motor shafts are facing opposite directions, but are controlled the same.
  // command_hand -= sil_command * FORK_TRQ_REDUCTION_RATIO; 

  Serial.print(sil_command,5);
  return;
}


//========================= [Calculate Sil+MM control] =========================//
void calc_mm_sil_control(BikeMeasurements& bike, double& command_fork, double& command_hand){
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
    
    if (bike.get_bike_speed() < 1E-3 && bike.get_bike_speed() > -1E-3){ //Will get singularity in the MM gains if speed is close to zero.
      command_fork = sil_command;
    }else{
      command_fork =  k_phi*bike.get_lean_angle()
                    + k_delta*bike.get_fork_angle()
                    + (k_dphi + k_tdelta*(K_SIL1 * (V_AVERAGE - bike.get_bike_speed())))*bike.get_lean_rate() 
                    + k_ddelta*bike.get_fork_rate()
                    + k_tphi*bike.get_lean_torque()
                    + k_tdelta*bike.get_hand_torque();
    }
  }
  else
  {
    sil_command = K_SIL2 * (bike.get_bike_speed() - V_AVERAGE)*bike.get_lean_angle();
    
    if (bike.get_bike_speed() < 1E-3 && bike.get_bike_speed() > -1E-3){ //Will get singularity in the MM gains if speed is close to zero.
      command_fork = sil_command;
    }else{
      command_fork =  (k_phi + k_tdelta*(K_SIL2 * (bike.get_bike_speed() - V_AVERAGE)))*bike.get_lean_angle() 
                    + k_delta*bike.get_fork_angle() 
                    + k_dphi*bike.get_lean_rate() 
                    + k_ddelta*bike.get_fork_rate() 
                    + k_tphi*bike.get_lean_torque()
                    + k_tdelta*bike.get_hand_torque();
    }
  }

  // command_hand = -sil_command * FORK_TRQ_REDUCTION_RATIO; //Different signs as the motor shafts are facing opposite directions, but are controlled the same.
  return;
}


//========================= [Calculate MM gains] =========================//
void calc_mm_gains(float& k_phi, float& k_delta, float& k_dphi, float& k_ddelta, float& k_tphi, float& k_tdelta, float speed){
  k_phi = K_MM_PHI_V0;
  k_delta = K_MM_DELT_V2*(speed*speed) + K_MM_DELT_V0;
  k_dphi = K_MM_DPHI_V1*speed + K_MM_DPHI_VMIN1/speed;
  k_ddelta = K_MM_DDELT_V1*speed + K_MM_DDELT_VMIN1/speed;
  k_tphi = K_MM_TPHI_V0;
  k_tdelta = K_MM_TDELT_V0;
}


//=========================== [Actuate steer motors] ===========================//
void actuate_steer_motors(double command_fork, double command_hand){
  /*NOTE: So, pwm_resolution on the command torques is 
  PWM_RESOLUTION BITS = 15 --> 0 till 32767. From the formula below
   > 16384 equals zero torque. Above and below 16384 is a different sign. 
  So you have a range of 0 to 16384. 
   > commanded torque is multiplied by -842.795, so commanded torque ranges 
  from 0 till 19.44. 

  From experiments we know that at a instant commanded torque of around 10
  the belt begins to slip. To prevent this: 
  -10*842.795 + 16384 < commanded torque < 10*842.795 + 16384
  `              7956 < commanded torque < 24812                            */
  //------[Calculate the PWM command
  uint64_t pwm_command_fork = (command_fork * -842.795 + 16384); //K: Sends signal in 0-3.3V range out, which then corresonds to some (unkonwn by me) range of current/torque
  uint64_t pwm_command_hand = (command_hand * -842.795 + 16384); //  This video (https://www.youtube.com/watch?v=-TC_ccQnk-Y&list=PLmklAQtFT_ZJzWOa9O6507qA0NiSU8hzN) shows that one can set the ratio between input voltage to output current on the motor driver.
  
  //Constrain max torque
  pwm_command_fork = constrain(pwm_command_fork, FORK_PWM_MIN, FORK_PWM_MAX); //Just realised it would have been easier to constrain the command, but now this is already in place
  pwm_command_hand = constrain(pwm_command_hand, HAND_PWM_MIN, HAND_PWM_MAX);
  
  //------[Send motor command
  analogWrite(pwm_pin_hand, pwm_command_hand);
  analogWrite(pwm_pin_fork, pwm_command_fork);
  return;
}


//=============================== [Read the IMU] ===============================//
#if USE_IMU
void update_IMU_data(uint32_t& dt_IMU_meas){
  // TODO: error handling
  //------[Read out data via I2C
  if(!IMU.Read()){ //Calling IMU.Read() will update the measurements. (even if called inside an if statement)
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
  // Serial.print("omega_x,");
  // Serial.print("omega_y,");
  // Serial.print("omega_z,");
  // Serial.print("raw_hand_torque,");
  // Serial.print("transducer_byte,");
  Serial.print("sil_command,");
  Serial.print("speed,");
  Serial.print("lean_angle,");
  Serial.print("lean_rate,");
  Serial.print("hand_angle,");
  Serial.print("fork_angle,");
  Serial.print("fork_rate,");
  Serial.print("lean_torque,");
  Serial.print("hand_torque,");
  Serial.print("x_acceleration,");
  Serial.print("y_acceleration,");
  Serial.print("command_fork,");
  Serial.print("command_hand");
  Serial.print("\n");
}


//=========================== [Print to serial] ===========================//
void print_to_serial(BikeMeasurements& bike, double command_fork, double command_hand){
  // Serial.print(sil_command);
  Serial.print(",");
  Serial.print(bike.get_bike_speed(),5);
  Serial.print(",");
  Serial.print(bike.get_lean_angle(),5);
  Serial.print(",");
  Serial.print(bike.get_lean_rate(),5);
  Serial.print(",");
  Serial.print(bike.get_hand_angle(),5);
  Serial.print(",");
  Serial.print(bike.get_fork_angle(),5);
  Serial.print(",");
  Serial.print(bike.get_fork_rate(),5);
  Serial.print(",");
  Serial.print(bike.get_lean_torque(),5);
  Serial.print(",");
  Serial.print(bike.get_hand_torque(),5);
  Serial.print(",");
  Serial.print(bike.get_accel_x(),5);
  Serial.print(",");
  Serial.print(bike.get_accel_y(),5);
  Serial.print(",");
  Serial.print(command_fork,5);
  Serial.print(",");
  Serial.print(command_hand,5);
  Serial.print("\n");
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


//========= [Update time between measurements of the steer angels] ========//
void update_dtime(uint32_t& dtime, elapsedMicros& timer){
  // TODO: make 'dtime' and 'timer' linked to each other. As in dtime and 
  // timer are a pair. If not, either timer is reset by another dtime 
  // messing up this dtime. Or dtime is set by the wrong timer.
  // maybe make it a class on its own?
  dtime = timer;
  timer = 0;
  return;
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












/*==================================================================================*\
 |                                     Appendix                                     |
\*==================================================================================*/
// These are old functions no longer used, but might be used ones or twice in the 
// future.

//============================ [Reset force transducer bias] ============================//
/*
Function enabling the resampling of the force transducer bias
//reset (and afterwards remeasure) the bias of the force transducer.

void reset_force_bias(uint64_t current_itteration){
  force_bias = 0;
  haveSampledBias = false;
  wait_itterations = current_itteration;
  return;  
}
*/

//========================== [Moving Average calculation] ==========================//
/*
const uint8_t MVING_AVG_SMPL_LEN = 10;

float moving_avg(float new_value){
  // TODO: At this point the moving average will break if it is used 
  // for two separate signals. Make the function generic
  static float avg_sum = 0;
  static uint8_t avg_idx = 0;
  static float avg_array[MVING_AVG_SMPL_LEN] = {0};

  avg_sum = avg_sum - avg_array[avg_idx];
  avg_array[avg_idx] = new_value;
  avg_sum = avg_sum + new_value;
  avg_idx = (avg_idx+1) % MVING_AVG_SMPL_LEN;
  return (float) (avg_sum / MVING_AVG_SMPL_LEN);
}
*/


//============================== [Switch debouncing] ===============================//
/*
pinMode(hand_switch,      INPUT); //For this to work electrically (circuit wise) the LED has to be turned on!

uint8_t hand_switch_value = 0;
uint8_t hand_switch_array[10] = {0};
uint8_t hand_switch_state = 0;
uint8_t hand_switch_state_prev = 0;
// Read the switch state
hand_switch_state_prev = hand_switch_state;
hand_switch_value = digitalRead(hand_switch);
hand_switch_state = check_switch(
  hand_switch_value, hand_switch_array, sizeof(hand_switch_array)/sizeof(hand_switch_array[0])
);

//-------------------------------- Check Switch --------------------------------//
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
*/


//======================= [Calculate model matching Control] =======================//
/*
void calc_mm_control(BikeMeasurements& bike, double& command_fork){
  //------[Calculate model matching torques
  // NOTE: The torque is only applied to the fork, as the driver should not notice 
  // that the fork is moving differently than the handlebar. Furthermore, we assume the 
  // lean torque is zero, aka there is no external torque in lean direction applied
  

  float k_phi, k_delta, k_dphi, k_ddelta, k_tphi, k_tdelta;
  calc_mm_gains(k_phi, k_delta, k_dphi, k_ddelta, k_tphi, k_tdelta, bike.get_bike_speed());

  if(bike.get_bike_speed() < 1E-3 && bike.get_bike_speed() > -1E-3){ //some mm gains become inf for zero speeds
    command_fork += 0;
  } else {
    command_fork += k_phi*bike.get_lean_angle()
                  + k_delta*bike.get_fork_angle() 
                  + k_dphi*bike.get_lean_rate() 
                  + k_ddelta*bike.get_fork_rate() 
                  + k_tphi*bike.get_lean_torque()
                  + k_tdelta*bike.get_hand_torque();
  }
}
*/


//======================= [Friction compensation Measurement] ======================//
/*
// The steer and fork experience friction. So a tiny commanded torque, does not move
// the steer or fork, while in theory it should. Thus the friction is compensated.
// To measure the amount of compensation necessary, give commands alternating in sign 
// and to the increasing in value. See at which command the steer/fork starts to move.
const uint16_t LOOPS_PER_SEC = (1E6/MIN_LOOP_LENGTH_MU);
const uint16_t LOOPS_PER_HALF_SEC = LOOPS_PER_SEC/2;
const float FRICT_CAL_CTRL_BOUND = 6;
const float FRIC_CAL_STEP_INCREASE = 0.1;

int itteration_step_friction = 0;
int8_t steer_direction_friction = 1;

void calc_friction_callibration_control(uint64_t loop_iter, double& command){
    if(loop_iter%LOOPS_PER_SEC == 0) //increase torque every second
        itteration_step_friction++;
    if(loop_iter%LOOPS_PER_HALF_SEC == 0) //switch direction every half seccond
        steer_direction_friction *= -1;
    
    command = max(-FRICT_CAL_CTRL_BOUND, min(FRICT_CAL_CTRL_BOUND, (itteration_step_friction*FRIC_CAL_STEP_INCREASE)*steer_direction_friction)); //from the first data 0.4 seemed like a safe number to not cross for a stationairy bicycle.
    return;
}
*/


//===================== [Steer torque callibration Measurement] ====================//
/*
// function that increases its torque in steps, allowing to measure the relation between
// commanded torque and actual measured torque.
const uint16_t LOOPS_PER_SEC = (1E6/MIN_LOOP_LENGTH_MU);
const float STEER_TORQUE_CAL_STEP_INCREASE = 0.1;
const float STEER_TRQ_CAL_CTRL_BOUND = 6;
const bool RIGHT = true;
const bool LEFT = false;

int itteration_step_trq_cal = 0;

void one_sided_steer_torque_call_control(uint64_t loop_iter, double& hand_command, const bool direction){
  if(loop_iter%(15*LOOPS_PER_SEC) == 0) //increase bias torque every second
        itteration_step_trq_cal++;

  if(direction)
    hand_command = -min(STEER_TRQ_CAL_CTRL_BOUND,itteration_step_trq_cal*STEER_TORQUE_CAL_STEP_INCREASE);
  else
    hand_command = min(STEER_TRQ_CAL_CTRL_BOUND,itteration_step_trq_cal*STEER_TORQUE_CAL_STEP_INCREASE);
  return;
}
*/


//====================== [Torque measurement through voltage] ======================//
/*
// Voltage given by the motor driver (a_hand pin) that indicates the amount of current. 
// An electrical circuit is necessary to get from +/- 4V to 0-3.3V

const float TEENSY_ANALOG_VOLTAGE = 3.3;
const uint16_t HAND_TORQUE_RESOLUTION = 1023;
const float TORQUE_SLOPE = 15.610654072386788; // Obtained from experimental measurements. See data_analysis/steer_torque_current_meas_callibration
const float TORQUE_BIAS  = -26.079960214259234;
const float CMD2VOLT_SLOPE = -0.09472095747953171; // Obtained from experimental measurements. See data_analysis/steer_torque_current_meas_callibration
const float CMD2VOLT_BIAS = 1.6679204455723928;

const uint8_t a_hand = 41;
// --Inside BikeMeasurements Class--
float m_hand_mtr_vlt = 0;
float get_hand_mtr_vlt(){return m_hand_mtr_vlt;}

pinMode(a_hand,         INPUT);

void BikeMeasurements::measure_hand_torque(){
  m_hand_mtr_vlt = TEENSY_ANALOG_VOLTAGE * analogRead(a_hand)/HAND_TORQUE_RESOLUTION;
  m_hand_torque = TORQUE_SLOPE*m_hand_mtr_vlt + TORQUE_BIAS;
}

void retreive_command_from_voltage(BikeMeasurements& bike, double command_fork){
  command_fork = (bike.get_hand_mtr_vlt()-CMD2VOLT_BIAS)/CMD2VOLT_SLOPE;
  return command_fork
}
*/


//========================= [Limit rate of command torque]  ========================//
/*
const int8_t MAX_FORK_TORQUE_RATE = 20;
const int8_t MAX_HAND_TORQUE_RATE = 5;

uint32_t dt_torque_command = 0;
float command_fork_prev = 0;
float command_hand_prev = 0;

elapsedMicros since_last_torque_command; // How long since last torque command

 //------[Constrain max torque rate
  update_dtime(dt_torque_command, since_last_torque_command);
  float command_fork_rate = (command_fork - command_fork_prev)/(dt_torque_command * MICRO_TO_UNIT);
  float command_hand_rate = (command_hand - command_hand_prev)/(dt_torque_command * MICRO_TO_UNIT);

  float dt = dt_torque_command; // may be unnecessary but should be tested, and there is currently no time for that
  if(command_fork_rate < -MAX_FORK_TORQUE_RATE || command_fork_rate > MAX_FORK_TORQUE_RATE ){
    command_fork = command_fork_prev + sgmd(command_fork_rate)*(float)MAX_FORK_TORQUE_RATE*(dt*MICRO_TO_UNIT);
  }
  if(command_hand_rate < -MAX_HAND_TORQUE_RATE || command_hand_rate > MAX_HAND_TORQUE_RATE ){
    command_hand = command_hand_prev + sgmd(command_hand_rate)*MAX_HAND_TORQUE_RATE*(dt*MICRO_TO_UNIT);
  }

  command_fork_prev = command_fork;
  command_hand_prev = command_hand;
*/