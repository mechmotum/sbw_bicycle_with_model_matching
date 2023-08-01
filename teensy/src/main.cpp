#include <Arduino.h>
#include <SPI.h> 
#include <Encoder.h>
#include "SdFat.h"
#include "RingBuf.h"
#include "mpu9250.h"
// #include "RF24.h"
// #include "mpu9250.h" // https://github.com/bolderflight/MPU9250

//=============================== Definitions ================================//
#define USE_IMU 0
#define USE_SD 0
// #define USE_RADIO 0
// #define USE_BT 1
#define USE_ANALOG 0
#define USE_ENCODERS 0
#define SERIAL_DEBUG 0

//=========================== Function definitions ===========================//
// void setup();
// void loop();
void haptics();
float return_scaling(uint64_t iteration);
//float moving_avg(float new_value);
uint8_t checkSwitch(uint8_t curr_value, uint8_t *val_array, uint8_t array_size);
// #if USE_BT
//   void bt_read();
//   void bt_parse();
// #endif
#if USE_SD
  void openFile();
#endif

//=========================== Global Variables ===============================//
//------------------------------- Minimal ------------------------------------//
// Pins
const uint8_t cs_hand = 24; // SPI Chip Select for handlebar encoder
const uint8_t cs_fork = 25; // SPI Chip Select for fork encoder
const uint8_t pwm_pin_hand = 8; // Send PWM signals to the handlebar motor
const uint8_t pwm_pin_fork = 9; // Send PWM signals to the fork motor
const uint8_t enable_hand = 29; // Turn the handlebar motor on or off
const uint8_t enable_fork = 30; // Turn the fork motor on or off
const uint8_t enable_encoder = 31; // HIGH to send power to the encoders
const uint8_t hand_led = 32; // LED installed on the handlebars
const uint8_t hand_switch = 28; // Switch installed on the handlebars
// Haptics
float error_prev = 0.0f;
uint32_t error_time_prev = 0.0f;
uint64_t haptics_iteration_counter = 0; // Ensure it never overflows
uint64_t mpc_iteration_counter = 8000; // 5 seconds of ramping, instead of 13
// Gains
float Kp_f = 2.0f;
float Kd_f = 0.029f;
float Kp_h = 0.9f;
float Kd_h = 0.012f;
// Steering rate calculation
//const uint8_t avg_filter_size = 10;
//uint8_t avg_index = 0;
//float avg_sum = 0.0f;
//float avg_array[avg_filter_size] = {0};
//float angle_prev = 0.0f;
// Time
elapsedMicros sinceLast; // How long has passed since last loop execution
// Switch debouncing
uint8_t hand_switch_value = 0;
uint8_t hand_switch_array[10] = {0};
uint8_t hand_switch_state = 0;
uint8_t hand_switch_state_prev = 0;
//------------------------------- Analog -------------------------------------//
#if USE_ANALOG
  const uint8_t a_force = 20; // Analog output pin of the force transducer
  const uint8_t a_torque = 21; // Analog output pin of the torque sensor
  const uint8_t a_fork = 40; // Analog output pin of the fork motor drive
  const uint8_t a_hand = 41; // Analog output pin of the handlebar motor drive
  float val_fork = 0.0f;
  float val_hand = 0.0f;
  float val_torque = 0.0f;
  float val_force = 0.0f;
  float a_torque_fork = 0.0f;
  float a_torque_hand = 0.0f;
  float analog_torque_sensor_Nm = 0.0f;
  float analog_force_transducer_N = 0.0f;
#endif
//------------------- Wheel Speed and Cadence Encoders -----------------------//
#if USE_ENCODERS
  Encoder wheel_counter(2, 3); // Rear wheel speed encoder
  const uint32_t WHEEL_COUNTS_LENGTH = 200;
  int32_t wheel_counts[WHEEL_COUNTS_LENGTH] = {0};
  uint32_t wheel_counts_index = 0;
  Encoder pedal_counter(23, 22); // Pedal cadence encoder
  const uint32_t PEDAL_COUNTS_LENGTH = 500;
  int32_t pedal_counts[PEDAL_COUNTS_LENGTH] = {0};
  uint32_t pedal_counts_index = 0;
#endif
//--------------------------------- IMU --------------------------------------//
#if USE_IMU
  const uint8_t cs_imu = 10; // SPI Chip Select for MPU9250
  bfs::Mpu9250 IMU(&SPI, cs_imu); // MPU9250 object
#endif
//-------------------------------- Radio -------------------------------------//
// #if USE_RADIO
//   const uint8_t cs_radio = 38; // SPI Chip Select for nRF24 radio
//   const uint8_t radio_enable = 37; // Make HIGH to enable the radio
//   RF24 radio(radio_enable, cs_radio, 1000000); // 1MHz SPI frequency
//   uint8_t radioAddresses[][6] = {"1Node", "2Node"};
//   uint8_t thisRadio = 0; // Set to 1 on the MPC radio
//   double messageRecv = 0.0; // Received message. Can be any type
//   double radio_message = 0.0; // Decoded message. Needs to be double
// #endif
//------------------------------ Bluetooth -----------------------------------//
// #if USE_BT
//   const byte bt_message_length = 8; // Length of the message
//   char bt_message_string[bt_message_length]; // Message array
//   double bt_message_double = 0.0; // Message
//   bool bt_message_new = false;
// #endif
//--------------------------- SD Card Logging --------------------------------//
#if USE_SD
  // The maximum expected legth of one sample of data. In bytes
  const uint64_t message_length = 100;
  // Log file should hold 10 minutes of data sampled at 1kHz
  const uint64_t log_file_size = message_length*1000*600;
  // Buffer should hold 2 seconds of data sampled at 1kHz
  const uint64_t ring_buf_capacity = message_length*2*1000;
  SdExFat sd;
  ExFile root; // Used to count the number of files
  ExFile countFile; // Used to count the number of files
  ExFile logFile; // Used for logging
  String fileName = "SbW_log_";
  String fileExt = ".csv";
  RingBuf<ExFile, ring_buf_capacity> rb; // Set up the buffer
  int fileCount = 0; // Number of files already on the SD
  bool isOpen = false; // Is file already open?
  bool isFull = false; // Is the file full?
#endif

//============================== Main Setup ==================================//
void setup(){
  // Initialize communications
  SPI.begin();        // IMU, encoders
  Serial.begin(9600); // Communication with PC through micro-USB

  // Setup OUTPUT pins
  pinMode(cs_hand, OUTPUT);
  digitalWrite(cs_hand, HIGH);

  pinMode(cs_fork, OUTPUT);
  digitalWrite(cs_fork, HIGH);

  pinMode(enable_encoder, OUTPUT);
  digitalWrite(enable_encoder, HIGH); // HIGH to enable power to the encoders

  pinMode(hand_led, OUTPUT);

  pinMode(enable_fork, OUTPUT);
  digitalWrite(enable_fork, HIGH); // Set HIGH to enable motor

  pinMode(enable_hand, OUTPUT);
  digitalWrite(enable_hand, HIGH); // Set HIGH to enable motor


  // Setup PWM pins
  analogWriteResolution(15);

  pinMode(pwm_pin_fork, OUTPUT);
  analogWriteFrequency(pwm_pin_fork, 4577.64);
  analogWrite(pwm_pin_fork, 16384);

  pinMode(pwm_pin_hand, OUTPUT);
  analogWriteFrequency(pwm_pin_hand, 4577.64);
  analogWrite(pwm_pin_hand, 16384);


  // Setup INPUT pins
  pinMode(hand_switch, INPUT); // If switch is HIGH - MPC is on


  // Setup optional features
  // #if USE_BT
  //   Serial1.begin(115200); // Bluetooth module
  // #endif
  #if USE_IMU
    pinMode(cs_imu, OUTPUT);
    digitalWrite(cs_imu, HIGH);

    // Initialize IMU
    if(!IMU.Begin()){
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
    }
    IMU.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G); // +- 4g
    IMU.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_250DPS); // +- 250 deg/s
  #endif

  #if USE_SD
    // Setup SD card
    if(!sd.begin(SdioConfig(FIFO_SDIO))){
      Serial.println("Please check SD card!");
      // Short-short error code (..)
      while(1) {
        digitalWrite(hand_led, HIGH);
        delay(500);
        digitalWrite(hand_led, LOW);
        delay(500);
      }
    }

    // Count files on SD card
    root.open("/");
    while (countFile.openNext(&root, O_RDONLY)) {
      if (!countFile.isHidden()) fileCount++;
      countFile.close();
    }
  #endif
  
  #if USE_RADIO
    SPI1.begin();
    SPI1.setMISO(39);
    if (!radio.begin(&SPI1)) {
      Serial.println("Failed to initialise the radio");
      // Long-short error code (-.)
      while (1) {
        digitalWrite(hand_led, HIGH);
        delay(1000);
        digitalWrite(hand_led, LOW);
        delay(500);
        digitalWrite(hand_led, HIGH);
        delay(500);
        digitalWrite(hand_led, LOW);
        delay(500);
      }
    }
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_2MBPS);
    radio.setChannel(46);
    radio.setPayloadSize(32);
    radio.openWritingPipe(radioAddresses[thisRadio]);
    radio.openReadingPipe(1, radioAddresses[!thisRadio]);
    radio.setAutoAck(false);
    radio.setCRCLength(RF24_CRC_16);
    radio.startListening();
  #endif

  delay(1);
  sinceLast = 0;
}

//============================== Main Loop ===================================//
void loop(){
  #if USE_SD
    if (!isOpen) openFile();
  #endif

  if (sinceLast >= 1000){
    sinceLast = sinceLast - 1000; // Reset the counter

    // #if USE_RADIO
    //   // Currently set up for debugging
    //   // TODO: adjust for normal use
    //   if (radio.available()) {
    //     float payloadRecv;
    //     radio.read(&payloadRecv, sizeof(payloadRecv));
    //     Serial.print(haptics_iteration_counter);
    //     Serial.write(',');
    //     Serial.print(sinceLast);
    //     Serial.write(',');
    //     Serial.println(payloadRecv);
    //     radio_message = payloadRecv;
    //   }
    // #endif

    // #if USE_BT
    //   // Read Bluetooth, see if there's a new message
    //   bt_read();
    //   bt_parse();
    // #endif

    // Turn on LED when bike is ready
    if (haptics_iteration_counter >= 13000) 
      digitalWrite(hand_led, HIGH);

    // // Read the switch state
    // hand_switch_state_prev = hand_switch_state;
    // hand_switch_value = digitalRead(hand_switch);
    // hand_switch_state = checkSwitch(hand_switch_value, 
    //                                 hand_switch_array,
    //                                 sizeof(hand_switch_array)/sizeof(hand_switch_array[0])
    //                                 );

    // Run PID (+MPC if switch is 1)
    haptics();
  }
}

//============================== Haptics =====================================//
void haptics(){
  //----------------------- Increase counters --------------------------------//
  // if (hand_switch_state == 1 && hand_switch_state_prev == 0) 
  //   mpc_iteration_counter = 8000;
  // else if (hand_switch_state == 1) 
  //   mpc_iteration_counter += 1;
  haptics_iteration_counter += 1;

  //---------------- SPI communication with Handlebar encoder ----------------//
  #if USE_IMU
    digitalWrite(cs_imu, HIGH); // HIGH to disable IMU communication
  #endif
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
  // First AND bit operator is used bit mask 0111 1111 1111 1111 
  // the 16bit is set to zero.
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

  // Encoder counts to degrees. substracting -99.2 deg to align fork with 
  // handlebars and - to get minus values from fork encoder.
  // 100.65
  float angle_fork = -(float)fork_dac * 360.0f / 8191.0f - 100.65f; 
  if (angle_fork < -180.0f) // CW fork rotation gives -360 deg-> -310 deg
    angle_fork = angle_fork + 360.0f; // Add 360 to get 0-180 deg CCW


  //---------------------- Calculate error derivative ------------------------//
  // S: Set the very first handlebar and fork encoder values to 0
  // D: setting handlebar and fork encoder first values to 0, we do that because first values seem to be floating values
  // K: most likely due to encoder just getting power, and still initializing. A delay in the setup might fix this
  if (haptics_iteration_counter < 10){ 
    angle_hand = 0.0f;
    angle_fork = 0.0f;
  }
  // Calculation of dT in seconds
  uint32_t error_time_curr = micros();
  float error_time_diff = ((float) (error_time_curr - error_time_prev)) 
    / 1000000.0f;
  error_time_prev = error_time_curr;
  // Calculation of dError
  float error_curr = (angle_hand - angle_fork);
  float error = (error_curr - error_prev) / error_time_diff;
  error_prev = error_curr;


  //----------------------- Calculate steering rate --------------------------//
  //float angle_diff = angle_fork - angle_prev;
  //float angle_rate = (float) (angle_diff / error_time_diff);
  //float filtered_angle_rate = moving_avg(angle_rate);
  //angle_prev = angle_fork;
  
  //---------------------- Calculate PID fork torque -------------------------//
  double command_fork = (Kp_f*(angle_hand - angle_fork) + Kd_f*error);
  command_fork = command_fork / return_scaling(haptics_iteration_counter);
  // Add MPC if the swith is 1
  // #if USE_RADIO
  //   if (haptics_iteration_counter >= 13000 && hand_switch_state == 1) {
  //     command_fork = command_fork + 
  //       radio_message / return_scaling(mpc_iteration_counter);
  //   }	
  // #endif
  // #if USE_BT
  //   if (haptics_iteration_counter >= 13000 && hand_switch_state == 1) {
  //     command_fork = command_fork + 
  //       bt_message_double / return_scaling(mpc_iteration_counter);
  //   }
  // #endif
  
  //---------------------- Find the fork PWM command -------------------------//
  uint64_t pwm_command_fork = (command_fork * -842.795 + 16384);
  pwm_command_fork = constrain(pwm_command_fork, 0, 32768);
  analogWrite(pwm_pin_fork, pwm_command_fork);


  //--------------------- Calculate PID handlebar torque ---------------------//
  double command_hand = (Kp_h*(angle_hand - angle_fork) + Kd_h*error);
  command_hand = command_hand / return_scaling(haptics_iteration_counter);
  // Add MPC if the swith is 1
  // Sign of message needs to be flipped to ensure both motors rotate
  // the same way and according to Whipple-Carvallo
  // #if USE_RADIO
  //   if (haptics_iteration_counter >= 13000 && hand_switch_state == 1) {
  //     command_hand = command_hand - 
  //       radio_message / return_scaling(mpc_iteration_counter);
  //   }
  // #endif
  // #if USE_BT
  //   if (haptics_iteration_counter >= 13000 && hand_switch_state == 1) {
  //     command_hand = command_hand - 
  //       bt_message_double / return_scaling(mpc_iteration_counter);
  //   }
  // #endif
  
  //-------------------- Find the handlebar PWM command ----------------------//
  uint64_t pwm_command_hand = (command_hand * -842.795 + 16384); //K: magic numbers, what do they mean :s
  pwm_command_hand = constrain(pwm_command_hand, 0, 32768);
  analogWrite(pwm_pin_hand, pwm_command_hand);


  #if USE_ENCODERS
  //------------------------ Calculate bicycle speed -------------------------//
    int32_t current_wheel_count = wheel_counter.read();
    int32_t previous_wheel_count = wheel_counts[wheel_counts_index];
    wheel_counts[wheel_counts_index] = current_wheel_count;
    wheel_counts_index += 1;
    if (wheel_counts_index >= WHEEL_COUNTS_LENGTH) wheel_counts_index = 0;
    // There are 192 counts/revolution, the radius of the wheel is 3.6m
    float rps_wheel = ((float) (current_wheel_count - previous_wheel_count)) 
      / 192.0f * 1000.0f / ((float) WHEEL_COUNTS_LENGTH); 
    float velocity_ms = -rps_wheel * 6.28f * 0.33f 
      / 1000.0f * 3600.0f * 0.277778;

  //--------------------------- Calculate cadence ----------------------------//
    int32_t current_pedal_count = pedal_counter.read();
    int32_t previous_pedal_count = pedal_counts[pedal_counts_index];
    pedal_counts[pedal_counts_index] = current_pedal_count;
    pedal_counts_index += 1;
    if (pedal_counts_index >= PEDAL_COUNTS_LENGTH) pedal_counts_index = 0;
    // There are 192 counts/revolution
    float cadence_rads = ((float) (current_pedal_count - previous_pedal_count)) 
      / 192.0f * 60.0f * 1000.0f 
      / ((float) PEDAL_COUNTS_LENGTH) * 0.10471975511970057;
  #endif

  #if USE_IMU
  //----------------------------- Read IMU data ------------------------------//
    IMU.Read();
    float accelY = IMU.accel_x_mps2();
    float accelX = IMU.accel_y_mps2();
    float accelZ = IMU.accel_z_mps2();
    float gyroY = IMU.gyro_x_radps();
    float gyroX = IMU.gyro_y_radps();
    float gyroZ = IMU.gyro_z_radps();
    float temp = IMU.die_temp_c();
  #endif
  
  #if SERIAL_DEBUG
  //------------------------ Printing to serial port -------------------------//
    // Limit the printing rate
    if (haptics_iteration_counter % 100 == 0){
      Serial.print("Switch: ");
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
      // IMU
      #if USE_IMU 
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
      #endif
      // Encoders
      #if USE_ENCODERS
        Serial.print(",Velocity= ");
        Serial.print(velocity_ms);
        Serial.print(",Cadence= ");
        Serial.print(cadence_rads);
      #endif
      // New line
      Serial.println();
    }
  #endif

  // #if USE_BT
  // //------------------------- Printing to Bluetooth --------------------------//
  //   // Limit the printing rate
  //   if (hand_switch_state == 1 && haptics_iteration_counter % 5 == 0){
  //     Serial1.print(hand_switch_state);
  //     Serial1.print(",");
  //     Serial1.println(angle_fork, 2);
  //   }
  // #endif
  
  // #if USE_RADIO
  // //--------------------------- Printing to Radio ----------------------------//
  //   // Limit the printing rate
  //   if (haptics_iteration_counter % 10 == 0){
  //     radio.stopListening();
  //     char payloadSend[32];
  //     sprintf(payloadSend, "%d,%.4f,%.4f", 
  //       hand_switch_state, angle_hand, filtered_angle_rate);
  //     radio.write(payloadSend, 32);
  //     radio.startListening();
  //   }
  // #endif

  //-------------------------- Printing to SD card ---------------------------//
  #if USE_SD
    size_t n = rb.bytesUsed();
    // Check if there is free space
    if ((n + logFile.curPosition()) > (log_file_size - 100)) {
      digitalWrite(hand_led, LOW);
      isFull = true;
    }
    if (!isFull) {
      // If the file is not busy, write out one sector of data
      if (n >= 512 && !logFile.isBusy()) rb.writeOut(512);
      // Write data to buffer
      rb.print(haptics_iteration_counter);
      rb.write(',');
      rb.print(mpc_iteration_counter);
      rb.write(',');
      rb.print(sinceLast);
      rb.write(',');
      rb.print(hand_switch_state);
      rb.write(',');
      rb.print(angle_hand,2);
      rb.write(',');
      rb.print(angle_fork,2);
      rb.write(',');
      rb.print(angle_rate,2);
      rb.write(',');
      rb.print(filtered_angle_rate,2);
      rb.write(',');
      rb.print(error,2);
      rb.write(',');
      rb.print(command_fork,2);
      rb.write(',');
      rb.print(command_hand,2);
      #if USE_BT
        rb.write(',');
        rb.print(bt_message_double,2);
      #endif
      #if USE_RADIO
        rb.write(',');
        rb.print(radio_message,2);
        rb.write(',');
        rb.print(payloadRecv);
      #endif
      #if USE_IMU 
        rb.write(',');
        rb.print(gyroX,3);
        rb.write(',');
        rb.print(gyroY,3);
        rb.write(',');
        rb.print(gyroZ,3);
      #endif
      rb.write('\n');
    }
    // Flush the data from buffer to file. Try to do it at rarely as possible.
    // Flushing takes a couple of milliseconds (around 3-4), which makes the
    // next 3 or 4 PID controller iterations slightly out-of-time.
    // Since the normal flush-less iteration takes significantly less than 1ms, 
    // the controller gets back in time quickly.
    // CAN SOMETIMES CAUSE A LAG SPIKE OF MULTIPLE SECONDS
    // TODO: Find a workaround
    if (haptics_iteration_counter % 1500 == 0) logFile.flush();
  #endif
}

float return_scaling(uint64_t iteration){
  // Slowly ramps up the torque over 13 seconds. Used to avoid commanding high 
  // torques when turning on the bicycle, if the handlebars and the wheel are
  // not aligned. Also used to slowly introduce the MPC torque to the rider.
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

//float moving_avg(float new_value){
//  avg_sum = avg_sum - avg_array[avg_index];
//  avg_array[avg_index] = new_value;
//  avg_sum = avg_sum + new_value;
//  avg_index = (avg_index+1) % avg_filter_size;
//
//  return (float) (avg_sum / avg_filter_size);
//}

uint8_t checkSwitch(uint8_t curr_value, uint8_t *val_array, uint8_t array_size){
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

// #if USE_BT
//   void bt_read(){
//     static byte ndx = 0;
//     char endMarker = '\n';
//     char rc;

//     while (Serial1.available() > 0 && bt_message_new == false){
//       rc = Serial1.read();
//       if (rc != endMarker){
//         bt_message_string[ndx] = rc;
//         ndx++;
//         if (ndx >= bt_message_length){
//           ndx = bt_message_length - 1;
//         }
//       }
//       else{
//         bt_message_string[ndx] = '\0'; // terminate the string
//         ndx = 0;
//         bt_message_new = true;
//       }
//     }
//   }

//   void bt_parse(){
//     if (bt_message_new == true){
//       bt_message_double = atof(bt_message_string);
//       bt_message_new = false;
//     }
//   }
// #endif

#if USE_SD
  void openFile() {
    String fullFileName = fileName + String(fileCount) + fileExt;
    if (!logFile.open(fullFileName.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
      Serial.println("Open failed");
      while (1) { // Long-long error code
        digitalWrite(hand_led, HIGH);
        delay(1000);
        digitalWrite(hand_led, LOW);
        delay(500);
      }
    }
    if (!logFile.preAllocate(log_file_size)) {
      Serial.println("Preallocate failed");
      while (1) { // Long-long error code
        digitalWrite(hand_led, HIGH);
        delay(1000);
        digitalWrite(hand_led, LOW);
        delay(500);
      }
    }
    rb.begin(&logFile);
    rb.print("haptics_iteration_counter");
    rb.write(',');
    rb.print("mpc_iteration_counter");
    rb.write(',');
    rb.print("sinceLast");
    rb.write(',');
    rb.print("hand_switch_state");
    rb.write(',');
    rb.print("angle_hand");
    rb.write(',');
    rb.print("angle_fork");
    rb.write(',');
    rb.print("angle_rate");
    rb.write(',');
    rb.print("filtered_angle_rate");
    rb.write(',');
    rb.print("error");
    rb.write(',');
    rb.print("command_fork");
    rb.write(',');
    rb.print("command_hand");
    // #if USE_BT
    //   rb.write(',');
    //   rb.print("bt_message_double");
    // #endif
    // #if USE_RADIO
    //   rb.write(',');
    //   rb.print("radio_message");
    //   rb.write(',');
    //   rb.print("payloadRecv");
    // #endif
    #if USE_IMU
      rb.write(',');
      rb.print("gyroX");
      rb.write(',');
      rb.print("gyroY");
      rb.write(',');
      rb.print("gyroZ");
    #endif
    rb.write('\n');
    size_t n = rb.bytesUsed();
    rb.writeOut(n);
    logFile.flush();
    isOpen = true;
  }
#endif