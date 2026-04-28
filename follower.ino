// LIBRARIES
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <QMC5883L.h>
#include <Wire.h>
//#include <PID_v1.h>
#include <QuickPID.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//---------------- TODO ----------------
/*
  - Strings Serial(F("   "))
  - Reverse 
 */

// DEFINITIONS
//#define DBG   // Debug Flag

// COMPASS
#define USE_BNO055
#define BNO055_OFFSET           0.0f
#define ONBOARD_COMPASS_OFFSET  -90.0f
#define COMPASS_MAX_RETRIES     2
#define HEADING_CORRECTION_OUTER_ANGLE  60
#define HEADING_CORRECTION_INNER_ANGLE  30

// MOTORS
#define RPM_TIMEOUT_US      100000 // 100ms
#define UNUSABLE_RPM        200
#define ABRUPT_THRESHOLD    0.2f
#define CM_S_TO_RPM         4.2441 // rpm = CM_S_TO_RPM * speed_cm/s
#define MAX_RPM             180
#define MAX_PWM             255
#define DEFAULT_NAV_RPM     120
#define DEFAULT_ATTACK_RPM  180
#define DEFAULT_CALIB_RPM   80
 
//---------------- STATE DEFINITIONS ----------------

// ON THE MOVE STATE
#define MOVE_DURATION_MS  5000   
#define MOVE_ANGLE_MIN    30
#define MOVE_ANGLE_MAX    330

// COLLISION AVOIDANCE
#define COLLISION_REV_MS  1000
#define COLLISION_TURN_MS 400
//---------------------------------------------------

//---------------- PINOUT ----------------

// IR 
#define IR_THRESHOLD  860
#define IR_LEFT_PIN   A7
#define IR_RIGHT_PIN  A6

// SONAR
#define SONAR_TIMEOUT_US  30000 
#define SONAR_OBSTACLE_CM 30
#define SONAR_TRIG_PIN    A1
#define SONAR_ECHO_PIN    A2

// MOTORS
#define MOTORS_STBY_PIN       4  
#define MOTORS_LEFT_IN1_PIN   A0
#define MOTORS_LEFT_IN2_PIN   6
#define MOTORS_RIGHT_IN1_PIN  10
#define MOTORS_RIGHT_IN2_PIN  9
#define MOTOR_LEFT_ENC_PIN    2
#define MOTOR_RIGHT_ENC_PIN   3
#define MOTOR_ENCODER_N       1050 // Pulses per revolution

// COMMS
#define NRF24_CE_PIN    7
#define NRF24_CSN_PIN   8

//----------------------------------------


//---------------- GLOBALs ----------------

#ifdef USE_BNO055
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#else
// COMPASS
// Instantiate an object for the compass
QMC5883L compass;
#endif

// RADIO
// Instantiate an object for the nRF24L01 transceiver
RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);

// an identifying device destination
// Let these addresses be used for the pair
uint8_t address[][6] = { "Node0", "Node1", "Node2" };
String  device_name[3] = { "Interface", "Master", "Follower" };

// Node 0 -> Interface
// Node 1 -> Master
// Node 2 -> Follower

// uniquely identify which address this radio will use to transmit
bool radioNumber = 2;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Timekeeping
unsigned long cur_timestamp, prev_timestamp, time_diff;

// ========== ENUMS ==========
typedef enum : uint16_t  {
  
  MSG_SIMPLE_START_BYTES      = 0xAABB,
  MSG_NAVIGATION_START_BYTES  = 0xAACC,
  MSG_ATTACK_START_BYTES      = 0xAADD,
  MSG_STATUS_START_BYTES      = 0xAAEE
  
} StartMsg;

typedef enum : uint8_t  {
  
  // This state is just for a request. It is not an actual state. 
  // Once the request is satisfied, it need to be cleared
  EMPTY_STATE               = 0x00,
  
  INIT_STATE                = 0xF0,
  ERROR_STATE               = 0xF1,
  STOPPED_STATE             = 0xF2,
  ON_THE_MOVE_STATE         = 0xF3,
  NAVIGATION_STATE          = 0xF4,
  ATTACK_STATE              = 0xF5,
  COMPASS_CALIBRATION_STATE = 0xF6
  
} State;

// State variables
State cur_state, req_state;

// All Messages
typedef enum : uint8_t  {
  
  HEARTBEAT_MSG           = 0xA1,
  STOP_MSG                = 0xA2,
  ON_THE_MOVE_MSG         = 0xA3,
  CALIBRATE_COMPASS_MSG   = 0xA4
  
} Messages;

// All Attack Flags
typedef enum : uint8_t {

  ATTACK_HEADING_RELATIVE_FLAG  = 0b00000001,
  ATTACK_DISTANCE_FLAG          = 0b00000010
  
} AttackFlags;

// All Navigation Flags
typedef enum : uint8_t { 

  HEADING_RELATIVE_FLAG     = 0b00000001,
  COLLISION_AVOIDANCE_FLAG  = 0b00000010,
  DISTANCE_FLAG             = 0b00000100,
  SPEED_FLAG                = 0b00001000,
  DURATION_FLAG             = 0b00010000
  
} NavigationFlags;

// All Avoidance Modes
typedef enum : uint8_t { 

  NO_AVOIDANCE      = 0x0,
  REVERSE_AVOIDANCE = 0x1,
  LEFT_AVOIDANCE    = 0x2,
  RIGHT_AVOIDANCE   = 0x3
  
} AvoidanceMode;

// ===========================

// State manager for Calibrate State
struct CalibrateCompassManager {

  uint8_t rpm_left;
  uint8_t rpm_right;
  bool left_forward;
  bool right_forward;
  uint16_t sel_duration_ms;  // MAX value == 65535
  
  bool has_init = false;
  unsigned long start_time_ms;
  
} calibrate_compass_manager;

// Μanager for collision avoidance
struct CollisionAvoidanceManager {

  uint8_t avoidance_mode;
  
  // Timekeeping
  bool has_init = false;
  unsigned long start_ms;

} collision_avoidance_manager;


// State manager for On The Move State
struct OnTheMoveManager {

  float sel_heading;

  // Timekeeping
  bool has_init = false;
  unsigned long start_ms;
  
} on_the_move_manager;

// State manager for Navigation State
struct NavigationManager {

  uint8_t flags;
  
  bool set_col_avoid    = true;
  bool set_rel_heading  = false;
  bool set_speed        = false;
  bool set_distance     = false;
  bool set_duration     = false;
  bool need_inner       = false;

  // Selected parameters
  float    sel_heading;
  uint16_t sel_distance_cm;
  int16_t  sel_speed_cms;
  uint16_t sel_duration_ms;
  
  // Timekeeping
  bool has_init = false;
  unsigned long start_ms;
  
  // Travelled 
  double count_cm = 0;
  
} navigation_manager;

// State manager for Attack State
struct AttackManager {

  uint8_t flags;
  
  bool set_rel_heading  = false;
  bool set_distance     = false;
  
  // Selected parameters
  float    sel_heading;
  uint16_t sel_distance_cm;
    
  // Timekeeping
  bool has_init = false;
  
  // Travelled 
  double count_cm = 0;
  unsigned long start_ms;
  
} attack_manager;

// ========= MESSAGES =========
// MAKE THE MOST COMMON CASE FAST !
struct Header {
  
  uint16_t start_bytes;
  uint8_t sender_id;
  uint8_t receiver_id;
};

struct SimpleMessage {  // Just a header with an action or state
  
  Header header;
  uint8_t action_state;       // 1 byte 

} simple_payload;

struct NavigationMessage {

  Header header;       
  uint8_t flags;            // 1 byte 
  uint16_t heading_dd;      // 2 bytes Always positive Degrees *10 e.g 0 -> 3600 ==> 0 -> 360
  int16_t speed_cms;        // 2 bytes Can be negative
  uint16_t dist_cm_dur_ms;  // 2 bytes Either for distance or duration Always positive int

} nav_payload;

struct AttackMessage {

  Header header;
  uint8_t flags;
  uint16_t heading_dd;
  uint16_t distance_cm;

} attack_payload;

struct StatusMessage {
  
  Header header;
  uint8_t state;      // 1 byte e.g INIT_STATE, ERROR_STATE
  uint16_t current_heading;
  int16_t current_speed;  

} status_payload;
// ============================

// Motors PID
//Define Variables we'll be connecting to
float motor_left_setpoint, motor_left_input, motor_left_output;
float motor_right_setpoint, motor_right_input, motor_right_output;
unsigned int motor_left_rpm[2], motor_right_rpm[2];
bool left_idx=false, right_idx=false;

// Heading PID
bool must_correct_heading = false;
float cur_heading = 0.0f, temp_heading = 0.0f;
float heading_input, heading_output, heading_setpoint;

uint8_t motor_left_pwm = 0, motor_right_pwm = 0; 
bool motor_left_forward = true, motor_right_forward = true;
  
float current_speed = 0.0;

// PIDs For 5V
//double Kp_left=0.3, Ki_left=6.0, Kd_left=0.000001;
//double Kp_right=0.3, Ki_right=6.0, Kd_right=0.000001;

// PIDs For 12V
//double Kp_left=0.15, Ki_left=0.7, Kd_left=0.0000001;
//double Kp_right=0.15, Ki_right=0.7, Kd_right=0.0000001;
//double Kp_heading=0.1, Ki_heading=0.1, Kd_heading=0.0000001;

// PIDs For 11V
double Kp_motor=3.0, Ki_motor=2.8, Kd_motor=0.0;//0001; //0001;
//double Kp_motor=0.06, Ki_motor=0.80, Kd_motor=0.0; //0001;
double Kp_heading=1.5, Ki_heading=0.8, Kd_heading=0.0000001;
//double Kp_heading=6.0, Ki_heading=1.0, Kd_heading=0.0001;

//PID pid_left(&motor_left_input, &motor_left_output, &motor_left_setpoint, Kp_motor, Ki_motor, Kd_motor, DIRECT);
//PID pid_right(&motor_right_input, &motor_right_output, &motor_right_setpoint, Kp_motor, Ki_motor, Kd_motor, DIRECT);
//PID pid_heading(&heading_input, &heading_output, &heading_setpoint, Kp_heading, Ki_heading, Kd_heading, DIRECT);

QuickPID pid_left(&motor_left_input, &motor_left_output, &motor_left_setpoint);
QuickPID pid_right(&motor_right_input, &motor_right_output, &motor_right_setpoint);
QuickPID pid_heading(&heading_input, &heading_output, &heading_setpoint);


// Motor Left
volatile unsigned long motor_left_t_cur = 0, motor_left_t_prev = 0, motor_left_dur;

// Motor Right
volatile unsigned long motor_right_t_cur = 0, motor_right_t_prev = 0, motor_right_dur;

// IR Variables
bool obstacle_left = false, obstacle_right = false;

// Sonar variables
bool obstacle_forward = false;
uint16_t sonar_dist_cm = 0;


void LEFT_RPM_ISR(void)
{
  #ifdef DBG
  //Serial.println("L");
  #endif

  motor_left_t_cur = micros();
  motor_left_dur = motor_left_t_cur - motor_left_t_prev;
  motor_left_t_prev = motor_left_t_cur;
}

void RIGHT_RPM_ISR(void)
{
  #ifdef DBG
  //Serial.println("R");
  #endif

  motor_right_t_cur = micros();
  motor_right_dur = motor_right_t_cur - motor_right_t_prev;
  motor_right_t_prev = motor_right_t_cur;
}

// Setup everything - Run once in the beginning
void setup() {
  
  // Define and initialize IO

  // Outputs
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(MOTORS_STBY_PIN, OUTPUT);
  pinMode(MOTORS_LEFT_IN1_PIN, OUTPUT);
  pinMode(MOTORS_LEFT_IN2_PIN, OUTPUT);
  pinMode(MOTORS_RIGHT_IN1_PIN, OUTPUT);
  pinMode(MOTORS_RIGHT_IN2_PIN, OUTPUT);

  // Initialize Outputs
  digitalWrite(SONAR_TRIG_PIN, LOW);
  digitalWrite(MOTORS_STBY_PIN, LOW);
  digitalWrite(MOTORS_LEFT_IN1_PIN, LOW);
  digitalWrite(MOTORS_LEFT_IN2_PIN, LOW);
  digitalWrite(MOTORS_RIGHT_IN1_PIN, LOW);
  digitalWrite(MOTORS_RIGHT_IN2_PIN, LOW);

  // Inputs
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  pinMode(MOTOR_LEFT_ENC_PIN, INPUT);
  pinMode(MOTOR_RIGHT_ENC_PIN, INPUT);

  // Initialize variables
  
  cur_state = State::INIT_STATE;

  obstacle_left = false;
  obstacle_right = false;
  obstacle_forward = false;

  sonar_dist_cm = 0;

  motor_left_rpm[0] = 0;
  motor_left_rpm[1] = 0;
  
  motor_right_rpm[0] = 0;
  motor_right_rpm[1] = 0;
  
  left_idx=false;
  right_idx=false;
  
  // Set initial setpoints
  
  heading_setpoint = 0;
  heading_input = 0;
  heading_output = 0;
  
  motor_left_setpoint = 0;
  motor_right_setpoint = 0;
  motor_left_pwm = 0;
  motor_right_pwm = 0;
  motor_left_forward = true;
  motor_right_forward = true;

  prev_timestamp = millis();
    
  // Initialize Serial communication for debug
  #ifdef DBG
  Serial.begin(115200);
  #else
  Serial.begin(115200);
  #endif

  // Start native I2C
  Wire.begin();

  #ifdef USE_BNO055

  // Initialize BNO055 module

  while(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(1000);
  }

  #else

  // Initialize onboard compass
  compass.init();
  compass.setSamplingRate(20);

  #endif  
  
  // Initialize the transceiver on the SPI bus
  while (!radio.begin()) {
    #ifdef DBG
    Serial.print("\nRadio hardware is not responding!");
    #endif
    delay(200);
  }

  // Must set outputs again as the radio uses SPI which might set an output (D10) as input
  // Outputs
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(MOTORS_STBY_PIN, OUTPUT);
  pinMode(MOTORS_LEFT_IN1_PIN, OUTPUT);
  pinMode(MOTORS_LEFT_IN2_PIN, OUTPUT);
  pinMode(MOTORS_RIGHT_IN1_PIN, OUTPUT);
  pinMode(MOTORS_RIGHT_IN2_PIN, OUTPUT);

  // Initialize Outputs
  digitalWrite(SONAR_TRIG_PIN, LOW);
  digitalWrite(MOTORS_STBY_PIN, LOW);
  digitalWrite(MOTORS_LEFT_IN1_PIN, LOW);
  digitalWrite(MOTORS_LEFT_IN2_PIN, LOW);
  digitalWrite(MOTORS_RIGHT_IN1_PIN, LOW);
  digitalWrite(MOTORS_RIGHT_IN2_PIN, LOW);
  
  #ifdef DBG
  Serial.print("\nRadio init done");
  #endif

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  //radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();  // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  // set the TX address of the RX node for use on the TX pipe (pipe 0)
  radio.stopListening(address[1]);  // put radio in TX mode

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[2]);  // using pipe 1
  
  // RPM sensor callback for Left motor
  cli(); // Clear interrupts
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENC_PIN),  LEFT_RPM_ISR,  FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENC_PIN), RIGHT_RPM_ISR, FALLING);
  sei(); // Enable interrupts

  // Enable motors
  digitalWrite(MOTORS_STBY_PIN, HIGH);
  
  #ifdef DBG
  Serial.print("\nSetup done!");
  #endif
    
  // Turn the PIDs on

  //PID pid_left(&motor_left_input, &motor_left_output, &motor_left_setpoint, Kp_motor, Ki_motor, Kd_motor, DIRECT);
//PID pid_right(&motor_right_input, &motor_right_output, &motor_right_setpoint, Kp_motor, Ki_motor, Kd_motor, DIRECT);
//PID pid_heading(&heading_input, &heading_output, &heading_setpoint, Kp_heading, Ki_heading, Kd_heading, DIRECT);


  pid_left.SetTunings(Kp_motor, Ki_motor, Kd_motor);
  pid_right.SetTunings(Kp_motor, Ki_motor, Kd_motor);
  pid_heading.SetTunings(Kp_heading, Ki_heading, Kd_heading);
  
  //pid_left.SetMode(AUTOMATIC);
  //pid_right.SetMode(AUTOMATIC);
  pid_left.SetMode(pid_left.Control::automatic);
  pid_right.SetMode(pid_right.Control::automatic);

  pid_left.SetAntiWindupMode(pid_left.iAwMode::iAwClamp); 
  pid_right.SetAntiWindupMode(pid_right.iAwMode::iAwClamp); 

  pid_left.SetSampleTimeUs(25000);
  pid_right.SetSampleTimeUs(25000);
  
  pid_left.SetOutputLimits(0, MAX_PWM);
  pid_right.SetOutputLimits(0, MAX_PWM);
  
  //pid_heading.SetMode(AUTOMATIC);
  pid_heading.SetMode(pid_heading.Control::automatic);
  pid_heading.SetOutputLimits(-MAX_RPM, MAX_RPM);
  pid_heading.SetSampleTimeUs(25000);

  // Start radio reception
  radio.startListening();  // put radio in RX mode

  // Wait a while
  delay(2000);
  cur_timestamp = millis();
}

// Run forever
void loop() {
  
  // Check Left IR
  obstacle_left = readLeftIR();
  
  // Read Right IR
  obstacle_right = readRightIR();

  // Read sonar distance in cm
  sonar_dist_cm = readSonar();

  // Check if Sonar detects obstacle
  if(sonar_dist_cm > 0 && sonar_dist_cm <= SONAR_OBSTACLE_CM){
    obstacle_forward = true;
  }else{
    obstacle_forward = false;
  }

  #ifdef DBG
  Serial.print(F("\nObstacles:\nLeft: "));
  Serial.print(obstacle_left);
  Serial.print(F("\tForward: "));
  Serial.print(obstacle_forward);
  Serial.print(F("\tRight: "));
  Serial.print(obstacle_right);   
  #endif
  
  delay(2);
  
  // Read compass heading in degrees
  temp_heading = readCompass();
  if(temp_heading >=0 ) cur_heading = temp_heading;

  //heading_setpoint      = 0.0;   // αυτές οι 3 γραμμές υπήρχαν κανονικά στον κώδικα
  //motor_left_setpoint   = 0.0;   //
  //motor_right_setpoint  = 0.0;   //
  //motor_left_output     = 0.0;
  //motor_right_output    = 0.0;
  
  heading_input = 0;
  heading_output = 0;

  must_correct_heading = false;

  // Yield for a while. Hardware routines need to run
  delay(1);
  
  // Disable interrupts
  //cli();
  //motor_left_has_init = false;

  // This device is a RX node

  uint8_t pipe;
  if (radio.available(&pipe)) { // is there a payload? get the pipe number that received it
    uint8_t n_bytes = radio.getDynamicPayloadSize();  // get the size of the payload

    #ifdef DBG
    Serial.print("\nMessage received with ");
    Serial.print(n_bytes);
    Serial.print(" bytes");
    #endif
    
    // Check which message is it
    if(n_bytes == 5){ // If bytes == 5 then it is probably a simple message
      
      radio.read(&simple_payload, sizeof(simple_payload));  // get incoming payload

      // Check if it is actually a simple message
      if(simple_payload.header.start_bytes == StartMsg::MSG_SIMPLE_START_BYTES){

        // Prepare Header payload
        Header tmp_header;

        switch(simple_payload.action_state){

          case Messages::HEARTBEAT_MSG:{

            tmp_header.start_bytes = StartMsg::MSG_STATUS_START_BYTES;
            tmp_header.sender_id = 2;
            tmp_header.receiver_id = 1;
            
            status_payload.header = tmp_header;
            status_payload.state = cur_state;
            status_payload.current_heading = uint16_t(cur_heading*100);
            status_payload.current_speed = int16_t(current_speed);
            
            #ifdef DBG
            Serial.print("\nSending status response:\nStatus: ");
            Serial.print(state_str(cur_state));
            Serial.print("\tHeading:");
            Serial.print(cur_heading);
            Serial.print("\tSpeed: ");
            Serial.print(current_speed);
            #endif
            
            radio.writeAckPayload(1, &status_payload, sizeof(status_payload));

            break;
          }

          case Messages::STOP_MSG:{

            // Request switching to STOPPED_STATE 
            req_state = State::STOPPED_STATE;

            break;
          }

          case Messages::ON_THE_MOVE_MSG:{

            // Request switching to ON_THE_MOVE_STATE 
            req_state = State::ON_THE_MOVE_STATE;

            break;
          }

          case Messages::CALIBRATE_COMPASS_MSG:{

            // Request switching to COMPASS_CALIBRATION_STATE 
            req_state = State::COMPASS_CALIBRATION_STATE;

            break;
          }
        }
      }
         
    }else if(n_bytes == 11){ // If bytes == 11 then it is probably a navigation message

      radio.read(&nav_payload, sizeof(nav_payload));  // get incoming payload 

      // Check if it is actually a navigation message
      if(nav_payload.header.start_bytes == StartMsg::MSG_NAVIGATION_START_BYTES){

        // Request switching to NAVIGATION_STATE 
        req_state = State::NAVIGATION_STATE;

        // Navigation Priorities: Distance > Speed > Duration
        navigation_manager.flags            = nav_payload.flags;
        navigation_manager.set_col_avoid    = ((nav_payload.flags & NavigationFlags::COLLISION_AVOIDANCE_FLAG) > 0);
        navigation_manager.set_rel_heading  = ((nav_payload.flags & NavigationFlags::HEADING_RELATIVE_FLAG) > 0); 
        navigation_manager.set_distance     = ((nav_payload.flags & NavigationFlags::DISTANCE_FLAG) > 0);
        navigation_manager.set_speed        = ((nav_payload.flags & NavigationFlags::SPEED_FLAG) > 0);
        navigation_manager.set_duration     = ((nav_payload.flags & NavigationFlags::DURATION_FLAG) > 0) and (!navigation_manager.set_distance);

        if(navigation_manager.set_rel_heading){ // In case of relative heading

          navigation_manager.sel_heading = float(nav_payload.heading_dd) / 10.0f;
          navigation_manager.sel_heading += cur_heading;

          // [0 -> 359]
          if(navigation_manager.sel_heading < 0){
            navigation_manager.sel_heading += 360;
          }else if(navigation_manager.sel_heading > 360){
            navigation_manager.sel_heading -= 360;
          }          
        
        }else{  // in case of absolute heading
          navigation_manager.sel_heading = float(nav_payload.heading_dd) / 10.0f;
        }

        // Set distance if applicable
        if(navigation_manager.set_distance) navigation_manager.sel_distance_cm = nav_payload.dist_cm_dur_ms;

        // Set speed if applicable
        if(navigation_manager.set_speed) navigation_manager.sel_speed_cms = nav_payload.speed_cms;

        // Set duration if applicable
        if(navigation_manager.set_duration) navigation_manager.sel_duration_ms = nav_payload.dist_cm_dur_ms;

        // Reset variables
        navigation_manager.count_cm     = 0;
        navigation_manager.has_init     = false;
      }
      
    }else if(n_bytes == 9){

      // Assume it is an Attack message
      radio.read(&attack_payload, sizeof(attack_payload));  // get incoming payload

      switch(attack_payload.header.start_bytes){
      
        case StartMsg::MSG_ATTACK_START_BYTES :{

          // State manager for Attack State

          attack_manager.flags = attack_payload.flags;
          attack_manager.set_rel_heading  = ((attack_payload.flags & NavigationFlags::HEADING_RELATIVE_FLAG) > 0); 
          attack_manager.set_distance     = ((attack_payload.flags & NavigationFlags::DISTANCE_FLAG) > 0);

          // Set heading... absolute, relative
          
          if(attack_manager.set_rel_heading){ // In case of relative heading

            attack_manager.sel_heading = float(attack_payload.heading_dd) / 10.0f;
            attack_manager.sel_heading += cur_heading;
  
            // [0 -> 359]
            if(attack_manager.sel_heading < 0){
              attack_manager.sel_heading += 360;
            }else if(attack_manager.sel_heading > 360){
              attack_manager.sel_heading -= 360;
            }          
          
          }else{  // in case of absolute heading
            attack_manager.sel_heading = float(attack_payload.heading_dd) / 10.0f;
          }

          // Set distance constrained flag

          attack_manager.set_distance = ((attack_payload.flags & AttackFlags::ATTACK_DISTANCE_FLAG) > 0);

          // Set distance
          if(attack_manager.set_distance) attack_manager.sel_distance_cm = attack_payload.distance_cm;

          // Request switching to ATTACK_STATE 
          req_state = State::ATTACK_STATE;

          attack_manager.has_init = false;
          attack_manager.count_cm = 0;

          break;          
        }

        case StartMsg::MSG_STATUS_START_BYTES :{

          // Since it is a status message, we need to convert it
          //memcpy(&status_payload, &attack_payload, sizeof(status_payload));

          break;          
        }

      }
      
    }
  }

  // Yield for a while. Hardware routines need to run
  delay(1);
  
  // Enable interrupts
  //sei();
  
  // Minimum delay for RPMs to be valid
  //delay(20);

  // Get wheels individual speeds
  
  motor_left_rpm[left_idx] = readLeftRPM();
  motor_right_rpm[right_idx] = readRightRPM();

  motor_left_input  = ((motor_left_rpm[left_idx] == 0) || (motor_left_rpm[left_idx] > UNUSABLE_RPM) ) ? float(motor_left_rpm[!left_idx]) : float(motor_left_rpm[left_idx]);
  motor_right_input = ((motor_right_rpm[left_idx] == 0) || (motor_right_rpm[left_idx] > UNUSABLE_RPM) ) ? float(motor_right_rpm[!left_idx]) : float(motor_right_rpm[left_idx]);
  //|| (abs((float(motor_left_rpm[left_idx]) - float(motor_left_input))/float(motor_left_input)) > ABRUPT_THRESHOLD)
  //|| (abs((float(motor_right_rpm[left_idx]) - float(motor_right_input))/float(motor_right_input)) > ABRUPT_THRESHOLD)
  left_idx  = !left_idx; 
  right_idx = !right_idx;
 
  //Serial.println(motor_left_input);

  // Calculate the actual speed of the robot 
  current_speed = motor_left_input;
  current_speed += motor_right_input;
  current_speed = current_speed / 2.0f;
  
  #ifdef DBG
  Serial.print("\nMotor Left RPM: ");
  Serial.print(motor_left_input);  
  Serial.print("\nMotor Right RPM: ");
  Serial.print(motor_right_input);
  #endif

  // Get current timestamp in ms
  cur_timestamp = millis();
  time_diff = cur_timestamp - prev_timestamp;
  
  // Check current state and apply behaviour
  switch(cur_state){

    // INIT_STATE -> Stop
    case State::INIT_STATE: {

      #ifdef DBG
      Serial.print("\nState: INIT_STATE");
      #endif
      
      cur_state = State::STOPPED_STATE; //ATTACK_STATE; //

      
      // Stop motors
      setLeftSpeed(0, true);
      setRightSpeed(0, true);

      break;
    }

    case State::ERROR_STATE: {

      #ifdef DBG
      Serial.print("\nState: ERROR_STATE");
      #endif 
      
      cur_state = State::ERROR_STATE;

      // Set initial motor setpoint
      motor_left_setpoint = 0;
      motor_right_setpoint = 0;

      // Stop motors
      setLeftSpeed(0, true);
      setRightSpeed(0, true);
      
      break;
    }

    case State::STOPPED_STATE: {
      
      #ifdef DBG
      Serial.print("\nState: STOPPED_STATE");
      #endif 

      /*
      pid_heading.SetMode(MANUAL);
      pid_heading.SetMode(AUTOMATIC);
      
      pid_left.SetMode(MANUAL);
      pid_left.SetMode(AUTOMATIC);
      
      pid_right.SetMode(MANUAL);
      pid_right.SetMode(AUTOMATIC);
      */

      resetMotorLeftPID();
      resetMotorRightPID();
      resetHeadingPID();
      
      break;
    }

    case State::ON_THE_MOVE_STATE: {
      
      #ifdef DBG
      Serial.print("\nState: ON_THE_MOVE_STATE");
      #endif

      // This mode requires heading correction
      must_correct_heading = true;

      // Initialize state manager
      if(!on_the_move_manager.has_init){

        // Set random heading 
        on_the_move_manager.sel_heading = random(MOVE_ANGLE_MIN, MOVE_ANGLE_MAX);
        on_the_move_manager.sel_heading += cur_heading;

        // [0 -> 359]
        if(on_the_move_manager.sel_heading < 0){
          on_the_move_manager.sel_heading += 360;
        }else if(on_the_move_manager.sel_heading > 360){
          on_the_move_manager.sel_heading -= 360;
        }
          
        on_the_move_manager.start_ms      = cur_timestamp;
        on_the_move_manager.has_init      = true;
        
        motor_left_setpoint   = DEFAULT_NAV_RPM;
        motor_right_setpoint  = DEFAULT_NAV_RPM;
        
        motor_left_forward = true;
        motor_right_forward = true;
        
        // PID reset
        resetHeadingPID();
      }
            
      // Check if time has passed
      if(on_the_move_manager.has_init){

        // Check if we have remaining time in ms
        if((cur_timestamp - on_the_move_manager.start_ms) >= MOVE_DURATION_MS) {

          on_the_move_manager.has_init = false;
        }
      }
      
      motor_left_setpoint   = DEFAULT_NAV_RPM;
      motor_right_setpoint  = DEFAULT_NAV_RPM;
      
      motor_left_forward = true;
      motor_right_forward = true;

      // Prepare heading input
      double tmp_diff = double(on_the_move_manager.sel_heading) - double(cur_heading);
      
      if((abs(tmp_diff) > 180) && (tmp_diff < 0)){
        tmp_diff += 360.0;    
      }else if((abs(tmp_diff) > 180) && (tmp_diff > 0)){
        tmp_diff -= 360.0; 
      }
      
      heading_input = tmp_diff;

      if(collision_avoidance()){

        must_correct_heading = false;  
      } 
      
      break;
    }

    case State::NAVIGATION_STATE: {

      #ifdef DBG
      Serial.print("\nState: NAVIGATION_STATE");
      #endif

      // Check for collision avoidance
      // TODO
      // if(navigation_manager.set_col_avoid){}
      bool is_reverse = false;

      // This mode requires heading correction
      must_correct_heading = true;

      // Prepare heading input
      double tmp_diff = double(navigation_manager.sel_heading) - double(cur_heading);
      
      if((abs(tmp_diff) > 180) && (tmp_diff < 0)){
        tmp_diff += 360.0;    
      }else if((abs(tmp_diff) > 180) && (tmp_diff > 0)){
        tmp_diff -= 360.0; 
      }
      
      heading_input = tmp_diff;

      if(!navigation_manager.has_init){

        // PID reset
        resetMotorLeftPID();
        resetMotorRightPID();
        resetHeadingPID();
        
        navigation_manager.start_ms = cur_timestamp;
        navigation_manager.count_cm = 0;
        
        navigation_manager.has_init = true;        
      }
      
      if(navigation_manager.set_speed) {
      
        is_reverse = (navigation_manager.sel_speed_cms < 0);

        float tmp_left_rpm    = (float)navigation_manager.sel_speed_cms*CM_S_TO_RPM;
        float tmp_right_rpm   = (float)navigation_manager.sel_speed_cms*CM_S_TO_RPM;
        motor_left_setpoint   = abs(tmp_left_rpm);
        motor_right_setpoint  = abs(tmp_right_rpm);

        motor_left_forward = !is_reverse;
        motor_right_forward = !is_reverse;
        
      }else{

        motor_left_setpoint   = DEFAULT_NAV_RPM;
        motor_right_setpoint  = DEFAULT_NAV_RPM;

        motor_left_forward = true;
        motor_right_forward = true;
        
      }

      if(navigation_manager.set_distance){

        if(navigation_manager.has_init){
          
          double dx = 14.137166941 * double(current_speed) * double(time_diff);

          dx = dx/60000.0;

          navigation_manager.count_cm += dx;
        
          // Check if we have traveled enough cm
          if(navigation_manager.count_cm >= navigation_manager.sel_distance_cm) {

            navigation_manager.has_init = false;
            
            // Switch to STOPPED_STATE
            cur_state = State::STOPPED_STATE;

            break;

          }
        }        
      }

      if(navigation_manager.set_duration) {
      
        if(navigation_manager.has_init){
        
          // Check if we have remaining time in ms
          if((cur_timestamp - navigation_manager.start_ms) >= navigation_manager.sel_duration_ms) {

            navigation_manager.has_init = false;
            
            // Switch to STOPPED_STATE
            cur_state = State::STOPPED_STATE;

          }
        }
      }

      if(navigation_manager.set_col_avoid){
        
        if(collision_avoidance()){

          must_correct_heading = false;  
        }
      }      

      break;
    } 

    case State::ATTACK_STATE: {

      #ifdef DBG
      Serial.print("\nState: ATTACK_STATE");
      #endif

      
      Serial.print(F("ATK | cur:"));                    // για diagnostics
      Serial.print(cur_heading);
      Serial.print(F(" sel:"));
      Serial.print(attack_manager.sel_heading);
      Serial.print(F(" err:"));
      Serial.print(heading_input);
      Serial.print(F(" out:"));
      Serial.print(heading_output);
      Serial.print(F(" L_set:"));
      Serial.print(motor_left_setpoint);
      Serial.print(F(" R_set:"));
      Serial.println(motor_right_setpoint);


      // Check for collision avoidance
      // TODO
      // if(navigation_manager.set_col_avoid){}
      bool is_reverse = false;

      // This mode requires heading correction
      must_correct_heading = true;

      // Prepare heading input
      double tmp_diff = double(attack_manager.sel_heading) - double(cur_heading);
      
      if((abs(tmp_diff) > 180) && (tmp_diff < 0)){
        tmp_diff += 360.0;    
      }else if((abs(tmp_diff) > 180) && (tmp_diff > 0)){
        tmp_diff -= 360.0; 
      }
      
      heading_input = tmp_diff;

      if(!attack_manager.has_init){

        // PID reset
        resetMotorLeftPID();
        resetMotorRightPID();
        resetHeadingPID();
        
        attack_manager.start_ms = cur_timestamp;
        attack_manager.count_cm = 0;
        
        attack_manager.has_init = true;        
      }
      
     
      motor_left_setpoint   = DEFAULT_ATTACK_RPM;
      motor_right_setpoint  = DEFAULT_ATTACK_RPM;

      motor_left_forward = true;
      motor_right_forward = true;
        
      if(attack_manager.set_distance){

        if(attack_manager.has_init){
          
          double dx = 14.137166941 * double(current_speed) * double(time_diff);

          dx = dx/60000.0;

          attack_manager.count_cm += dx;
        
          // Check if we have traveled enough cm
          if(attack_manager.count_cm >= attack_manager.sel_distance_cm) {

            attack_manager.has_init = false;
            
            // Switch to STOPPED_STATE
            cur_state = State::STOPPED_STATE;

            break;

          }
        }        
      }

      break;
    }

    case State::COMPASS_CALIBRATION_STATE: {

      #ifdef DBG
      Serial.print("\nState: COMPASS_CALIBRATION_STATE");
      #endif
      
      // Check if it is initialized
      if(!calibrate_compass_manager.has_init){

        // Setup calibrate compass parameters
        calibrate_compass_manager.rpm_left      = DEFAULT_CALIB_RPM;
        calibrate_compass_manager.rpm_right     = DEFAULT_CALIB_RPM;
        calibrate_compass_manager.left_forward  = true;
        calibrate_compass_manager.right_forward = false;
        calibrate_compass_manager.sel_duration_ms = 10000;  // MAX value == 65535

        // Initialize
        calibrate_compass_manager.start_time_ms = cur_timestamp;
        calibrate_compass_manager.has_init = true;
      }

      // Check if we have remaining time in ms
      if((cur_timestamp - calibrate_compass_manager.start_time_ms) < calibrate_compass_manager.sel_duration_ms){

        // Apply motors setpoints and directions
        motor_left_setpoint   = calibrate_compass_manager.rpm_left;
        motor_right_setpoint  = calibrate_compass_manager.rpm_right;
        motor_left_forward    = calibrate_compass_manager.left_forward;
        motor_right_forward   = calibrate_compass_manager.right_forward;
  
      }else{

        // All Done, need to switch to STOPPED_STATE
        calibrate_compass_manager.has_init = false;

        // Switch to STOPPED_STATE
        cur_state = State::STOPPED_STATE;

        // Set initial motor setpoint
        motor_left_setpoint   = 0;
        motor_right_setpoint  = 0;
  
        // Stop motors
        setLeftSpeed(0, true);
        setRightSpeed(0, true);
      }
        
      break;
    }

    // Unknown State -> Stop
    default :{
      cur_state = State::STOPPED_STATE;

      // Set initial motor setpoint
      motor_left_setpoint = 0;
      motor_right_setpoint = 0;

      // Stop motors
      setLeftSpeed(0, true);
      setRightSpeed(0, true);

      break;
    }
  }
  
  // Yield for a while. Hardware routines need to run
  delay(1);

  #ifdef DBG
  Serial.print(F("\nPID Heading:\nHeading: "));
  Serial.print(cur_heading);
  Serial.print(F("\tSetpoint: "));
  Serial.print(heading_setpoint);
  Serial.print(F("\tInput: "));
  Serial.print(heading_input);
  #endif
  
  // Prepare heading PID
  pid_heading.Compute();
  
  #ifdef DBG
  Serial.print(F("\nOutput: "));
  Serial.println(heading_output);
  #endif      

  if (must_correct_heading){
  
    // Get PID correction
    double tmp_l = motor_left_setpoint;
    double tmp_r = motor_right_setpoint;
    
    // Apply heading correction
    tmp_l -= heading_output;
    tmp_r += heading_output;
    
    // Change Left and Right setpoints to control heading
    motor_left_setpoint = constrain(tmp_l,  0, MAX_RPM);
    motor_right_setpoint = constrain(tmp_r, 0, MAX_RPM);
  }
  
  motor_left_setpoint   = constrain(motor_left_setpoint,  0, MAX_RPM);
  motor_right_setpoint  = constrain(motor_right_setpoint, 0, MAX_RPM);
    
  // PID step
  
  pid_left.Compute();
  pid_right.Compute();
  
  
  //Serial.println(motor_right_input); //pid_right.GetError()

  // Apply constrained PID estimated corrections to the motors
  motor_left_pwm  = uint8_t(constrain(motor_left_output, 0, MAX_PWM));
  motor_right_pwm = uint8_t(constrain(motor_right_output, 0, MAX_PWM));

  // The motors should be completely stopped when in STOPPED_STATE or ERROR_STATE
  if(cur_state == State::STOPPED_STATE || cur_state == State::ERROR_STATE){

    // Set PWMs to 0
    motor_left_pwm = 0;
    motor_right_pwm = 0;

    resetMotorLeftPID();
    resetMotorRightPID();
    resetHeadingPID();
  }

  #ifdef DBG
  Serial.print(F("\nDiff Time:"));
  Serial.print(time_diff);
  Serial.print(F("\nMotor Left (Setpoint, RPM, PWM, Direction):\n"));
  Serial.print(motor_left_setpoint);
  Serial.print(F("\t"));
  Serial.print(motor_left_input);
  Serial.print(F("\t"));
  Serial.print(motor_left_pwm);
  Serial.print(F("\t"));
  Serial.print(motor_left_forward ? F("Forward") : F("Reverse"));
  Serial.print(F("\nMotor Right (Setpoint, RPM, PWM, Direction):\n"));
  Serial.print(motor_right_setpoint);
  Serial.print(F("\t"));
  Serial.print(motor_right_input);
  Serial.print(F("\t"));
  Serial.print(motor_right_pwm);
  Serial.print(F("\t"));
  Serial.print(motor_right_forward ? F("Forward") : F("Reverse"));
  #endif 

  // Set motors pwm 
  setLeftSpeed(motor_left_pwm,   motor_left_forward); 
  setRightSpeed(motor_right_pwm, motor_right_forward);
  
  // Yield for a while. Hardware routines need to run
  delay(1);

  // Handle state change request if the requested state is valid (Not EMPTY_STATE)
  if(req_state != State::EMPTY_STATE) {

    #ifdef DBG
    Serial.print(F("\nSwitching state to: "));
    Serial.print(state_str(req_state));
    #endif

    // Reinit variables
    calibrate_compass_manager.has_init = false;

    // Switch state for next loop
    cur_state = req_state;

    // Clear request
    req_state = State::EMPTY_STATE;
  }

  #ifdef DBG
  Serial.print(F("\n"));
  #endif

  // Timekeeping
  prev_timestamp = cur_timestamp;
}

String state_str(uint8_t state_t){

  switch(state_t){

    case State::EMPTY_STATE:{
      return "EMPTY_STATE";
    }

    case State::INIT_STATE:{
      return "INIT_STATE";
    }

    case State::ERROR_STATE:{
      return "ERROR_STATE";
    }

    case State::STOPPED_STATE:{
      return "STOPPED_STATE";
    }

    case State::ON_THE_MOVE_STATE:{
      return "ON_THE_MOVE_STATE";
    }

    case State::NAVIGATION_STATE:{
      return "NAVIGATION_STATE";
    }

    case State::ATTACK_STATE:{
      return "ATTACK_STATE";
    }

    case State::COMPASS_CALIBRATION_STATE:{
      return "COMPASS_CALIBRATION_STATE";
    }

    default:{
      return " ";
    }
  }

  return " ";  
}

void setLeftSpeed(uint8_t val, bool forward){
  
  if(forward){
    val = 0xFF - val;
    digitalWrite(MOTORS_LEFT_IN1_PIN, HIGH);
    analogWrite(MOTORS_LEFT_IN2_PIN, val);
  }else{
    digitalWrite(MOTORS_LEFT_IN1_PIN, LOW);
    analogWrite(MOTORS_LEFT_IN2_PIN, val);
  }
}

void setRightSpeed(uint8_t val, bool forward){

  if(forward){
    val = 0xFF - val;
    digitalWrite(MOTORS_RIGHT_IN1_PIN, HIGH);
    analogWrite(MOTORS_RIGHT_IN2_PIN, val);
  }else{
    digitalWrite(MOTORS_RIGHT_IN1_PIN, LOW);
    analogWrite(MOTORS_RIGHT_IN2_PIN, val);
  }
}

// Get RPM value for the left motor, assuming a timeout is set properly
unsigned int readRightRPM(){

  unsigned long t_now = micros();

  if(t_now - motor_right_t_cur >= RPM_TIMEOUT_US) {
    return 0;
  }

  unsigned long rpm = (60000000) / (motor_right_dur * MOTOR_ENCODER_N);

  return (unsigned int)rpm;  
}


// Get RPM value for the left motor, assuming a timeout is set properly
unsigned int readLeftRPM(){

  unsigned long t_now = micros();

  if((t_now - motor_left_t_cur) >= RPM_TIMEOUT_US) return 0;

  unsigned long rpm = (60000000) / (motor_left_dur * MOTOR_ENCODER_N);

  return (unsigned int)rpm;  
}

// Check if the Left IR detects an obstacle
bool readLeftIR(){

  uint16_t val = analogRead(IR_LEFT_PIN);
  
  #ifdef DBG
  Serial.print("\nIR Left value: ");
  Serial.print(val);
  Serial.print(", Obstacle: ");
  Serial.print((val <= IR_THRESHOLD));
  #endif
  
  return (val <= IR_THRESHOLD);
}

// Check if the Right IR detects an obstacle
bool readRightIR(){

  uint16_t val = analogRead(IR_RIGHT_PIN);
  
  #ifdef DBG
  Serial.print("\nIR Right value: ");
  Serial.print(val);
  Serial.print(", Obstacle: ");
  Serial.print((val <= IR_THRESHOLD));
  #endif

  return (val <= IR_THRESHOLD);
}

// Collision avoidance
bool collision_avoidance(){

  // Maybe it is already correcting
  if(collision_avoidance_manager.avoidance_mode == AvoidanceMode::NO_AVOIDANCE) {
 
    // Check for obstacles
    if((obstacle_left && obstacle_right && obstacle_forward) || obstacle_forward){ // Must avoid by performing reverse first

      collision_avoidance_manager.avoidance_mode = AvoidanceMode::REVERSE_AVOIDANCE;
      collision_avoidance_manager.start_ms = cur_timestamp; 
      
    }else if(obstacle_left && !obstacle_right){ // Must avoid by performing right turn

      collision_avoidance_manager.avoidance_mode = AvoidanceMode::RIGHT_AVOIDANCE;
      collision_avoidance_manager.start_ms = cur_timestamp; 
              
    }else if(!obstacle_left && obstacle_right){ // Must avoid by performing left turn

      collision_avoidance_manager.avoidance_mode = AvoidanceMode::LEFT_AVOIDANCE;
      collision_avoidance_manager.start_ms = cur_timestamp; 
    }
  }

  // Check if collision avoidance must occur
  if(collision_avoidance_manager.avoidance_mode != AvoidanceMode::NO_AVOIDANCE) {

    // Apply reverse collision avoidance if required
    if(collision_avoidance_manager.avoidance_mode == AvoidanceMode::REVERSE_AVOIDANCE){
      
      if((cur_timestamp - collision_avoidance_manager.start_ms) < COLLISION_REV_MS){
   
        motor_left_setpoint   = DEFAULT_NAV_RPM;
        motor_right_setpoint  = DEFAULT_NAV_RPM;
        
        motor_left_forward  = false;
        motor_right_forward = false;  
      
      }else{

        bool tmp_avoid_left = (random(0, 10) > 5);
                  
        collision_avoidance_manager.avoidance_mode = tmp_avoid_left ? AvoidanceMode::LEFT_AVOIDANCE : AvoidanceMode::RIGHT_AVOIDANCE;
        collision_avoidance_manager.start_ms = cur_timestamp;
      }
    }

    // Apply turning collision avoidance if required
    if(collision_avoidance_manager.avoidance_mode == AvoidanceMode::LEFT_AVOIDANCE){

      if((cur_timestamp - collision_avoidance_manager.start_ms) < COLLISION_TURN_MS){
  
        motor_left_setpoint   = DEFAULT_NAV_RPM;
        motor_right_setpoint  = DEFAULT_NAV_RPM;
        
        motor_left_forward  = false;
        motor_right_forward = true; 
         
      }else{

         collision_avoidance_manager.avoidance_mode = AvoidanceMode::NO_AVOIDANCE;
      }
     
    }else if(collision_avoidance_manager.avoidance_mode == AvoidanceMode::RIGHT_AVOIDANCE){

      if((cur_timestamp - collision_avoidance_manager.start_ms) < COLLISION_TURN_MS){
   
        motor_left_setpoint   = DEFAULT_NAV_RPM;
        motor_right_setpoint  = DEFAULT_NAV_RPM;
        
        motor_left_forward  = true;
        motor_right_forward = false; 
         
      }else{

         collision_avoidance_manager.avoidance_mode = AvoidanceMode::NO_AVOIDANCE;
      }          
    } 
  }

  return (collision_avoidance_manager.avoidance_mode != AvoidanceMode::NO_AVOIDANCE); 
}

// Get compass heading in degrees
// Returns compass heading in degrees after applying offset correction
// Return -1 if measurement could not be aquired
float readCompass(){

  float heading   = -1;
  bool is_ready = false;

  for(uint8_t count=0; count<COMPASS_MAX_RETRIES; count++){

    #ifdef USE_BNO055

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    if (orientationData.type == SENSOR_TYPE_ORIENTATION) {
      
      //Serial.print("\nOrient:");
      //Serial.print(orientationData.orientation.x);

      heading = orientationData.orientation.x;

      // Apply offset correction
      heading += BNO055_OFFSET;

      // [0 -> 359]
      if(heading < 0){
        heading = 360.0f + heading;
      }else if(heading > 360.0f){
        heading = heading - 360.0f;
      }

      break;
    }
    
    #else
    
    is_ready = compass.ready();

    if (is_ready){

      // Get heading
      heading = float(compass.readHeading());

      // Apply offset correction
      heading = heading + ONBOARD_COMPASS_OFFSET;

      // [0 -> 359]
      if(heading < 0){
        heading = 360.0f + heading;
      }else if(heading > 360.0f){
        heading = heading - 360.0f;
      }

      break;  
            
    }else{
       heading = -1.0f;
    }

    #endif
  }
  
  //Serial.println(heading);

  #ifdef DBG
  Serial.print("\nHeading: ");
  Serial.print(heading);
  #endif

  return heading; 
}

// Get sonar distance measurement in cm
uint16_t readSonar(){

  // Send Pulse
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(SONAR_ECHO_PIN, HIGH, SONAR_TIMEOUT_US);

  // Measure Duration for ECHO
  uint16_t distance = ((float)duration*.0343)/2;
  
  #ifdef DBG
  Serial.print("\nDistance (cm): ");
  Serial.print(distance);
  #endif
  
  return distance;
}

void resetMotorLeftPID(){

  //pid_left.SetMode(pid_left.Control::automatic);  
  pid_left.Reset();
}

void resetMotorRightPID(){

  //pid_right.SetMode(pid_right.Control::manual);
  //pid_right.SetMode(pid_right.Control::automatic);  
  pid_right.Reset();
  
}


void resetHeadingPID(){

  //pid_heading.SetMode(pid_heading.Control::manual);
  //pid_heading.SetMode(pid_heading.Control::automatic);  
  pid_heading.Reset();
}
