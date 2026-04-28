// LIBRARIES
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <QMC5883L.h>
#include <Wire.h>
#include <QuickPID.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "HUSKYLENS.h"

//---------------- TODO ----------------
/*
  - Strings Serial(F("   "))
  - Reverse
 */

// DEFINITIONS
//#define DBG   // Debug Flag

// IDs
#define FOLLOWER_ID       1
#define TARGET_ID         2

// COMPASS
//#define USE_BNO055
#define BNO055_OFFSET           0.0f
#define ONBOARD_COMPASS_OFFSET  00.0f
#define COMPASS_MAX_RETRIES     2

// MOTORS
#define RPM_TIMEOUT_US      100000
#define UNUSABLE_RPM        200
#define CM_S_TO_RPM         4.2441
#define MAX_RPM             180
#define MAX_PWM             255
#define DEFAULT_NAV_RPM     120
#define DEFAULT_ATTACK_RPM  180
#define DEFAULT_CALIB_RPM   80

// VISION
#define PX_CM         318
#define PX_DEG        0.21875f
#define HUSKY_WIDTH   320
#define HUSKY_HEIGHT  240
#define WIDTH_THRES   10

//---------------- STATE DEFINITIONS ----------------

// ON THE MOVE STATE
#define MOVE_DURATION_MS  5000
#define MOVE_ANGLE_MIN    30
#define MOVE_ANGLE_MAX    330

// COMPASS CALIBRATION
#define CALIB_DURATION_MS 10000

// COLLISION AVOIDANCE
#define COLLISION_REV_MS  1000
#define COLLISION_TURN_MS 400
//---------------------------------------------------

//---------------- PINOUT ----------------

// SERVO
#define SERVO_PIN       5
#define SERVO_ZERO_POS  92
#define MIN_SERVO_POS   0
#define MAX_SERVO_POS   180
#define SERVO_STEP      1

// IR
#define IR_THRESHOLD    860
#define IR_LEFT_PIN     A7
#define IR_RIGHT_PIN    A6
#define IR_FORWARD_PIN  A3

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
#define MOTOR_ENCODER_N       1050

// COMMS
#define NRF24_CE_PIN    7
#define NRF24_CSN_PIN   8

//----------------------------------------

//---------------- GLOBALs ----------------

// COMPASS
#ifdef USE_BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#else
QMC5883L compass;
#endif

// SERVO + HUSKYLENS
Servo huskyServo;
int16_t servo_pos    = SERVO_ZERO_POS;
bool servo_clockwise = true;

// Vision mode flag
bool color_recognition_active = false;

HUSKYLENS huskylens;

// RADIO
RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);
uint8_t address[][6]   = {"Node0", "Node1", "Node2"};
String  device_name[3] = {"Interface", "Master", "Follower"};

// Timekeeping
unsigned long cur_timestamp, prev_timestamp, time_diff;

// ========== ENUMS ==========
typedef enum : uint16_t {
  MSG_SIMPLE_START_BYTES      = 0xAABB,
  MSG_NAVIGATION_START_BYTES  = 0xAACC,
  MSG_ATTACK_START_BYTES      = 0xAADD,
  MSG_STATUS_START_BYTES      = 0xAAEE
} StartMsg;

typedef enum : uint8_t {
  EMPTY_STATE               = 0x00,
  INIT_STATE                = 0xF0,
  ERROR_STATE               = 0xF1,
  STOPPED_STATE             = 0xF2,
  ON_THE_MOVE_STATE         = 0xF3,
  NAVIGATION_STATE          = 0xF4,
  ATTACK_STATE              = 0xF5,
  COMPASS_CALIBRATION_STATE = 0xF6
} State;

State cur_state, req_state;

typedef enum : uint8_t {
  HEARTBEAT_MSG         = 0xA1,
  STOP_MSG              = 0xA2,
  ON_THE_MOVE_MSG       = 0xA3,
  CALIBRATE_COMPASS_MSG = 0xA4
} Messages;

typedef enum : uint8_t {
  ATTACK_HEADING_RELATIVE_FLAG = 0b00000001,
  ATTACK_DISTANCE_FLAG         = 0b00000010
} AttackFlags;

typedef enum : uint8_t {
  HEADING_RELATIVE_FLAG    = 0b00000001,
  COLLISION_AVOIDANCE_FLAG = 0b00000010,
  DISTANCE_FLAG            = 0b00000100,
  SPEED_FLAG               = 0b00001000,
  DURATION_FLAG            = 0b00010000
} NavigationFlags;

// HuskyLens FSM — runs in parallel with motor FSM
typedef enum : uint8_t {
  HUSKY_STOPPED_STATE         = 0x1,
  LOCATE_FOLLOWER_STATE       = 0x2,
  LOCATE_TARGET_STATE         = 0x3,
  BOTH_ON_FRAME_STATE         = 0x4
} HuskyState;

HuskyState husky_state;

// ========= MESSAGES =========
struct Header {
  uint16_t start_bytes;
  uint8_t  sender_id;
  uint8_t  receiver_id;
};

struct SimpleMessage {
  Header  header;
  uint8_t action_state;
} simple_payload;

struct NavigationMessage {
  Header   header;
  uint8_t  flags;
  uint16_t heading_dd;
  int16_t  speed_cms;
  uint16_t dist_cm_dur_ms;
} nav_payload;

struct AttackMessage {
  Header   header;
  uint8_t  flags;
  int16_t  heading_dd;   // int16_t για αρνητικές γωνίες
  uint16_t distance_cm;
} attack_payload;

struct StatusMessage {
  Header   header;
  uint8_t  state;
  uint16_t current_heading;
  int16_t  current_speed;
} status_payload;
// ============================

// ========= STATE MANAGERS =========

struct CalibrateCompassManager {
  bool          has_init        = false;
  unsigned long start_time_ms;
  uint16_t      sel_duration_ms = CALIB_DURATION_MS;
} calibrate_compass_manager;

struct OnTheMoveManager {
  float         sel_heading;
  bool          has_init = false;
  unsigned long start_ms;
} on_the_move_manager;

struct NavigationManager {
  uint8_t  flags;
  bool     set_col_avoid   = true;
  bool     set_rel_heading = false;
  bool     set_speed       = false;
  bool     set_distance    = false;
  bool     set_duration    = false;
  float    sel_heading;
  uint16_t sel_distance_cm;
  int16_t  sel_speed_cms;
  uint16_t sel_duration_ms;
  bool          has_init = false;
  unsigned long start_ms;
  double        count_cm = 0;
} navigation_manager;

struct ObjectOfInterest {
  int      ID;
  int      xCenter;
  int      yCenter;
  int      width;
  int      height;
  float    size_cm;
  bool     detected  = false;
  bool     estimated = false;
  uint16_t distance;
  uint16_t bearing;
} follower_obj, target_obj;

// ========= MOTOR + PID =========
float motor_left_setpoint, motor_left_input, motor_left_output;
float motor_right_setpoint, motor_right_input, motor_right_output;
unsigned int motor_left_rpm[2], motor_right_rpm[2];
bool left_idx = false, right_idx = false;

bool  must_correct_heading = false;
float cur_heading = 0.0f, temp_heading = 0.0f;
float heading_input, heading_output, heading_setpoint;

uint8_t motor_left_pwm = 0, motor_right_pwm = 0;
bool motor_left_forward = true, motor_right_forward = true;

float current_speed = 0.0;

double Kp_motor = 3.0, Ki_motor = 2.8, Kd_motor = 0.0;
double Kp_heading = 1.5, Ki_heading = 0.8, Kd_heading = 0.0000001;

QuickPID pid_left(&motor_left_input,   &motor_left_output,   &motor_left_setpoint);
QuickPID pid_right(&motor_right_input, &motor_right_output,  &motor_right_setpoint);
QuickPID pid_heading(&heading_input,   &heading_output,      &heading_setpoint);

volatile unsigned long motor_left_t_cur  = 0, motor_left_t_prev  = 0, motor_left_dur;
volatile unsigned long motor_right_t_cur = 0, motor_right_t_prev = 0, motor_right_dur;

// IR
bool obstacle_left = false, obstacle_right = false, obstacle_forward = false;

// ===================================

void LEFT_RPM_ISR(void) {
  motor_left_t_cur  = micros();
  motor_left_dur    = motor_left_t_cur - motor_left_t_prev;
  motor_left_t_prev = motor_left_t_cur;
}

void RIGHT_RPM_ISR(void) {
  motor_right_t_cur  = micros();
  motor_right_dur    = motor_right_t_cur - motor_right_t_prev;
  motor_right_t_prev = motor_right_t_cur;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {

  // Motor pins
  pinMode(MOTORS_STBY_PIN,      OUTPUT);
  pinMode(MOTORS_LEFT_IN1_PIN,  OUTPUT);
  pinMode(MOTORS_LEFT_IN2_PIN,  OUTPUT);
  pinMode(MOTORS_RIGHT_IN1_PIN, OUTPUT);
  pinMode(MOTORS_RIGHT_IN2_PIN, OUTPUT);

  digitalWrite(MOTORS_STBY_PIN,      LOW);
  digitalWrite(MOTORS_LEFT_IN1_PIN,  LOW);
  digitalWrite(MOTORS_LEFT_IN2_PIN,  LOW);
  digitalWrite(MOTORS_RIGHT_IN1_PIN, LOW);
  digitalWrite(MOTORS_RIGHT_IN2_PIN, LOW);

  // IR pins
  pinMode(IR_LEFT_PIN,    INPUT);
  pinMode(IR_RIGHT_PIN,   INPUT);
  pinMode(IR_FORWARD_PIN, INPUT);

  // Encoder pins
  pinMode(MOTOR_LEFT_ENC_PIN,  INPUT);
  pinMode(MOTOR_RIGHT_ENC_PIN, INPUT);

  // Servo
  huskyServo.attach(SERVO_PIN);
  huskyServo.write(SERVO_ZERO_POS);
  servo_pos        = SERVO_ZERO_POS;
  servo_clockwise  = true;

  // Init variables
  cur_state  = State::INIT_STATE;
  req_state  = State::EMPTY_STATE;
  husky_state = HuskyState::HUSKY_STOPPED_STATE;

  follower_obj.ID      = FOLLOWER_ID;
  follower_obj.size_cm = 23.0;
  target_obj.ID        = TARGET_ID;
  target_obj.size_cm   = 15.5;

  heading_setpoint     = 0;
  heading_input        = 0;
  heading_output       = 0;
  motor_left_setpoint  = 0;
  motor_right_setpoint = 0;
  motor_left_pwm       = 0;
  motor_right_pwm      = 0;
  motor_left_forward   = true;
  motor_right_forward  = true;

  motor_left_rpm[0]  = 0; motor_left_rpm[1]  = 0;
  motor_right_rpm[0] = 0; motor_right_rpm[1] = 0;
  left_idx = false; right_idx = false;

  prev_timestamp = millis();

  Serial.begin(115200);
  Wire.begin();

  // Compass init
  #ifdef USE_BNO055
  while (!bno.begin()) {
    Serial.println(F("BNO055 not found!"));
    delay(1000);
  }
  #else
  compass.init();
  compass.setSamplingRate(20);
  #endif

  // HuskyLens init
  while (!huskylens.begin(Wire)) {
    Serial.println(F("HuskyLens not found!"));
    delay(100);
  }

  // Radio init
  while (!radio.begin()) {
    Serial.println(F("Radio not responding!"));
    delay(200);
  }

  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.stopListening(address[1]);
  radio.openReadingPipe(1, address[1]);
  radio.startListening();

  Serial.println(F("Radio OK."));

  // Enable motors
  digitalWrite(MOTORS_STBY_PIN, HIGH);

  // Encoder interrupts
  cli();
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENC_PIN),  LEFT_RPM_ISR,  FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENC_PIN), RIGHT_RPM_ISR, FALLING);
  sei();

  // PID setup
  pid_left.SetTunings(Kp_motor, Ki_motor, Kd_motor);
  pid_right.SetTunings(Kp_motor, Ki_motor, Kd_motor);
  pid_heading.SetTunings(Kp_heading, Ki_heading, Kd_heading);

  pid_left.SetMode(pid_left.Control::automatic);
  pid_right.SetMode(pid_right.Control::automatic);
  pid_heading.SetMode(pid_heading.Control::automatic);

  pid_left.SetAntiWindupMode(pid_left.iAwMode::iAwClamp);
  pid_right.SetAntiWindupMode(pid_right.iAwMode::iAwClamp);

  pid_left.SetSampleTimeUs(25000);
  pid_right.SetSampleTimeUs(25000);
  pid_heading.SetSampleTimeUs(25000);

  pid_left.SetOutputLimits(0, MAX_PWM);
  pid_right.SetOutputLimits(0, MAX_PWM);
  pid_heading.SetOutputLimits(-MAX_RPM, MAX_RPM);

  // Start with Object Classification
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
  color_recognition_active = false;
  Serial.println(F("Vision: Object Classification"));

  delay(1000);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {

  // Read IR
  obstacle_left    = (analogRead(IR_LEFT_PIN)    <= IR_THRESHOLD);
  obstacle_right   = (analogRead(IR_RIGHT_PIN)   <= IR_THRESHOLD);
  obstacle_forward = (analogRead(IR_FORWARD_PIN) <= IR_THRESHOLD);

  // Read compass
  temp_heading = readCompass();
  if (temp_heading >= 0) cur_heading = temp_heading;

  // Reset heading setpoint (NOT input/output)
  heading_setpoint     = 0.0;
  motor_left_setpoint  = 0.0;
  motor_right_setpoint = 0.0;
  must_correct_heading = false;

  delay(1);

  // ---- RX: Incoming messages from Interface ----
  uint8_t pipe;
  if (radio.available(&pipe)) {

    uint8_t n_bytes = radio.getDynamicPayloadSize();

    if (n_bytes == 5) {

      radio.read(&simple_payload, sizeof(simple_payload));

      if (simple_payload.header.start_bytes == StartMsg::MSG_SIMPLE_START_BYTES) {

        switch (simple_payload.action_state) {

          case Messages::HEARTBEAT_MSG: {

            Header tmp_header;
            tmp_header.start_bytes = StartMsg::MSG_STATUS_START_BYTES;
            tmp_header.sender_id   = 1;
            tmp_header.receiver_id = simple_payload.header.sender_id;

            status_payload.header          = tmp_header;
            status_payload.state           = cur_state;
            status_payload.current_heading = uint16_t(cur_heading * 100);
            status_payload.current_speed   = int16_t(current_speed);

            radio.writeAckPayload(1, &status_payload, sizeof(status_payload));
            break;
          }

          case Messages::STOP_MSG: {
            req_state = State::STOPPED_STATE;
            break;
          }

          case Messages::ON_THE_MOVE_MSG: {
            req_state = State::ON_THE_MOVE_STATE;
            break;
          }

          case Messages::CALIBRATE_COMPASS_MSG: {
            req_state = State::COMPASS_CALIBRATION_STATE;
            break;
          }
        }
      }

    } else if (n_bytes == 11) {

      radio.read(&nav_payload, sizeof(nav_payload));

      if (nav_payload.header.start_bytes == StartMsg::MSG_NAVIGATION_START_BYTES) {

        req_state = State::NAVIGATION_STATE;

        navigation_manager.flags           = nav_payload.flags;
        navigation_manager.set_col_avoid   = ((nav_payload.flags & NavigationFlags::COLLISION_AVOIDANCE_FLAG) > 0);
        navigation_manager.set_rel_heading = ((nav_payload.flags & NavigationFlags::HEADING_RELATIVE_FLAG) > 0);
        navigation_manager.set_distance    = ((nav_payload.flags & NavigationFlags::DISTANCE_FLAG) > 0);
        navigation_manager.set_speed       = ((nav_payload.flags & NavigationFlags::SPEED_FLAG) > 0);
        navigation_manager.set_duration    = ((nav_payload.flags & NavigationFlags::DURATION_FLAG) > 0) && (!navigation_manager.set_distance);

        if (navigation_manager.set_rel_heading) {
          navigation_manager.sel_heading  = float(nav_payload.heading_dd) / 10.0f;
          navigation_manager.sel_heading += cur_heading;
          if (navigation_manager.sel_heading < 0)   navigation_manager.sel_heading += 360;
          if (navigation_manager.sel_heading > 360)  navigation_manager.sel_heading -= 360;
        } else {
          navigation_manager.sel_heading = float(nav_payload.heading_dd) / 10.0f;
        }

        if (navigation_manager.set_distance) navigation_manager.sel_distance_cm = nav_payload.dist_cm_dur_ms;
        if (navigation_manager.set_speed)    navigation_manager.sel_speed_cms   = nav_payload.speed_cms;
        if (navigation_manager.set_duration) navigation_manager.sel_duration_ms = nav_payload.dist_cm_dur_ms;

        navigation_manager.count_cm  = 0;
        navigation_manager.has_init  = false;
      }

    } else if (n_bytes == 9) {

      radio.read(&attack_payload, sizeof(attack_payload));

      if (attack_payload.header.start_bytes == StartMsg::MSG_ATTACK_START_BYTES) {
        // Leader forwards AttackMessage to Follower
        sendAttackToFollower(
          float(attack_payload.heading_dd) / 10.0f,
          attack_payload.distance_cm,
          attack_payload.flags
        );
      }
    }
  }

  delay(1);

  // ---- Read RPM ----
  motor_left_rpm[left_idx]   = readLeftRPM();
  motor_right_rpm[right_idx] = readRightRPM();

  motor_left_input  = ((motor_left_rpm[left_idx]  == 0) || (motor_left_rpm[left_idx]  > UNUSABLE_RPM)) ? float(motor_left_rpm[!left_idx])   : float(motor_left_rpm[left_idx]);
  motor_right_input = ((motor_right_rpm[right_idx] == 0) || (motor_right_rpm[right_idx] > UNUSABLE_RPM)) ? float(motor_right_rpm[!right_idx]) : float(motor_right_rpm[right_idx]);

  left_idx  = !left_idx;
  right_idx = !right_idx;

  current_speed = (motor_left_input + motor_right_input) / 2.0f;

  // Timestamp
  cur_timestamp = millis();
  time_diff     = cur_timestamp - prev_timestamp;

  // ============================================================
  //  MOTOR FSM
  // ============================================================
  switch (cur_state) {

    case State::INIT_STATE: {
      cur_state = State::STOPPED_STATE;
      setLeftSpeed(0, true);
      setRightSpeed(0, true);
      break;
    }

    case State::ERROR_STATE: {
      motor_left_setpoint  = 0;
      motor_right_setpoint = 0;
      setLeftSpeed(0, true);
      setRightSpeed(0, true);
      break;
    }

    case State::STOPPED_STATE: {
      resetMotorLeftPID();
      resetMotorRightPID();
      resetHeadingPID();
      break;
    }

    case State::ON_THE_MOVE_STATE: {

      must_correct_heading = true;

      if (!on_the_move_manager.has_init) {
        on_the_move_manager.sel_heading  = random(MOVE_ANGLE_MIN, MOVE_ANGLE_MAX);
        on_the_move_manager.sel_heading += cur_heading;
        if (on_the_move_manager.sel_heading < 0)   on_the_move_manager.sel_heading += 360;
        if (on_the_move_manager.sel_heading > 360) on_the_move_manager.sel_heading -= 360;
        on_the_move_manager.start_ms  = cur_timestamp;
        on_the_move_manager.has_init  = true;
        motor_left_setpoint  = DEFAULT_NAV_RPM;
        motor_right_setpoint = DEFAULT_NAV_RPM;
        motor_left_forward   = true;
        motor_right_forward  = true;
        resetHeadingPID();
      }

      if ((cur_timestamp - on_the_move_manager.start_ms) >= MOVE_DURATION_MS) {
        on_the_move_manager.has_init = false;
      }

      motor_left_setpoint  = DEFAULT_NAV_RPM;
      motor_right_setpoint = DEFAULT_NAV_RPM;
      motor_left_forward   = true;
      motor_right_forward  = true;

      double tmp_diff = double(on_the_move_manager.sel_heading) - double(cur_heading);
      if ((abs(tmp_diff) > 180) && (tmp_diff < 0)) tmp_diff += 360.0;
      else if ((abs(tmp_diff) > 180) && (tmp_diff > 0)) tmp_diff -= 360.0;
      heading_input = tmp_diff;

      break;
    }

    case State::NAVIGATION_STATE: {

      must_correct_heading = true;

      double tmp_diff = double(navigation_manager.sel_heading) - double(cur_heading);
      if ((abs(tmp_diff) > 180) && (tmp_diff < 0)) tmp_diff += 360.0;
      else if ((abs(tmp_diff) > 180) && (tmp_diff > 0)) tmp_diff -= 360.0;
      heading_input = tmp_diff;

      if (!navigation_manager.has_init) {
        resetMotorLeftPID();
        resetMotorRightPID();
        resetHeadingPID();
        navigation_manager.start_ms = cur_timestamp;
        navigation_manager.count_cm = 0;
        navigation_manager.has_init = true;
      }

      if (navigation_manager.set_speed) {
        bool is_reverse         = (navigation_manager.sel_speed_cms < 0);
        float tmp_rpm           = abs((float)navigation_manager.sel_speed_cms * CM_S_TO_RPM);
        motor_left_setpoint     = tmp_rpm;
        motor_right_setpoint    = tmp_rpm;
        motor_left_forward      = !is_reverse;
        motor_right_forward     = !is_reverse;
      } else {
        motor_left_setpoint  = DEFAULT_NAV_RPM;
        motor_right_setpoint = DEFAULT_NAV_RPM;
        motor_left_forward   = true;
        motor_right_forward  = true;
      }

      if (navigation_manager.set_distance) {
        if (navigation_manager.has_init) {
          double dx = 14.137166941 * double(current_speed) * double(time_diff) / 60000.0;
          navigation_manager.count_cm += dx;
          if (navigation_manager.count_cm >= navigation_manager.sel_distance_cm) {
            navigation_manager.has_init = false;
            cur_state = State::STOPPED_STATE;
            break;
          }
        }
      }

      if (navigation_manager.set_duration) {
        if (navigation_manager.has_init) {
          if ((cur_timestamp - navigation_manager.start_ms) >= navigation_manager.sel_duration_ms) {
            navigation_manager.has_init = false;
            cur_state = State::STOPPED_STATE;
          }
        }
      }

      break;
    }

    case State::COMPASS_CALIBRATION_STATE: {

      if (!calibrate_compass_manager.has_init) {
        calibrate_compass_manager.start_time_ms = cur_timestamp;
        calibrate_compass_manager.has_init      = true;
        setLeftSpeed(DEFAULT_CALIB_RPM,  true);
        setRightSpeed(DEFAULT_CALIB_RPM, false);
        Serial.println(F("Calibration spin started."));
      }

      if ((cur_timestamp - calibrate_compass_manager.start_time_ms) >= calibrate_compass_manager.sel_duration_ms) {
        setLeftSpeed(0,  true);
        setRightSpeed(0, true);
        calibrate_compass_manager.has_init = false;
        cur_state = State::STOPPED_STATE;
        Serial.println(F("Calibration done."));
      }

      break;
    }

    default: {
      cur_state = State::STOPPED_STATE;
      setLeftSpeed(0, true);
      setRightSpeed(0, true);
      break;
    }
  }

  delay(1);

  // ---- Heading PID ----
  pid_heading.Compute();

  if (must_correct_heading) {
    double tmp_l = motor_left_setpoint;
    double tmp_r = motor_right_setpoint;
    tmp_l -= heading_output;
    tmp_r += heading_output;
    motor_left_setpoint  = constrain(tmp_l, 0, MAX_RPM);
    motor_right_setpoint = constrain(tmp_r, 0, MAX_RPM);
  }

  motor_left_setpoint  = constrain(motor_left_setpoint,  0, MAX_RPM);
  motor_right_setpoint = constrain(motor_right_setpoint, 0, MAX_RPM);

  pid_left.Compute();
  pid_right.Compute();

  motor_left_pwm  = uint8_t(constrain(motor_left_output,  0, MAX_PWM));
  motor_right_pwm = uint8_t(constrain(motor_right_output, 0, MAX_PWM));

  if (cur_state == State::STOPPED_STATE || cur_state == State::ERROR_STATE) {
    motor_left_pwm  = 0;
    motor_right_pwm = 0;
    resetMotorLeftPID();
    resetMotorRightPID();
    resetHeadingPID();
  }

  setLeftSpeed(motor_left_pwm,   motor_left_forward);
  setRightSpeed(motor_right_pwm, motor_right_forward);

  delay(1);

  // ---- State change request ----
  if (req_state != State::EMPTY_STATE) {
    calibrate_compass_manager.has_init  = false;
    navigation_manager.has_init         = false;
    on_the_move_manager.has_init        = false;
    cur_state = req_state;
    req_state = State::EMPTY_STATE;
  }

  prev_timestamp = cur_timestamp;

  // ============================================================
  //  HUSKYLENS FSM
  // ============================================================
  huskyServo.write(servo_pos);
  delay(15);

  follower_obj.detected = false;
  target_obj.detected   = false;

  if (!huskylens.request()) {

    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));

  } else if (!huskylens.isLearned()) {

    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));

  } else if (!color_recognition_active) {

    // ---- Object Classification mode ----
    if (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();

      /*
      Serial.print(F("OBJ CLASS detected ID: "));
      Serial.println(result.ID);
      */

      if (result.ID == TARGET_ID) {
        Serial.println(F("Robot detected! Switching to Color Recognition..."));
        huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
        delay(200);
        color_recognition_active = true;
        husky_state = HuskyState::LOCATE_FOLLOWER_STATE;
        Serial.println(F("Vision: Color Recognition"));
      }
    }

  } else {

    // ---- Color Recognition mode ----
    if (!huskylens.available()) {
      Serial.println(F("Lost all targets. Back to Object Classification."));
      huskylens.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
      delay(200);
      color_recognition_active = false;
      husky_state = HuskyState::HUSKY_STOPPED_STATE;
    } else {
      while (huskylens.available()) {
        HUSKYLENSResult result = huskylens.read();
        parseResult(result);
      }
    }
  }

  switch (husky_state) {

    case HuskyState::HUSKY_STOPPED_STATE: {
      husky_state = HuskyState::LOCATE_FOLLOWER_STATE;
      break;
    }

    case HuskyState::LOCATE_FOLLOWER_STATE: {

      if (follower_obj.detected) {
        bool follower_left  = is_left(follower_obj.xCenter,  follower_obj.width);
        bool follower_right = is_right(follower_obj.xCenter, follower_obj.width);

        if ((!follower_left && !follower_right) || (follower_left && follower_right)) {   // Ο Follower βρίσκεται μέσα στο frame
          Serial.print(F("\nFollower:"));
          Serial.print(F("\tDistance(cm): ")); Serial.print(calcDistanceCm(follower_obj.width, follower_obj.size_cm));
          Serial.print(F("\tAngle: "));        Serial.print(calcAngleDeg(follower_obj.xCenter));
          husky_state = HuskyState::LOCATE_TARGET_STATE;
        } else if (follower_left && !follower_right) {
          servo_clockwise = false;
          servo_pos -= SERVO_STEP;
          servo_pos = constrain(servo_pos, MIN_SERVO_POS, MAX_SERVO_POS);
        } else if (!follower_left && follower_right) {
          servo_clockwise = true;
          servo_pos += SERVO_STEP;
          servo_pos = constrain(servo_pos, MIN_SERVO_POS, MAX_SERVO_POS);
        }
      } else {
        if (servo_clockwise) {
          servo_pos += SERVO_STEP;
          servo_pos = constrain(servo_pos, MIN_SERVO_POS, MAX_SERVO_POS);
          if (servo_pos >= MAX_SERVO_POS) servo_clockwise = false;
        } else {
          servo_pos -= SERVO_STEP;
          servo_pos = constrain(servo_pos, MIN_SERVO_POS, MAX_SERVO_POS);
          if (servo_pos <= MIN_SERVO_POS) servo_clockwise = true;
        }
      }
      break;
    }

    case HuskyState::LOCATE_TARGET_STATE: {

      if (target_obj.detected) {
        bool target_left  = is_left(target_obj.xCenter,  target_obj.width);
        bool target_right = is_right(target_obj.xCenter, target_obj.width);

        if ((!target_left && !target_right) || (target_left && target_right)) {    // Ο Target βρίσκεται μέσα στο frame
          Serial.print(F("\nTarget:"));
          Serial.print(F("\tDistance(cm): ")); Serial.print(calcDistanceCm(target_obj.width, target_obj.size_cm));
          Serial.print(F("\tAngle: "));        Serial.print(calcAngleDeg(target_obj.xCenter));
          sendAttackToFollower(calcAngleDeg(target_obj.xCenter), calcDistanceCm(target_obj.width, target_obj.size_cm), ATTACK_HEADING_RELATIVE_FLAG | ATTACK_DISTANCE_FLAG);
          husky_state = HuskyState::LOCATE_FOLLOWER_STATE;
        } else if (target_left && !target_right) {
          servo_clockwise = false;
          servo_pos -= SERVO_STEP;
          servo_pos = constrain(servo_pos, MIN_SERVO_POS, MAX_SERVO_POS);
        } else if (!target_left && target_right) {
          servo_clockwise = true;
          servo_pos += SERVO_STEP;
          servo_pos = constrain(servo_pos, MIN_SERVO_POS, MAX_SERVO_POS);
        }
      } else {
        if (servo_clockwise) {
          servo_pos += SERVO_STEP;
          servo_pos = constrain(servo_pos, MIN_SERVO_POS, MAX_SERVO_POS);
          if (servo_pos >= MAX_SERVO_POS) servo_clockwise = false;
        } else {
          servo_pos -= SERVO_STEP;
          servo_pos = constrain(servo_pos, MIN_SERVO_POS, MAX_SERVO_POS);
          if (servo_pos <= MIN_SERVO_POS) servo_clockwise = true;
        }
      }
      break;
    }

    case HuskyState::BOTH_ON_FRAME_STATE: {
      break;
    }
  }
}

// ============================================================
//  RF SEND FUNCTIONS
// ============================================================

void sendAttackToFollower(float angle_deg, float distance_cm, uint8_t flags) {

  radio.stopListening();
  radio.openWritingPipe(address[2]);

  Header h;
  h.start_bytes  = MSG_ATTACK_START_BYTES;
  h.sender_id    = 1;
  h.receiver_id  = 2;

  attack_payload.header      = h;
  attack_payload.flags       = flags;
  attack_payload.heading_dd  = (int16_t)(angle_deg * 10.0f);
  attack_payload.distance_cm = (uint16_t)distance_cm;

  radio.write(&attack_payload, sizeof(attack_payload));
  radio.startListening();

  Serial.print(F("→ Follower ATTACK angle: "));
  Serial.print(angle_deg);
  Serial.print(F("° dist: "));
  Serial.print(distance_cm);
  Serial.println(F(" cm"));
}

// ============================================================
//  MOTOR CONTROL
// ============================================================

void setLeftSpeed(uint8_t val, bool forward) {
  if (forward) {
    val = 0xFF - val;
    digitalWrite(MOTORS_LEFT_IN1_PIN, HIGH);
    analogWrite(MOTORS_LEFT_IN2_PIN,  val);
  } else {
    digitalWrite(MOTORS_LEFT_IN1_PIN, LOW);
    analogWrite(MOTORS_LEFT_IN2_PIN,  val);
  }
}

void setRightSpeed(uint8_t val, bool forward) {
  if (forward) {
    val = 0xFF - val;
    digitalWrite(MOTORS_RIGHT_IN1_PIN, HIGH);
    analogWrite(MOTORS_RIGHT_IN2_PIN,  val);
  } else {
    digitalWrite(MOTORS_RIGHT_IN1_PIN, LOW);
    analogWrite(MOTORS_RIGHT_IN2_PIN,  val);
  }
}

unsigned int readRightRPM() {
  unsigned long t_now = micros();
  if (t_now - motor_right_t_cur >= RPM_TIMEOUT_US) return 0;
  return (unsigned int)((60000000UL) / (motor_right_dur * MOTOR_ENCODER_N));
}

unsigned int readLeftRPM() {
  unsigned long t_now = micros();
  if ((t_now - motor_left_t_cur) >= RPM_TIMEOUT_US) return 0;
  return (unsigned int)((60000000UL) / (motor_left_dur * MOTOR_ENCODER_N));
}

void resetMotorLeftPID()  { pid_left.Reset();    }
void resetMotorRightPID() { pid_right.Reset();   }
void resetHeadingPID()    { pid_heading.Reset(); }

// ============================================================
//  VISION
// ============================================================

float calcAngleDeg(int obj_x) {
  return float(obj_x - HUSKY_WIDTH / 2) * PX_DEG;
}

float calcDistanceCm(uint16_t width_px, uint16_t dim_cm) {
  if (width_px == 0) return 0;
  return (float(dim_cm) * float(PX_CM)) / float(width_px);
}

bool is_left(int x_center, int width_px) {
  return ((x_center - (width_px / 2)) <= WIDTH_THRES);
}

bool is_right(int x_center, int width_px) {
  return ((x_center + (width_px / 2)) >= (HUSKY_WIDTH - WIDTH_THRES));
}

void parseResult(HUSKYLENSResult result) {
  switch (result.ID) {
    case FOLLOWER_ID:
      follower_obj.detected = true;
      follower_obj.xCenter  = result.xCenter;
      follower_obj.yCenter  = result.yCenter;
      follower_obj.width    = result.width;
      follower_obj.height   = result.height;
      break;
    case TARGET_ID:
      target_obj.detected = true;
      target_obj.xCenter  = result.xCenter;
      target_obj.yCenter  = result.yCenter;
      target_obj.width    = result.width;
      target_obj.height   = result.height;
      break;
    default: return;
  }
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK)
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  else if (result.command == COMMAND_RETURN_ARROW)
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  else
    Serial.println(F("Object unknown!"));
}

// ============================================================
//  COMPASS
// ============================================================

float readCompass() {
  float heading = -1;

  for (uint8_t count = 0; count < COMPASS_MAX_RETRIES; count++) {

    #ifdef USE_BNO055
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    if (orientationData.type == SENSOR_TYPE_ORIENTATION) {
      heading  = orientationData.orientation.x;
      heading += BNO055_OFFSET;
      if (heading < 0)      heading += 360.0f;
      if (heading > 360.0f) heading -= 360.0f;
      break;
    }
    #else
    if (compass.ready()) {
      heading  = float(compass.readHeading());
      heading += ONBOARD_COMPASS_OFFSET;
      if (heading < 0)      heading += 360.0f;
      if (heading > 360.0f) heading -= 360.0f;
      break;
    } else {
      heading = -1.0f;
    }
    #endif
  }
  return heading;
}
