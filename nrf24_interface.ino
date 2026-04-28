/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another
 * with Acknowledgement (ACK) payloads attached to ACK packets.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 7
#define CSN_PIN 8

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// an identifying device destination
// Let these addresses be used for the pair
uint8_t address[][6] = { "Node0", "Node1", "Node2" };
String  device_name[3] = { "Interface", "Master", "Follower" };

// Node 0 -> Interface
// Node 1 -> Master
// Node 2 -> Follower

// It is very helpful to think of an address as a path instead of as
// an identifying device destination
// to use different addresses on a pair of radios, we need a variable to

// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

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

// ===========================

// ========= MESSAGES =========
// MAKE THE MOST COMMON CASE FAST !
struct Header {
  
  uint16_t start_bytes;
  uint8_t sender_id;
  uint8_t receiver_id;
};

struct SimpleMessage {      // 5 bytes Just a header with an action or state
  
  Header header;
  uint8_t action_state;       // 1 byte e.g FOLLOW_HEADING_SOFT, FOLLOW_HEADING_HARD

} simple_payload;

struct NavigationMessage {  // 11 bytes

  Header header;       
  uint8_t flags;            // 1 byte 
  int16_t heading_dd;      // 2 bytes Always positive Degrees *10 e.g 0 -> 3600 ==> 0 -> 360
  int16_t speed_cms;        // 2 bytes Can be negative
  uint16_t dist_cm_dur_ms;  // 2 bytes Either for distance or duration Always positive int

} nav_payload;

struct AttackMessage {      // 9 bytes

  Header header;
  uint8_t flags;
  int16_t heading_dd;
  uint16_t distance_cm;

} attack_payload;

struct StatusMessage {      // 9
  
  Header header;
  uint8_t state;      // 1 byte e.g INIT_STATE, ERROR_STATE
  uint16_t current_heading;
  int16_t current_speed;  

} status_payload;
// ============================

void setup() {

  // In order to inteface, we need a Serial port
  Serial.begin(115200);
  
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
    delay(1);
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();  // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  // set the TX address of the RX node for use on the TX pipe (pipe 0)
  radio.stopListening(address[radioNumber]);  // put radio in TX mode
 
  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

  //delay(2000);
  //send_defined_attack_msg(2, 0, true); // 
}

void loop() {

  uint8_t num_sel = 0;

  // Prompt user to select interfacing device
  while(num_sel == 0){
    // Show initital menu and read selection
    num_sel = select_device_menu();
    delay(2);
  }

  bool exit_loop_main_menu = false;
  
  while(!exit_loop_main_menu){
    
    switch(num_sel){
  
      // Master
      case 1:{
  
        // Inner selection
        int menu_master_options_sel = 0;
  
        // Prompt user to select master action
        while(menu_master_options_sel == 0){
          // Show menu and read selection
          menu_master_options_sel = select_master_action_menu();
          delay(2);
        }
  
        // Previous menu
        if (menu_master_options_sel == 10) {
          exit_loop_main_menu = true;
          break;
        }
  
        switch(menu_master_options_sel){
  
          // View telemetry
          case 1: {
  
            Serial.println("\nSend 1 to stop receiving and return...");
       
            while(true){
          
              // Send heartbeat to master and read status
              send_heartbeat_msg(1);
      
              // Check for possible exit request 
              if(get_int_clean(false) == 1) break;
                    
            }
  
            break;
          }
  
          // Start AUTO
          case 2: {
  
            break;
          }
  
          // Stop
          case 3: {

            send_simple_msg(1, Messages::STOP_MSG);

            break;
          }
  
          // Stay on the move
          case 4: {

            send_simple_msg(1, Messages::ON_THE_MOVE_MSG);
  
            break;
          }
  
          // Calibrate compass
          case 5: {

            send_simple_msg(1, Messages::CALIBRATE_COMPASS_MSG);
  
            break;
          }
  
          // Navigate
          case 6: {

            send_navigation_msg(1);
  
            break;
          }
  
          // Attack
          case 7: {

            send_attack_msg(1);
  
            break;
          }
  
          // Search for target
          case 8: {
  
            break;
          }
  
          // Search for Follower
          case 9: {
  
            break;
          }
          
        }
  
        break;
      }
  
      // Follower
      case 2:{
  
        // Selection
        int menu_follower_options_sel = 0;
  
        // Prompt user to select follower action
        while(menu_follower_options_sel == 0){
          // Show menu and read selection
          menu_follower_options_sel = select_follower_action_menu();
          delay(2);
        }
  
        // Previous menu
        if (menu_follower_options_sel == 7) {
          exit_loop_main_menu = true;
          break;
        }
  
        switch(menu_follower_options_sel){
  
          // View telemetry
          case 1: {
  
            Serial.println(F("\nSend 1 to stop receiving and return..."));
  
            while(true){

              // Send heartbeat to follower and read status
              send_heartbeat_msg(2);
      
              // Check for possible exit request 
              if(get_int_clean(false) == 1) break;
             
            }
  
            break;
          }
  
          // Stop
          case 2: {
  
            send_simple_msg(2, Messages::STOP_MSG);
      
            break;
          }
  
          // Stay on the move
          case 3: {

            send_simple_msg(2, Messages::ON_THE_MOVE_MSG);
  
            break;
          }
  
          // Calibrate compass
          case 4: {
  
            send_simple_msg(2, Messages::CALIBRATE_COMPASS_MSG);
  
            break;
          }
  
          // Navigate
          case 5: {

            send_navigation_msg(2);
  
            break;
          }
  
          // Attack
          case 6: {

            send_attack_msg(2);
  
            break;
          }
          
        }
        
        break;
      }
  
      // View telemetries
      case 3:{
  
        Serial.println(F("\nSend 1 to stop receiving and return..."));

        while(true){

          // Send heartbeat to Master and read status
          send_heartbeat_msg(1);

          // Send heartbeat to Follower and read status
          send_heartbeat_msg(2);
          
          // Check for possible exit request 
          if(get_int_clean(false) == 1) {
            exit_loop_main_menu = true;
            break;
          }          
        }
        break;
      }
    }  

    delay(2);
  } 
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

void send_simple_msg(uint8_t receiver_id, uint8_t msg_t){

  // Prepare Header payload
  Header tmp_header;
  
  // Set receiving address
  radio.openWritingPipe(address[receiver_id]);
  
  delay(2);
  
  // Prepare header
  tmp_header.start_bytes = StartMsg::MSG_SIMPLE_START_BYTES;
  tmp_header.sender_id = 0;
  tmp_header.receiver_id = receiver_id;
  
  // Prepare payload
  simple_payload.header = tmp_header;
  simple_payload.action_state = msg_t;
  
  // Send payload
  radio.write(&simple_payload, sizeof(simple_payload));
}

void send_heartbeat_msg(uint8_t receiver_id){

  bool report = false;
  
  // Prepare Header payload
  Header tmp_header;
  
  // In order to send to Master
  radio.openWritingPipe(address[receiver_id]);
  
  delay(2);
  
  // Prepare header
  tmp_header.start_bytes = StartMsg::MSG_SIMPLE_START_BYTES;
  tmp_header.sender_id = 0;
  tmp_header.receiver_id = receiver_id;
  
  // Prepare payload
  simple_payload.header = tmp_header;
  simple_payload.action_state = Messages::HEARTBEAT_MSG;
  
  // Send payload
  report = radio.write(&simple_payload, sizeof(simple_payload));
    
  if (report) {
    
    delay(2);
                 
    uint8_t pipe;
    if (radio.available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
            
      radio.read(&status_payload, sizeof(status_payload));  // get incoming ACK payload

      uint8_t tmp_state = status_payload.state;
      float tmp_heading = float(status_payload.current_heading)/100.0f;
      float tmp_speed = float(status_payload.current_speed);
      
      Serial.print(F("["));
      Serial.print(device_name[receiver_id]);
      Serial.print(F("] State: "));
      Serial.print(state_str(tmp_state));
      Serial.print(F("\tHeading: "));
      Serial.print(tmp_heading); 
      Serial.print(F("\tSpeed: ")); 
      Serial.println(tmp_speed);
   }
  }
}

// To assist the follower you must call send_defined_attack_msg(2, ...,)
void send_defined_attack_msg(uint8_t receiver_id, uint16_t tmp_head, bool is_relative){

  // Prepare Header payload
  Header tmp_header;
  
  // In order to send to Follower
  radio.openWritingPipe(address[receiver_id]); // receiver_id (Follower) == 2
  
  // Prepare header
  tmp_header.start_bytes = StartMsg::MSG_ATTACK_START_BYTES;
  tmp_header.sender_id   = 0;
  tmp_header.receiver_id = receiver_id;
  
  // Prepare payload
  attack_payload.header       = tmp_header;
  attack_payload.flags        = 0;
  attack_payload.heading_dd   = 0;          
  attack_payload.distance_cm  = 0; 

  if(is_relative) attack_payload.flags |= ATTACK_HEADING_RELATIVE_FLAG;
    
  attack_payload.heading_dd = (int16_t)(tmp_head*10); // Must be 0 -> 360

  // Send Message
  radio.write(&attack_payload, sizeof(attack_payload));
  
  return;
}

// To assist the follower you must call send_defined_attack_msg(2, ..., ...)
void send_defined_attack_msg(uint8_t receiver_id, uint16_t tmp_head, bool is_relative, uint16_t tmp_dist_cm){

  // Prepare Header payload
  Header tmp_header;
  
  // In order to send to Follower
  radio.openWritingPipe(address[receiver_id]); // receiver_id (Follower) == 2
  
  // Prepare header
  tmp_header.start_bytes = StartMsg::MSG_ATTACK_START_BYTES;
  tmp_header.sender_id   = 0;
  tmp_header.receiver_id = receiver_id;
  
  // Prepare payload
  attack_payload.header       = tmp_header;
  attack_payload.flags        = 0;
  attack_payload.heading_dd   = 0;          
  attack_payload.distance_cm  = 0; 
  
  attack_payload.heading_dd = (int16_t)(tmp_head*10); // Must be 0 -> 360
  attack_payload.distance_cm = tmp_dist_cm;

  if(is_relative) attack_payload.flags |= ATTACK_HEADING_RELATIVE_FLAG;
  
  attack_payload.flags |= AttackFlags::ATTACK_DISTANCE_FLAG;

  // Send Message
  radio.write(&attack_payload, sizeof(attack_payload));
  
  return;
}


void send_attack_msg(uint8_t receiver_id){

  int menu_sel  = 0;
  int tmp_sel   = 0;

  // Prepare Header payload
  Header tmp_header;
  
  // In order to send to Master
  radio.openWritingPipe(address[receiver_id]);
  
  delay(2);

  // Prepare header
  tmp_header.start_bytes = StartMsg::MSG_ATTACK_START_BYTES;
  tmp_header.sender_id   = 0;
  tmp_header.receiver_id = receiver_id;
  
  // Prepare payload
  attack_payload.header       = tmp_header;
  attack_payload.flags        = 0;
  attack_payload.heading_dd   = 0;          
  attack_payload.distance_cm  = 0; 
  
  do{
    
    Serial.print(F("\nChoose Attack type:"));
    Serial.print(F("\n[1] Select Heading"));
    Serial.print(F("\n[2] Select Heading, Distance"));
    Serial.print(F("\n[3] Previous menu"));
    
    // Get number
    menu_sel = get_int_clean();

    // Must exit
    if(menu_sel == 3) return;

    // Check for wrong input
    if((menu_sel <= 0) || (menu_sel > 3)) Serial.println(F("\nWrong input"));

  }while((menu_sel <= 0) || (menu_sel > 3));


  // Get Heading relative
  do{
     
    Serial.print(F("\nWould you like to express relative heading?\n[0] No\n[1] Yes"));

    // Get collision avoidance
    tmp_sel = get_int_clean();
    
  }while((tmp_sel < 0) || (tmp_sel > 1));
  
  Serial.print(F("\nRelative heading: "));
  Serial.print((tmp_sel == 0) ? "False" : "True");

  // Set relative heading flag if necessary
  if(tmp_sel == 1) attack_payload.flags |= AttackFlags::ATTACK_HEADING_RELATIVE_FLAG; 

  // Get Heading
  do{
 
    Serial.print(F("\nEnter Heading in positive integer number of degrees [0, 359]"));

    // Get Heading
    tmp_sel = get_int_clean();
    
  }while((tmp_sel < 0) || (tmp_sel >= 360)); 

  Serial.print(F("\nSelected Heading: "));
  Serial.print(tmp_sel);

  attack_payload.heading_dd = (int16_t)(tmp_sel*10);

  // Check if we need distance
  if(menu_sel == 2){

    // Get Distance
    do{
      
      Serial.print(F("\nEnter Distance in positive integer number of cm [1, 1000]"));
        
      // Get Distance
      tmp_sel = get_int_clean();
      
    }while((tmp_sel < 1) || (tmp_sel > 1000)); 

    Serial.print(F("\nSelected Distance (cm): "));
    Serial.print(tmp_sel);

    attack_payload.distance_cm = (uint16_t)tmp_sel;
    attack_payload.flags |= AttackFlags::ATTACK_DISTANCE_FLAG;
  }

  // Send Message
  radio.write(&attack_payload, sizeof(attack_payload));
  
  return;
}

void send_navigation_msg(uint8_t receiver_id){

  int menu_sel  = 0;
  int tmp_sel   = 0;

  // Prepare Header payload
  Header tmp_header;
  
  // In order to send to Master
  radio.openWritingPipe(address[receiver_id]);
  
  delay(2);

  // Prepare header
  tmp_header.start_bytes = StartMsg::MSG_NAVIGATION_START_BYTES;
  tmp_header.sender_id   = 0;
  tmp_header.receiver_id = receiver_id;
  
  // Prepare payload
  nav_payload.header = tmp_header;
  nav_payload.flags           = 0;
  nav_payload.heading_dd      = 0;    
  nav_payload.speed_cms       = 0;        
  nav_payload.dist_cm_dur_ms  = 0; 
  
  do{
    
    Serial.print(F("\nChoose Navigation type:"));
    Serial.print(F("\n[1] Select Heading"));
    Serial.print(F("\n[2] Select Heading, Distance"));
    Serial.print(F("\n[3] Select Heading, Distance, Speed"));
    Serial.print(F("\n[4] Select Heading, Speed"));
    Serial.print(F("\n[5] Select Heading, Speed, Duration"));
    Serial.print(F("\n[6] Select Heading, Duration"));
    Serial.print(F("\n[7] Previous menu"));
    
    // Get number
    menu_sel = get_int_clean();

    // Must exit
    if(menu_sel == 7) return;

    // Check for wrong input
    if((menu_sel <= 0) || (menu_sel > 7)) Serial.println(F("\nWrong input"));

  }while((menu_sel <= 0) || (menu_sel > 7));

  // Get collision avoidance
  do{

    Serial.print(F("\nWould you like to enable collision avoidance functionality?\n[0] No\n[1] Yes"));

    // Get collision avoidance
    tmp_sel = get_int_clean();
    
  }while((tmp_sel < 0) || (tmp_sel > 1));
  
  Serial.print(F("\nCollision avoidance: "));
  Serial.print((tmp_sel == 0) ? "Disabled" : "Enabled");

  // Set collision avoidance flag if necessary
  if(tmp_sel == 1) nav_payload.flags |= NavigationFlags::COLLISION_AVOIDANCE_FLAG;

  // Get Heading relative
  do{
     
    Serial.print(F("\nWould you like to express relative heading?\n[0] No\n[1] Yes"));

    // Get collision avoidance
    tmp_sel = get_int_clean();
    
  }while((tmp_sel < 0) || (tmp_sel > 1));
  
  Serial.print(F("\nRelative heading: "));
  Serial.print((tmp_sel == 0) ? "False" : "True");

  // Set relative heading flag if necessary
  if(tmp_sel == 1) nav_payload.flags |= NavigationFlags::HEADING_RELATIVE_FLAG; 

  // Get Heading
  do{
 
    Serial.print(F("\nEnter Heading in positive integer number of degrees [0, 359]"));

    // Get Heading
    tmp_sel = get_int_clean();
    
  }while((tmp_sel < 0) || (tmp_sel >= 360)); 

  Serial.print(F("\nSelected Heading: "));
  Serial.print(tmp_sel);

  nav_payload.heading_dd = (uint16_t)(tmp_sel*10);

  // Check if we need duration
  if((menu_sel == 5) || (menu_sel == 6)){

    // Get Duration
    do{
      
      Serial.print(F("\nEnter Duration in positive integer number of ms [1, 30000]"));
        
      // Get Duration
      tmp_sel = get_int_clean();
      
    }while((tmp_sel < 1) || (tmp_sel > 30000));

    Serial.print(F("\nSelected Duration (ms): "));
    Serial.print(tmp_sel);

    nav_payload.dist_cm_dur_ms = (uint16_t)tmp_sel;
    nav_payload.flags |= NavigationFlags::DURATION_FLAG;   
  }

  // Check if we need distance
  if((menu_sel == 2) || (menu_sel == 3)){

    // Get Distance
    do{
      
      Serial.print(F("\nEnter Distance in positive integer number of cm [1, 1000]"));
        
      // Get Distance
      tmp_sel = get_int_clean();
      
    }while((tmp_sel < 1) || (tmp_sel > 1000)); 

    Serial.print(F("\nSelected Distance (cm): "));
    Serial.print(tmp_sel);

    nav_payload.dist_cm_dur_ms = (uint16_t)tmp_sel;
    nav_payload.flags |= NavigationFlags::DISTANCE_FLAG;
  }

  // Check if we need speed
  if((menu_sel == 3) || (menu_sel == 4) || (menu_sel == 5)){

    // Get Speed
    do{
       
      Serial.print(F("\nEnter Speed in integer number of cm/s [-42, 42]"));
        
      // Get Speed
      tmp_sel = get_int_clean();
      
    }while((tmp_sel < -42) || (tmp_sel > 42));

    Serial.print(F("\nSelected Speed (cm/s): "));
    Serial.print(tmp_sel);

    nav_payload.speed_cms = (int16_t)tmp_sel;
    nav_payload.flags |= NavigationFlags::SPEED_FLAG;   
  }

  // Send Message
  radio.write(&nav_payload, sizeof(nav_payload));
  
  return;
}

// Menu to select interfacing device
uint8_t select_device_menu(){

  Serial.print(F("\nChoose function:"));
  Serial.print(F("\n[1] Interface with Master"));
  Serial.print(F("\n[2] Interface with Follower"));
  Serial.println(F("\n[3] View available telemetries"));

  // Get number
  int num = get_int_clean();

  // Check for wrong input
  if((num <= 0) || (num > 3)){
    Serial.println(F("\nWrong input"));
    return 0;    
  }else{
    return uint8_t(num);    
  }

  return 0;
}

uint8_t select_master_action_menu(){

  Serial.print(F("\nChoose Master action:"));
  Serial.print(F("\n[1] View telemetry"));
  Serial.print(F("\n[2] Start AUTO"));
  Serial.print(F("\n[3] Stop"));
  Serial.print(F("\n[4] Stay on the move"));
  Serial.print(F("\n[5] Calibrate compass"));
  Serial.print(F("\n[6] Navigate"));
  Serial.print(F("\n[7] Attack"));
  Serial.print(F("\n[8] Search for target"));
  Serial.print(F("\n[9] Search for Follower"));
  Serial.println(F("\n[10] Previous menu"));

  // Get number
  int num = get_int_clean();

  // Check for wrong input
  if((num <= 0) || (num > 10)){
    Serial.println(F("\nWrong input"));
    return 0;    
  }else{
    return uint8_t(num);    
  }

  return 0;    
}

// Menu to select device action
uint8_t select_follower_action_menu(){

  Serial.print(F("\nChoose Follower action:"));
  Serial.print(F("\n[1] View telemetry"));
  Serial.print(F("\n[2] Stop"));
  Serial.print(F("\n[3] Stay on the move"));
  Serial.print(F("\n[4] Calibrate compass"));
  Serial.print(F("\n[5] Navigate"));
  Serial.print(F("\n[6] Attack"));
  Serial.println(F("\n[7] Previous menu"));

  // Get number
  int num = get_int_clean();

  // Check for wrong input
  if((num <= 0) || (num > 7)){
    Serial.println(F("\nWrong input"));
    return 0;    
  }else{
    return uint8_t(num);    
  }

  return 0;  
}

// Default get_int_clean() block
int get_int_clean(){

  return get_int_clean(true);  
}

int get_int_clean(bool blocking){

  int num = 0;

  if(blocking){
    // Block until bytes are available
    while(Serial.available() <= 0) delay(10);
  }

  if(Serial.available() > 0){

    // Get number
    num = Serial.parseInt(SKIP_WHITESPACE);
    delay(5);
  }

  // Get rid of the integer following bytes
  while(Serial.available() > 0) {
    delay(1);
    Serial.read();
  }

  return num;  
}
