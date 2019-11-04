/* 
 * Date: 25 April, 2019
 * Author: Atoosa Karimi (akarimi@etalim.com)
 * Owner: Keith Antonelli (kantonelli@etalim.com)
 * Description: Code for the internal bore hole welding jig positioning
 * automation. Structured as a finite state machine with interrupts. <add more description>.
 * Documentation: <insert location of specifications and supporting documents>
 */

//libraries 
#include "PinChangeInterrupt.h"

//pin definitions (buttons and actuators)
#define STOP_BUTTON 3
#define START_BUTTON 4
#define SET_SEAM1_BUTTON 5
#define SEAM1_BUTTON 6
#define SEAM2_BUTTON 7
#define SEAM3_BUTTON 8
#define OPTO_OVERTRAVEL 9
#define OPTO_SYSREF 10
#define SEAM1_LED 13
#define SEAM2_LED 12
#define SEAM3_LED 11
#define STATUS_LED A1
#define ERROR_LED A2
#define JOYSTICK A3


//signal definitions
#define NOTHING_SIG 0
#define START_SIG 1
#define SEAM1_SIG 2
#define SEAM2_SIG 3
#define SEAM3_SIG 4
#define SYSREF_SIG 5

//state definitions
#define _SYSREF 0
#define _IDLE 1
#define _INITIALIZED 2
#define _ERROR 3

//constants
#define COM_LEN 9 //communication with TMCM-1140 is 9 bytes
#define ADC_VOLT_CONVERSION 0.0049 //derived from the ADC and actual voltage relationship of analogRead(pin)
#define JOYSTICK_UPTHRESH 4.2 //in volts
#define JOYSTICK_DOWNTHRESH 0.8 //in volts
#define JOYSTICK_UPPERDELTA 1.2 //in volts
#define JOYSTICK_LOWERDELTA 0.5 //in volts
#define JOYSTICK_MIDRANGE 2.5 //in volts
#define MOTOR_LEAD 2.0L //in mm
#define USTEP_RESOLUTION 32L //consult firmware manual of stepper motor
#define STEPS_PER_REV 200 //consult hardware manual of stepper motor
#define MM_TO_NEXT_SEAM 8.4L //in mm
#define STEPS_TO_NEXT_SEAM float(MM_TO_NEXT_SEAM/MOTOR_LEAD*USTEP_RESOLUTION*STEPS_PER_REV)

//function prototypes
char calculate_checksum(char command[]);
void stop_motor(void);
void set_seam1(void);
void go_seam1(void);
void go_seam2(void);
void go_seam3(void);
void start_fsm(void);
void over_travel(void);
void set_sys_ref(void);
bool check_wip(void);
byte calculate_checksum(byte command[]);
void set_seam_command(byte command[], long location);
void set_other_seams(void);

//TMCL commands for stepper motor - derived from firmware manual
//for specifics refer to the firmware manual or the TMCL IDE
//module address (1 byte), command field (1 byte), type field (1 byte), motor/bank field (1 byte), value (4 bytes), checksum (1 byte)
byte stop_command[]= {0x01,0x03,0x0,0x0,0x0,0x0,0x0,0x0,0x04}; //stops motor
byte is_WIP_command[] = {0x01, 0x06, 0x08, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF}; //accesses the position reached flag
byte scrolldown_command[] = {0x01, 0x04, 0x01, 0x0, 0xFF, 0xFF, 0x9E, 0x58, 0xFA}; //for jogging the motor down - MVP REL, -25000
byte scrollup_command[] = {0x01, 0x04, 0x01, 0x0, 0x0, 0x00, 0x61, 0xA8, 0x0F}; //for jogging the motor up - MVP REL, 25000
byte scrolldownmid_command[] = {0x01, 0x04, 0x01, 0x0, 0xFF, 0xFF, 0xFC, 0x18, 0x18}; //for nudging the motor down - MVP REL, -1000
byte scrollupmid_command[] = {0x01, 0x04, 0x01, 0x0, 0x0, 0x0, 0x03, 0xE8, 0xF1}; //for nudging the motor up - MVP REL, 1000
byte go_seam1_command[] = {0x01, 0x04, 0x02, 0x0, 0x0, 0x0, 0x0, 0x02, 0x09}; //go to coordinate 2 
byte go_seam2_command[] = {0x01, 0x04, 0x02, 0x0, 0x0, 0x0, 0x0, 0x03, 0x0A}; //go to coordinate 3
byte go_seam3_command[] = {0x01, 0x04, 0x02, 0x0, 0x0, 0x0, 0x0, 0x04, 0x0B}; //go to coordinate 4
byte actualpos_command[] = {0x01, 0x06, 0x01, 0x0, 0x0, 0x0, 0x0,0x0, 0x08}; //request the actual position parameter
byte set_seam1_command[] = {0x01, 0x20, 0x02, 0x0, 0x0, 0x0, 0x0, 0x0, 0x23}; //stores current position into coordinate number 2 - associated with seam 1
byte set_seam2_command[] = {0x01, 0x1E, 0x03, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; //store location in coordinate 3 (this command is modified during operation) - associated with seam 2
byte set_seam3_command[] = {0x01, 0x1E, 0x04, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; //store location in coordinate 4 (this command is modified during operation) - associated with seam 3
byte get_coordinate_seam1[] = {0x01, 0x1F, 0x02, 0x0, 0x0, 0x0, 0x0, 0x0, 0x22}; //request positon of coordinate 2 - associated with seam 1
byte set_pos_zero_command[] = {0x01, 0x05, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x07}; //set current position to 0 - used in setting system reference
byte set_target_zero_command[] = {0x01, 0x05, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x06}; //set target position to 0 - used in setting system reference
byte set_encoder_zero_command[]={0x01, 0x05, 0xD1, 0x0, 0x0, 0x0, 0x0, 0x0, 0xD7}; //set encoder position to 0 - used in setting system reference
byte set_EEPROM_storage[] = {0x01, 0x09, 0x54, 0x0, 0x0, 0x0, 0x0, 0x01, 0x5F}; //set global parameter 84 to 1 to store coordinates automatically in EEPROM
byte store_actualpos_in_EEPROM[] = {0x01, 0x07, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x09}; 
byte ROR_command[] = {0x01, 0x01, 0x0, 0x0, 0x0, 0x0, 0x07, 0xFF, 0x08}; //rotate motor right indefinitely with given speed
byte ROL_command[] = {0x01, 0x02, 0x0, 0x0, 0x0, 0x0, 0x07, 0xFF, 0x09}; //rotate motor left indefinitely with given speed

//variables
volatile bool set_seams_flag = false;
volatile bool store_actualpos_in_EEPROM_flag = false;
volatile byte current_state = _SYSREF;
volatile byte new_signal = NOTHING_SIG;
byte current_signal;
byte previous_signal;
byte reply[9];
byte reply_ref[9];
byte reply_test[9];
bool wip;
bool set_other_seams_flag = false;
float joystick_status;
long seam1_location;
long seam2_location;
long seam3_location;
long actual_location;
int i = 0;

void setup() {
  Serial.begin(9600); //set up RS485
  
  //set up buttons
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(SET_SEAM1_BUTTON, INPUT_PULLUP);
  pinMode(SEAM1_BUTTON, INPUT_PULLUP);
  pinMode(SEAM2_BUTTON, INPUT_PULLUP);
  pinMode(SEAM3_BUTTON, INPUT_PULLUP);
  
  //set up opto switches
  pinMode(OPTO_OVERTRAVEL, INPUT_PULLUP);
  pinMode(OPTO_SYSREF, INPUT_PULLUP);
  
  //set up joystick
  pinMode(JOYSTICK, INPUT);

  //set up LEDs
  pinMode(SEAM1_LED, OUTPUT);
  pinMode(SEAM2_LED, OUTPUT);
  pinMode(SEAM3_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  
  //set up pin change interrupts
  attachPCINT(digitalPinToPCINT(STOP_BUTTON), stop_motor, RISING);
  attachPCINT(digitalPinToPCINT(SET_SEAM1_BUTTON), set_seam1, RISING);
  attachPCINT(digitalPinToPCINT(START_BUTTON), start_fsm, RISING);
  attachPCINT(digitalPinToPCINT(SEAM1_BUTTON), go_seam1, RISING);
  attachPCINT(digitalPinToPCINT(SEAM2_BUTTON), go_seam2, RISING); 
  attachPCINT(digitalPinToPCINT(SEAM3_BUTTON), go_seam3, RISING);
  attachPCINT(digitalPinToPCINT(OPTO_OVERTRAVEL), over_travel, RISING);
  attachPCINT(digitalPinToPCINT(OPTO_SYSREF), set_sys_ref, RISING);

}

void loop() {
  noInterrupts();
  current_signal = new_signal;
  interrupts();

  wip = check_wip(); //check if there is work in progress
  joystick_status = analogRead(JOYSTICK)*ADC_VOLT_CONVERSION; //store the voltage read from the joystick

  //whenever the user presses the stop button the current position stored in EEPROM for future reference in case of a power cycle
  if (store_actualpos_in_EEPROM_flag) {
    Serial.write(store_actualpos_in_EEPROM, COM_LEN);
    Serial.flush();
    store_actualpos_in_EEPROM_flag = false;
    }

  //sets the location of seam 1 whenever the user presses the set seam button
  if (set_seams_flag) {
    Serial.write(set_seam1_command,COM_LEN);
    Serial.flush();
    i = 0;
    set_seams_flag = false;
    set_other_seams_flag = true;
    }

  //sets the location of seams 2 and 3, 6 loop times after seam 1 has been set
  if (set_other_seams_flag) {   
    //experimentally determined that ~6 loop times is necessary between setting seam 1 and other seams
    if (i == 5) {
    set_other_seams();
    Serial.write(go_seam1_command,COM_LEN);
    set_other_seams_flag = false;
    }
    i++; 
  }

  //FSM - ordered by most commonly used states
  switch(current_state) {
    case _INITIALIZED:
      //allow jogging, going to seams, setting reference
      digitalWrite(STATUS_LED,HIGH);
      digitalWrite(ERROR_LED,LOW);
      if (joystick_status >= JOYSTICK_UPTHRESH) {
        new_signal = NOTHING_SIG;
        Serial.write(scrollup_command,COM_LEN);
        }
      else if (joystick_status < JOYSTICK_MIDRANGE + JOYSTICK_UPPERDELTA && joystick_status > JOYSTICK_MIDRANGE + JOYSTICK_LOWERDELTA) {
        new_signal = NOTHING_SIG;
        Serial.write(scrollupmid_command,COM_LEN);
        }
      else if (joystick_status < JOYSTICK_MIDRANGE - JOYSTICK_LOWERDELTA && joystick_status > JOYSTICK_MIDRANGE - JOYSTICK_UPPERDELTA) {
        new_signal = NOTHING_SIG;
        Serial.write(scrolldownmid_command,COM_LEN);
        }
      else if (joystick_status <= JOYSTICK_DOWNTHRESH) {
        new_signal = NOTHING_SIG;
        Serial.write(scrolldown_command,COM_LEN);
         }
      if (current_signal == NOTHING_SIG)
        break;
      else if (current_signal == SEAM1_SIG && !wip) {
        Serial.write(go_seam1_command, COM_LEN);
        digitalWrite(SEAM1_LED, HIGH);
        digitalWrite(SEAM2_LED, LOW);
        digitalWrite(SEAM3_LED, LOW);
        }
      else if (current_signal == SEAM2_SIG && !wip) {
        Serial.write(go_seam2_command, COM_LEN);
        digitalWrite(SEAM2_LED, HIGH); 
        digitalWrite(SEAM3_LED, LOW);
        digitalWrite(SEAM1_LED, LOW);
        }
      else if (current_signal == SEAM3_SIG && !wip) {
        Serial.write(go_seam3_command, COM_LEN);
        digitalWrite(SEAM3_LED, HIGH); 
        digitalWrite(SEAM2_LED, LOW);
        digitalWrite(SEAM1_LED, LOW);
        }
      break;
    
    case _IDLE:
      //only allow jogging of the motor, system reference signal, and pressing the start button
      digitalWrite(STATUS_LED,LOW);
      digitalWrite(SEAM1_LED, LOW);
      digitalWrite(SEAM2_LED, LOW);
      digitalWrite(SEAM3_LED, LOW);
      digitalWrite(ERROR_LED,LOW);
      if (joystick_status >= JOYSTICK_UPTHRESH) {
        new_signal = NOTHING_SIG;
        Serial.write(scrollup_command,COM_LEN);
        }
      else if (joystick_status < JOYSTICK_MIDRANGE + JOYSTICK_UPPERDELTA && joystick_status > JOYSTICK_MIDRANGE + JOYSTICK_LOWERDELTA) {
        new_signal = NOTHING_SIG;
        Serial.write(scrollupmid_command,COM_LEN);
        }
      else if (joystick_status < JOYSTICK_MIDRANGE - JOYSTICK_LOWERDELTA && joystick_status > JOYSTICK_MIDRANGE - JOYSTICK_UPPERDELTA) {
        new_signal = NOTHING_SIG;
        Serial.write(scrolldownmid_command,COM_LEN);
        }
      else if (joystick_status <= JOYSTICK_DOWNTHRESH) {
        new_signal = NOTHING_SIG;
        Serial.write(scrolldown_command,COM_LEN);
        }
      if (current_signal == NOTHING_SIG) {//do nothing
        Serial.write(stop_command,COM_LEN);
        break;
        }
      else if (current_signal == START_SIG)
        current_state = _INITIALIZED;
      break;
      
    case _SYSREF:
      if (current_signal == SYSREF_SIG) {
        Serial.write(stop_command,COM_LEN);
        Serial.flush();
        delay(15); //experimentally determined time needed for SAP commands
        Serial.write(set_pos_zero_command,COM_LEN);
        Serial.flush();
        delay(15); 
        Serial.write(set_encoder_zero_command,COM_LEN);
        Serial.flush();
        delay(15);
        Serial.write(set_target_zero_command,COM_LEN);
        Serial.flush();
        current_state = _IDLE; 
        }
      else {
        Serial.write(ROL_command,COM_LEN);
        Serial.flush();
        }
      break;
      
    case _ERROR:
      //only allow jogging down of the joystick and the pressing of stop button
      if (joystick_status <= JOYSTICK_DOWNTHRESH) {
        Serial.write(scrolldown_command,COM_LEN);
        }
      else {
        Serial.write(stop_command, COM_LEN);
        Serial.flush();
        digitalWrite(ERROR_LED,HIGH); 
        }
      digitalWrite(STATUS_LED, LOW);
      digitalWrite(SEAM1_LED, LOW);
      digitalWrite(SEAM2_LED, LOW);
      digitalWrite(SEAM3_LED, LOW);
      break;
      
    default:
      current_state = _ERROR;
    }
}
//ISRs
void stop_motor() {
  store_actualpos_in_EEPROM_flag = true;
  current_state = _IDLE;
  new_signal = NOTHING_SIG;
  }
void set_seam1() {
  set_seams_flag = true;
}
void go_seam1() {
  new_signal = SEAM1_SIG;
  }
void go_seam2() {
  new_signal = SEAM2_SIG;
  }
void go_seam3() {
  new_signal = SEAM3_SIG;
  }
void start_fsm() {
  new_signal = START_SIG;
  }
void over_travel() {
  current_state = _ERROR;
  }
void set_sys_ref(){
  current_state = _SYSREF;
  new_signal = SYSREF_SIG;
}

//other functions
bool check_wip() {
  Serial.write(is_WIP_command, COM_LEN);
  if (Serial.readBytes(reply, COM_LEN)) {
    if (reply[7] == 1)
      return false;
    else
      return true; 
    }
  else 
    return true; 
  }

byte calculate_checksum(byte command[]) {
  byte i, checksum;
  checksum = command[0];
  for (i = 1; i<COM_LEN-1; i++) {
    checksum += command[i];
    }
   return checksum;
  }

void set_seam_command(byte command[], long location) {
  for (int i = 0; i < 4; i++) {
    command[i+4] = ( location >> (24 - 8*i)) & 0xFF ;
    }
  command[8] = calculate_checksum(command);
  Serial.write(command,COM_LEN);
  Serial.flush();
  Serial.readBytes(reply_test,COM_LEN);
  if (reply_test[2] == 100) {
    return;
    }
  }

void set_other_seams(void) {
   Serial.write(get_coordinate_seam1, COM_LEN);
   Serial.flush();
   Serial.readBytes(reply_test, COM_LEN);
   seam1_location = (long) reply_test[7] | ((long) reply_test[6] << 8) | ((long) reply_test[5] << 16) | ((long)reply_test[4] << 24);
   seam2_location = seam1_location + (long) STEPS_TO_NEXT_SEAM;
   seam3_location = seam2_location + (long) STEPS_TO_NEXT_SEAM;
   set_seam_command(set_seam2_command, seam2_location);
   set_seam_command(set_seam3_command, seam3_location);
  }
