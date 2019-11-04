/*
 * Author: Atoosa Karimi (akarimi@etalim.com)
 * Owner Keith Antonelli (kantonelli@etalim.com)
 * Date: April 8, 2019
 * Description: Internal Bore Hole Welding Jig Automation Source Code
 * 
 */

/* TMCL Commands, Instruction #,      Description     ,  Type  
 * ROR          ,     (#1)     ,   rotate right       ,   0    
 * ROL          ,     (#2)     ,   rotate left        ,   0    
 * MST          ,     (#3)     ,   stop motor movement,   0    
 * MVP          ,     (#4)     ,   move to position   ,  0/1/2 
 * SAP          ,     (#5)     ,   set axis parameter ,  0-218 
 * GAP          ,     (#6)     ,   get axis parameter ,  0-218 
 * 
 *  Little endian 9 byte representation of data
 *  Byte,    Description
 *  8   ,  target address 
 *  7   ,  instruction number
 *  6   ,  type
 *  5   ,  motor/bank (is usu a don't care (x) byte)
 *  4   ,  value
 *  3   ,  value
 *  2   ,  value
 *  1   ,  value
 *  0   ,  checksum 
 *  
 */
//constants
#define COM_LEN 9 //both commands and replies to and from TMCM-1140 are 9 bytes

//function prototypes
char calculate_checksum(char command[]);
void crack_byte(byte b,int variable[8]);
//pin definitions
int led = 13;
int EN = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(EN, OUTPUT);
}

int i = 0;
void loop() {
  
  //Put shield in AUTOMATIC TRANSMISSION MODE using the surface mounted switch
  //0x01 01 00 00 00 00 03 E8 ED = ROR 0, 1000, rotate motor 0 right with 1000 velocity
  //0x01 02 00 00 00 00 03 E8 EE = ROL 0, 1000, rotate motor 0 left with 1000 velocity
  //0x01 03 00 00 00 00 00 00 04 = MST 0, stop motor 0
  //0x01 04 01 00 FF FF D8 F0 CC = MVP REL,0, -1000, move motor 0 from current position 10000 microsteps backwards
  //0x01 06 01 00 00 00 00 00 08 = GAP 1, 0, get the actual position of motor
  //0x01 06 08 00 00 00 00 00 0F = GAP 8, 0, position reached flag is always true if target position and actual position are true
  //0x01 1E 01 00 00 00 03 E8 0B = SCO 1, 0, 1000, set coordinate 1 of motor 0 to 1000
  //0x01 20 01 00 00 00 00 00 22 = CCO 3, 0, store the current position of motor 0 into coordinate array entry 3
  //0x01 04 02 00 00 00 00 08 0F = MVP COORD, 0, 8, move motor 0 to stored coordinate 8
  //0x01 1F 01 00 00 00 00 00 21 = GCO 1, 0, get coordinate 1 of motor 0
  //0x01 05 01 00 00 00 00 00 07 = SAP 1, 0, set the actual position to 0
  //0x01 05 D1 00 00 00 00 00 D7 = SAP 209, 0, set the encoder position to 0
  //0x01 8A 01 00 00 00 00 01 8D = get target position reached message for each MVP command that follows
      //immediate reply with status 100, and then another reply later with status 128 when reached target
  byte stop_command[]= {0x01,0x03,0x0,0x0,0x0,0x0,0x0,0x0,0x04};
  byte ROR_command[] = {0x01, 0x01, 0x0, 0x0, 0x0, 0x0, 0x03, 0xE8, 0xED};
  byte ROL_command[] = {0x01, 0x02, 0x0, 0x0, 0x0, 0x0, 0x03, 0xE8, 0xEE};
  byte actualpos_command[] = {0x01, 0x06, 0x01, 0x0, 0x0, 0x0, 0x0,0x0, 0x08};
  byte set_user_ref_command[] = {0x01, 0x20, 0x02, 0x0, 0x0, 0x0, 0x0, 0x0, 0x23}; //stores current position into coordinate number 2 - associated with user reference
  byte is_WIP_command[] = {0x01, 0x06, 0x08, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF};
  byte set_pos_zero_command[] = {0x01, 0x05, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x07};
  byte set_encoder_zero_command[]={0x01, 0x05, 0xD1, 0x0, 0x0, 0x0, 0x0, 0x0, 0xD7};
  byte reply[9];
  int variable[8];

  digitalWrite(led, LOW);
  //Serial.write(stop_command, COM_LEN);
  Serial.write(is_WIP_command, COM_LEN);
  if (Serial.readBytes(reply,COM_LEN)) { //reply[2] corresponds to status, if reply[2] == 100 then status OK
      if (reply[7] == 1)
        digitalWrite(led, HIGH);
      }
      
  int actualpos = reply[4] + (reply[5] << 8) + (reply[6] << 16) + (reply[7] << 24);
  /*crack_byte(reply[2], variable);
  for (i = 0; i < 8; i++) {
    digitalWrite(led, !variable[i]);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);
    }
   }
  else
    digitalWrite(led, HIGH);*/
  delay(10000);
}

char calculate_checksum(char command[]) {
  if (sizeof(command) != COM_LEN) {
      return (byte) 0;
    }
  char i, checksum;
  checksum = command[0];
  for (i = 1; i<COM_LEN-1; i++) {
    checksum += command[i];
    }
   return checksum;
  }

void crack_byte( byte b, int variable[8] )
{
 byte i;
 
 for ( i=0; i < 8; ++i )
 {
   variable[i] = b & 1;
   b = b >> 1;
 }
}
 
