/*
 * Author: Atoosa Karimi
 * Description: preliminary code for interfacing the Sanhua 3 way valve using LIN protocol.
 * Note: The lin_bus.cpp was modified to output the correct ID according to LIN 1.3.
 */
#include "lin_bus.h"
#include "SdFat.h"          //SD Card library
#include <TimeLib.h>        //Timekeeping library for RTC

#define ID_FIELD  0x04 // command PID
#define DIAGNOSTIC_FRAME 0x03 // diagnostic PID (never used)
#define LEN 8 // bytes of data
#define SETPOINT_TEMP 40*1000 // duct setpoint temperature C*1000
#define TEMP_HYST 2*1000 // on/off controller deadband C*1000
#define ANALOG_MAX 255
#define ANALOG_MIN 0
#define MULTIPLEXER_READ_DELAY 30  
#define TEMPERATURE_AVERAGING 0.90              
#define TEMP_SCALE 1038631                      // new mapping

// multiplexer pin definitions
#define mux_A_pin 39
#define mux_B_pin 38
#define mux_C_pin 37
#define multiplexer_first 23
#define multiplexer_second 17
#define MULTIPLEXER_LOW_READING 0                    //multiplexer thermocouple minimum reading 
#define MULTIPLEXER_HIGH_READING 1023                //multiplexer thermocouple maximum reading 

// thermocouple channels
const bool TC0[4] = {HIGH, LOW, HIGH, LOW};
const bool TC1[4] = {HIGH, LOW, LOW, LOW};
const bool TC2[4] = {LOW, HIGH, HIGH, LOW};
const bool TC3[4] = {LOW, HIGH, LOW, LOW};
const bool TC4[4] = {LOW, LOW, HIGH, LOW};
const bool TC5[4] = {LOW, LOW, LOW, LOW};
const bool TC6[4] = {HIGH, LOW, HIGH, HIGH};
const bool TC7[4] = {HIGH, LOW, LOW, HIGH};
const bool TC8[4] = {LOW, HIGH, HIGH, HIGH};
const bool TC9[4] = {LOW, HIGH, LOW, HIGH};
const bool TC10[4] = {LOW, LOW, HIGH, HIGH};
const bool TC11[4] = {LOW, LOW, LOW, HIGH};

//SD Card File
char filename[30] = "";
SdFatSdio sd;
File saveFile;

// function prototypes
float read_sensor(const bool *channel, int prev, int max_scale, float averaging);
void set_valve(byte valve);
int read_in_command();

// other constants
int lin_cs = 14; // set HIGH to enable transceiver chip
String readString;
byte valve = 0x80;
byte previous_valve = 0x0;
float temp_compression = 20.00*1000;
float tim;
byte valve_open = 0xFF;
byte valve_close = 0x00;


LIN lin(&Serial3, 19200); // this is the only baudrate that works
uint8_t tx_buffer[] = {0x01, valve, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // transmitting buffer
uint8_t rx_buffer[LEN]; // receiving buffer


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  pinMode(lin_cs, OUTPUT); 
  digitalWrite(lin_cs, HIGH); // enable transceiver chip
  delay(1000);
  
  Serial.begin(115200);  // Configure serial port for debug
  Serial.println("Time, Compression Temperature");
  
  pinMode(mux_A_pin, OUTPUT);
  pinMode(mux_B_pin, OUTPUT);
  pinMode(mux_C_pin, OUTPUT);
  
  digitalWrite(LED_BUILTIN, LOW);

  if (sd.begin()) {
  Serial.println("Card present");
  sprintf(filename, "%02d_%02d_%02d.log",year(),month(),day());
  SdFile::dateTimeCallback(date_time);
    saveFile = sd.open(filename, FILE_WRITE);
    if (saveFile){
      char DateString[] = "";
      sprintf(DateString, "Time, Expansion");
      saveFile.print(DateString);
      saveFile.println();
      saveFile.close();
    }
  }
}

void loop() {
  temp_compression = read_sensor(TC0, temp_compression, TEMP_SCALE, TEMPERATURE_AVERAGING); //for relay 1
  delay(15);
  tim = hour() * 3600 + minute() * 60 + second();

  // input from user taked precedence
  if (Serial.available()) {
  int user_input = read_in_command();
  if (user_input == -99);
  else 
    valve = user_input & 0xFF; //turn into a byte <
    set_valve(valve);
    }

  set_valve(0x00); // fully closed to see what steady-state temperature is
  // on-off control implementation
  /*if (temp_compression > SETPOINT_TEMP + TEMP_HYST) {
    set_valve(valve_open);
    delay(100);
    set_valve(valve_open);
    previous_valve = valve_close;
    }
  else if (temp_compression < SETPOINT_TEMP - TEMP_HYST) {
    set_valve(valve_close);
    delay(100);
    set_valve(valve_close);
    previous_valve = valve_close;
    }
  else
    set_valve(previous_valve);*/
    

  Serial.print("\n");
  Serial.print(millis());
  Serial.print(",");
  Serial.println(temp_compression/1000.0);
  
  //Printing to SD card
  saveFile = sd.open(filename, FILE_WRITE);
  if (saveFile){
    char timeString[11] = "0000000000";
    sprintf(timeString,"%10.1f,",tim);
    saveFile.print(timeString);
    char physVals[55] = "0000000000";
    sprintf(physVals,"%10.2f",temp_compression);
    saveFile.print(physVals);
    saveFile.println();
    saveFile.close();
  }
  delay(2000);
  
}

int read_in_command() 
{      
    String readString;
    while (Serial.available() > 0) 
    {
     char c = Serial.read();
      readString += c;
      delay(2);
      }
    if (readString.length() > 0) {
      int n = readString.toInt();
      Serial.print("\nread: ");
      Serial.print(n);
      Serial.print("   ");
      Serial.println(readString);
      if (n > 255 || n < 0 )
        return -99;
      return n;
      }
}

float read_sensor(const bool *channel, int prev, int max_scale, float averaging)
{
  uint16_t unmapped; 
  int mapped; 

 
    //read analog multiplexer
    digitalWrite(mux_C_pin, channel[0]);
    digitalWrite(mux_B_pin, channel[1]);
    digitalWrite(mux_A_pin, channel[2]);
    delay(MULTIPLEXER_READ_DELAY);
    
    if(channel[3])
      unmapped =  analogRead(multiplexer_first);
    else if (!channel[3])
      unmapped =  analogRead(multiplexer_second);
    else 
      return 0; 
      
    mapped = map(unmapped, MULTIPLEXER_LOW_READING, MULTIPLEXER_HIGH_READING, 0, max_scale); 
  

  //AVERAGE VALUE 
  return mapped*(1.0-averaging) + (prev * averaging); 
}

void set_valve(byte valve) {
  tx_buffer[1] = valve;
  lin.order(ID_FIELD, tx_buffer, LEN, 0); // 0 corresponds to LIN 2.1 enhanced checksum calculation
}
time_t get_Teensy3_time()
{
  return Teensy3Clock.get();
}

//
void date_time(uint16_t* date, uint16_t* time) 
{
  time_t t = get_Teensy3_time();
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(t), month(t),day(t));

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(t), minute(t), second(t));

}

float right_now(void) {
  return hour() * 3600 + minute() * 60 + second();
  }
