/*
 * Author: Atoosa Karimi
 * Date: September 17, 2019
 * Description: Reads the analog voltage output from the Convectron KJLC
 * 275 vacuum guage sensor. Outputs the mapped voltage and pressure to 
 * the serial monitor.
 */

// libraries
#include <math.h>

// constants
#define BAUD_RATE 9600
#define INITIAL_DELAY 100
#define CONVECTRON_IN A1
#define LED 13
#define ADC_CONVERSION 0.0049
#define SAMPLE_TIME 5000 // in ms

// variables
double convectron_pressure;
double voltage_read;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  pinMode(CONVECTRON_IN, INPUT);
  pinMode(LED, OUTPUT);
  delay(INITIAL_DELAY);
  Serial.println("A1 Voltage [V], Convectron Pressure [Torr]");
}

void loop() {
  voltage_read = analogRead(CONVECTRON_IN) * ADC_CONVERSION;

  // following datasheet mapping of P = 10^(V-5)
  convectron_pressure = pow(10, voltage_read - 5);

  Serial.print(voltage_read);
  Serial.print(",");
  Serial.println(convectron_pressure,5);
  
  delay(SAMPLE_TIME);
}
