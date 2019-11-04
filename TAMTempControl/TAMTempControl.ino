/*
 * Author: Atoosa Karimi
 * Date: October 30, 2019
 * Description: PID control of individual TAM temperatures
 */

// pin definitions
#define TAM1_TEMP_PIN A0
#define TAM2_TEMP_PIN A1
#define TAM3_TEMP_PIN A2
#define TAM4_TEMP_PIN A3
#define POWER_FEEDFWD_PIN A4
#define TAM1_RELAY_PIN 6
#define TAM2_RELAY_PIN 9
#define TAM3_RELAY_PIN 10
#define TAM4_RELAY_PIN 11

// constants
#define ANALOG_VOLT 0.0049 // to convert ADC read to volts
#define VOLT_TEMP 200 // 0V = 0C and 5V=1000C
#define FLOAT_PWM 255 // duty cycle 0 always OFF and 255 always ON
#define FEEDFWD_TEMP_POWER 80.0/800 // max temperature of 800C and individual heater power rating of 125W
#define KP 2 // PID proportional
#define KD 1 // PID derivative
#define KI 2 // PID integral
#define DIVI 150 // PID integral filter time. Cycle time appears to be about 150 sec
#define DIVT 10 // temperature averaging
#define MAX_VOLT 230.0 // the maximum AC voltage
#define HEATER_RESISTANCE 60.0 // individual heater resistance in ohms
#define FULL_POWER MAX_VOLT*MAX_VOLT/HEATER_RESISTANCE // in watts
#define RAMP_RATE (800-200)/3600.0 // slowly increases setpoint temp for lower thermal stress
#define FEEDFWDREFTEMP 800

// function prototypes
float calculate_duty(float pid);
float calculate_feedfwd(float temperature);

// global variables
int setpoint_temp = 0; // in degrees C
float ramping_setpoint_temp; // in degrees C set to ambient temperaturelong int user_input = 800; // in degrees C
float tam1_temp = 0.0;
float tam2_temp = 0.0;
float tam3_temp = 0.0;
float tam4_temp = 0.0;
float tam1_duty = 0.0;
float tam2_duty = 0.0;
float tam3_duty = 0.0;
float tam4_duty = 0.0;
float tam1_error = setpoint_temp;
float tam2_error = setpoint_temp;
float tam3_error = setpoint_temp;
float tam4_error = setpoint_temp;
float tam1_prev_error = 0.0;
float tam2_prev_error = 0.0;
float tam3_prev_error = 0.0;
float tam4_prev_error = 0.0;
float tam1_integral = 0.0;
float tam2_integral = 0.0;
float tam3_integral = 0.0;
float tam4_integral = 0.0;
float tam1_derivative = 0.0;
float tam2_derivative = 0.0;
float tam3_derivative = 0.0;
float tam4_derivative = 0.0;
float power = 0.0;
int user_input = 0;
bool ramping_flag = false;

void setup() {
  Serial.begin(9600);

  pinMode(TAM1_TEMP_PIN, INPUT);
  pinMode(TAM2_TEMP_PIN, INPUT);
  pinMode(TAM3_TEMP_PIN, INPUT);
  pinMode(TAM4_TEMP_PIN, INPUT);
  pinMode(TAM1_RELAY_PIN, OUTPUT);
  pinMode(TAM2_RELAY_PIN, OUTPUT);
  pinMode(TAM3_RELAY_PIN, OUTPUT);
  pinMode(TAM4_RELAY_PIN, OUTPUT);

  Serial.print("Full power calculated as: ");
  Serial.println(FULL_POWER);
  Serial.print("Setpoint temperature set as: ");
  Serial.println(setpoint_temp);
  Serial.println("TAM 1 temp, TAM 2 temp, TAM 3 temp, TAM 4 temp, TAM 1 pwm, TAM 2 pwm, TAM 3 pwm, TAM 4 pwm, Ramping setpoint");

  float voltage_read = analogRead(TAM1_TEMP_PIN) * ANALOG_VOLT;
  tam1_temp = voltage_read * VOLT_TEMP;

  voltage_read = analogRead(TAM2_TEMP_PIN) * ANALOG_VOLT;
  tam2_temp = voltage_read * VOLT_TEMP;

  voltage_read = analogRead(TAM3_TEMP_PIN) * ANALOG_VOLT;
  tam3_temp = voltage_read * VOLT_TEMP;

  voltage_read = analogRead(TAM4_TEMP_PIN) * ANALOG_VOLT;
  tam4_temp = voltage_read * VOLT_TEMP;
}

void loop() {
  // check to see if user has changed setpoint
  if (Serial.available()) {
    user_input = Serial.parseInt();
    if (user_input > 0 && user_input <= 800)
      setpoint_temp = user_input;
    Serial.print("New setpoint temperature: ");
    Serial.println(setpoint_temp);
    ramping_flag = true;

    float voltage_read = analogRead(TAM1_TEMP_PIN) * ANALOG_VOLT;
    tam1_temp = voltage_read * VOLT_TEMP;

    voltage_read = analogRead(TAM2_TEMP_PIN) * ANALOG_VOLT;
    tam2_temp = voltage_read * VOLT_TEMP;

    voltage_read = analogRead(TAM3_TEMP_PIN) * ANALOG_VOLT;
    tam3_temp = voltage_read * VOLT_TEMP;

    voltage_read = analogRead(TAM4_TEMP_PIN) * ANALOG_VOLT;
    tam4_temp = voltage_read * VOLT_TEMP;

    ramping_setpoint_temp = (tam1_temp + tam2_temp + tam3_temp + tam4_temp) / 4; // in degrees C set to measured temperature
    }

  // ramp internal setpoint temperature
  if (ramping_setpoint_temp < setpoint_temp && ramping_flag) {
    ramping_setpoint_temp += RAMP_RATE;
    tam1_integral = tam2_integral = tam3_integral = tam4_integral = 0;  // prevent integral windup during temperature ramp
    }
  else {
    ramping_setpoint_temp = setpoint_temp;
    ramping_flag = false;
    }


  // read the temperatures
  float voltage_read = analogRead(TAM1_TEMP_PIN) * ANALOG_VOLT;
  tam1_temp = ((DIVT - 1) * tam1_temp + voltage_read * VOLT_TEMP) / DIVT;

  voltage_read = analogRead(TAM2_TEMP_PIN) * ANALOG_VOLT;
  tam2_temp = ((DIVT - 1) * tam2_temp + voltage_read * VOLT_TEMP) / DIVT;

  voltage_read = analogRead(TAM3_TEMP_PIN) * ANALOG_VOLT;
  tam3_temp = ((DIVT - 1) * tam3_temp + voltage_read * VOLT_TEMP) / DIVT;

  voltage_read = analogRead(TAM4_TEMP_PIN) * ANALOG_VOLT;
  tam4_temp = ((DIVT - 1) * tam4_temp + voltage_read * VOLT_TEMP) / DIVT;

  // read the feedforward power
  voltage_read = analogRead(POWER_FEEDFWD_PIN) * ANALOG_VOLT;
  // todo: map voltage to power
  
  // for testing
  /*tam1_temp = 25.0;
  tam2_temp = 300.0;
  tam3_temp = 110;
  tam4_temp = 800;*/

  // calculate PID parameters
  tam1_error = ramping_setpoint_temp - tam1_temp;
  tam1_integral = ((DIVI - 1) * tam1_integral + tam1_error)/DIVI;
  tam1_derivative = tam1_error - tam1_prev_error;
  float pid = (KP * tam1_error) + (KD * tam1_derivative) + (KI * tam1_integral) + calculate_feedfwd_temp(tam1_temp) + calculate_feedfwd_power(power); 
  tam1_duty = calculate_duty(pid);
  tam1_prev_error = tam1_error;

  tam2_error = ramping_setpoint_temp - tam2_temp;
  tam2_integral = ((DIVI - 1) * tam2_integral + tam2_error)/DIVI;
  tam2_derivative = tam2_error - tam2_prev_error;
  pid = (KP * tam2_error) + (KD * tam2_derivative) + (KI * tam2_integral) + calculate_feedfwd_temp(tam2_temp) + calculate_feedfwd_power(power);
  tam2_duty = calculate_duty(pid);
  tam2_prev_error = tam2_error;

  tam3_error = ramping_setpoint_temp - tam3_temp;
  tam3_integral = ((DIVI - 1) * tam3_integral + tam3_error)/DIVI;
  tam3_derivative = tam3_error - tam3_prev_error;
  pid = (KP * tam3_error) + (KD * tam3_derivative) + (KI * tam3_integral) + calculate_feedfwd_temp(tam3_temp) + calculate_feedfwd_power(power);
  tam3_duty = calculate_duty(pid);
  tam3_prev_error = tam3_error;

  tam4_error = ramping_setpoint_temp - tam4_temp;
  tam4_integral  = ((DIVI - 1) * tam4_integral + tam4_error)/DIVI;
  tam4_derivative = tam4_error - tam4_prev_error;
  pid = (KP * tam4_error) + (KD * tam4_derivative) + (KI * tam4_integral)  + calculate_feedfwd_temp(tam4_temp) + calculate_feedfwd_power(power);
  tam4_duty = calculate_duty(pid);
  tam4_prev_error = tam4_error;

  // PWM relays
  analogWrite(TAM1_RELAY_PIN, (int) (tam1_duty * FLOAT_PWM));
  analogWrite(TAM2_RELAY_PIN, (int) (tam2_duty * FLOAT_PWM));
  analogWrite(TAM3_RELAY_PIN, (int) (tam3_duty * FLOAT_PWM));
  analogWrite(TAM4_RELAY_PIN, (int) (tam4_duty * FLOAT_PWM));

  // prints
  Serial.print(tam1_temp);
  Serial.print(",");
  Serial.print(tam2_temp);
  Serial.print(",");
  Serial.print(tam3_temp);
  Serial.print(",");
  Serial.print(tam4_temp);
  Serial.print(",");
  Serial.print(tam1_duty);
  Serial.print(",");
  Serial.print(tam2_duty);
  Serial.print(",");
  Serial.print(tam3_duty);
  Serial.print(",");
  Serial.print(tam4_duty);
  Serial.print(",");
  Serial.println(ramping_setpoint_temp);

  delay(1000);
}

// calculates what the duty cycle should be based on PID parameter
float calculate_duty(float pid) {
  float duty = pid/FULL_POWER;
  if (duty < 0 )
    return 0;
  else if (duty > 1)
    return 1;
  else return duty;
}

// calculates the feedforward watts based on max temperature and heater power rating
float calculate_feedfwd_temp(float temperature) {
  return FEEDFWD_TEMP_POWER*temperature / FEEDFWDREFTEMP;
  }

// calculates the feedforward watts based on power
float calculate_feedfwd_power(float power) {
  return 0;
  }
