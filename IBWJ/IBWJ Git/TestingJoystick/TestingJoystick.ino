
#define JOYSTICK A3
#define ADC_VOLT_CONVERSION 0.0049 //derived from the ADC and actual voltage relationship of analogRead(pin)
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(JOYSTICK, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  float voltage_read = analogRead(JOYSTICK) * ADC_VOLT_CONVERSION;
  Serial.println(voltage_read);
  delay(500);
}
