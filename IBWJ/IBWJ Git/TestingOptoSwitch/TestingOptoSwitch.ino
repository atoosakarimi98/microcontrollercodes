void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(3, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool opto_status = digitalRead(3);
  Serial.println(opto_status);
  delay(1000);
}
