int sensorValue;

void setup() {
  Serial.begin(9600);  // sets the serial port to 9600
  pinMode(32, INPUT);
}
void loop() {
  sensorValue = analogRead(32);  // read analog input pin 0
  Serial.print("AirQuality=");
  Serial.print(sensorValue, DEC);  // prints the value read
  Serial.println(" PPM");
  delay(100);  // wait 100ms for next reading
}
