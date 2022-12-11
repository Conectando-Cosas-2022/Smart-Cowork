#include <Servo.h>

#define SERVO_PUERTA 13  // ESP32 pin GIOP26 connected to servo motor

Servo servo_puerta;  // derecha mirando desde afuera --> pos =0 esta cerrado

void setup() {
  servo_puerta.attach(SERVO_PUERTA);  // attaches the servo on ESP32 pin
  servo_puerta.write(0);
  delay(500);
}

void loop(){
  abrirPuerta();
  delay(100);
  cerrarPuerta();
}


void abrirPuerta() {
  for (int pos = 0; pos <= 120; pos += 1) {
    servo_puerta.write(pos);
    delay(15);  // waits 15ms to reach the position
  }
}

void cerrarPuerta() {
  for (int pos = 120; pos >= 0; pos -= 1) {
    // in steps of 1 degree
    servo_puerta.write(pos);
    delay(15);  // waits 15ms to reach the position
  }
}