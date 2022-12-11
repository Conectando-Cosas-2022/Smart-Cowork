#include <Servo.h>

#define SERVO_PIN1 26  // ESP32 pin GIOP26 connected to servo motor
#define SERVO_PIN2 27  // ESP32 pin GIOP26 connected to servo motor


Servo servo1;  // derecha mirando desde afuera --> pos =0 esta cerrado
Servo servo2;  // izquierda mirando desde afuera --> pos = 55 esta cerrado
int pos_servo1 = 0;
int pos_servo2 = 0;


void setup() {
  servo1.attach(SERVO_PIN1);  // attaches the servo on ESP32 pin
  servo2.attach(SERVO_PIN2);  // attaches the servo on ESP32 pin

  servo1.write(0);
  servo2.write(55);
  delay(500);
}

void loop() {
  abrirVentana2();
  cerrarVentana2();
}


void abrirVentanas() {
  abrirVentana1();
  abrirVentana2();
}


void abrirVentana1() {
  for (int pos = 0; pos <= 55; pos += 1) {
    servo1.write(pos);
    delay(20);  // waits 15ms to reach the position
  }
}

void abrirVentana2() {
  for (int pos = 55; pos >= 0; pos -= 1) {
    servo2.write(pos);
    delay(20);  // waits 15ms to reach the position
  }
}

void cerrarVentana1() {
  for (int pos = 55; pos >= 0; pos -= 1) {
    // in steps of 1 degree
    servo1.write(pos);
    delay(15);  // waits 15ms to reach the position
  }
}
void cerrarVentana2() {
  for (int pos = 0; pos <= 55; pos += 1) {
    servo2.write(pos);
    delay(20);  // waits 15ms to reach the position
  }
}