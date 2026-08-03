// Проверка сервопривода.

#include <Servo.h>

const int SERVO_PIN = 9;

Servo scanner;

void setup() {
  scanner.attach(SERVO_PIN);

  scanner.write(90);
  delay(1000);

  scanner.write(30);
  delay(1000);

  scanner.write(90);
  delay(1000);

  scanner.write(150);
  delay(1000);

  scanner.write(90);
}

void loop() {
}
