// Поиск дверного проёма по трём измерениям:
// справа, прямо и слева.

#include <Servo.h>

const int LEFT_DIR = 4;
const int LEFT_PWM = 5;
const int RIGHT_PWM = 6;
const int RIGHT_DIR = 7;
const int SERVO_PIN = 9;
const int BUTTON = 11;
const int IR_SENSOR = A2;

const int SPEED = 120;
const float K = 3782.0;
const float C = 96.0;

Servo scanner;

float readDistance() {
  int raw = analogRead(IR_SENSOR);

  if (raw >= 700) {
    return 5.0;
  }

  if (raw < 160) {
    return 60.0;
  }

  return K / (raw - C);
}

void stopMotors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

void forward() {
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(LEFT_PWM, SPEED);
  analogWrite(RIGHT_PWM, SPEED);
}

void turnLeft() {
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(LEFT_PWM, SPEED);
  analogWrite(RIGHT_PWM, SPEED);
}

void turnRight() {
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, HIGH);
  analogWrite(LEFT_PWM, SPEED);
  analogWrite(RIGHT_PWM, SPEED);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  scanner.attach(SERVO_PIN);
  scanner.write(90);
  stopMotors();

  while (digitalRead(BUTTON) == HIGH) {
  }
  while (digitalRead(BUTTON) == LOW) {
  }

  scanner.write(30);
  delay(700);
  float rightDistance = readDistance();

  scanner.write(90);
  delay(700);
  float centerDistance = readDistance();

  scanner.write(150);
  delay(700);
  float leftDistance = readDistance();

  scanner.write(90);
  delay(500);

  Serial.print("LEFT = ");
  Serial.print(leftDistance);
  Serial.print("   CENTER = ");
  Serial.print(centerDistance);
  Serial.print("   RIGHT = ");
  Serial.println(rightDistance);

  if (leftDistance > centerDistance && leftDistance > rightDistance) {
    turnLeft();
    delay(700);
  } else if (rightDistance > centerDistance) {
    turnRight();
    delay(700);
  }

  stopMotors();
  delay(300);

  forward();
  delay(2500);
  stopMotors();
}

void loop() {
}
