// Кегельринг: не выезжаем за границу и ищем кеглю.

#include <Servo.h>

const int LEFT_DIR = 4;
const int LEFT_PWM = 5;
const int RIGHT_PWM = 6;
const int RIGHT_DIR = 7;
const int SERVO_PIN = 9;
const int BUTTON = 11;
const int LEFT_LINE = A0;
const int RIGHT_LINE = A1;
const int IR_SENSOR = A2;

const int LEFT_THRESHOLD = 500;
const int RIGHT_THRESHOLD = 500;
const float OBJECT_DISTANCE = 25.0;

const int SEARCH_SPEED = 90;
const int ATTACK_SPEED = 170;
const int ESCAPE_SPEED = 140;

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
  analogWrite(LEFT_PWM, ATTACK_SPEED);
  analogWrite(RIGHT_PWM, ATTACK_SPEED);
}

void backward() {
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(RIGHT_DIR, HIGH);
  analogWrite(LEFT_PWM, ESCAPE_SPEED);
  analogWrite(RIGHT_PWM, ESCAPE_SPEED);
}

void turnLeft() {
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(LEFT_PWM, SEARCH_SPEED);
  analogWrite(RIGHT_PWM, SEARCH_SPEED);
}

void turnRight() {
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, HIGH);
  analogWrite(LEFT_PWM, SEARCH_SPEED);
  analogWrite(RIGHT_PWM, SEARCH_SPEED);
}

void setup() {
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

  delay(3000);
}

void loop() {
  int left = analogRead(LEFT_LINE);
  int right = analogRead(RIGHT_LINE);

  // На белом значение больше порога, на чёрной границе — меньше.
  if (left < LEFT_THRESHOLD || right < RIGHT_THRESHOLD) {
    backward();
    delay(400);

    if (left < LEFT_THRESHOLD) {
      turnRight();
    } else {
      turnLeft();
    }

    delay(500);
    stopMotors();
  } else if (readDistance() < OBJECT_DISTANCE) {
    forward();
  } else {
    turnLeft();
  }

  delay(10);
}
