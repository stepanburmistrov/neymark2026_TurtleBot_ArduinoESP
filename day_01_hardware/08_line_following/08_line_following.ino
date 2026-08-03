// Движение по чёрной линии.
// Впишите свои пороги из программы 03_line_sensors_check.

const int LEFT_DIR = 4;
const int LEFT_PWM = 5;
const int RIGHT_PWM = 6;
const int RIGHT_DIR = 7;

const int LEFT_LINE = A0;
const int RIGHT_LINE = A1;
const int BUTTON = 11;

const int LEFT_THRESHOLD = 300;
const int RIGHT_THRESHOLD = 300;

const int NORMAL_SPEED = 200;
const int FAST_SPEED = 250;
const int SLOW_SPEED = 40;

void setup() {
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  // Провода уже проверены: LOW всегда означает движение вперёд.
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  while (digitalRead(BUTTON) == HIGH) {
  }
  while (digitalRead(BUTTON) == LOW) {
  }
  delay(500);
}

void loop() {
  int left = analogRead(LEFT_LINE);
  int right = analogRead(RIGHT_LINE);

  if (left >= LEFT_THRESHOLD && right >= RIGHT_THRESHOLD) {
    // Оба датчика на белом — едем прямо.
    analogWrite(LEFT_PWM, NORMAL_SPEED);
    analogWrite(RIGHT_PWM, NORMAL_SPEED);
  } else if (left < LEFT_THRESHOLD && right >= RIGHT_THRESHOLD) {
    // Левый датчик увидел линию — поворачиваем влево.
    analogWrite(LEFT_PWM, SLOW_SPEED);
    analogWrite(RIGHT_PWM, FAST_SPEED);
  } else if (left >= LEFT_THRESHOLD && right < RIGHT_THRESHOLD) {
    // Правый датчик увидел линию — поворачиваем вправо.
    analogWrite(LEFT_PWM, FAST_SPEED);
    analogWrite(RIGHT_PWM, SLOW_SPEED);
  } else {
    // Оба датчика на чёрном — финиш.
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
  }

  delay(10);
}
