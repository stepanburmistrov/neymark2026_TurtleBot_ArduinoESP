// День 2. Полная одометрия дифференциального робота.
// Поставьте робот в (0, 0), направив его вдоль положительной оси X,
// и катайте руками по размеченному столу.

#include <math.h>

const uint8_t LEFT_ENCODER_A_PIN = 2;
const uint8_t LEFT_ENCODER_B_PIN = 8;
const uint8_t RIGHT_ENCODER_A_PIN = 3;
const uint8_t RIGHT_ENCODER_B_PIN = 10;

const float WHEEL_DIAMETER_MM = 44.0f;
const float WHEEL_BASE_MM = 128.0f;
const float ENCODER_TICKS_PER_REV = 358.0f;
const float MM_PER_TICK =
    PI * WHEEL_DIAMETER_MM / ENCODER_TICKS_PER_REV;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
long previousLeftTicks = 0;
long previousRightTicks = 0;
float xMm = 0.0f;
float yMm = 0.0f;
float thetaRad = 0.0f;

void onLeftEncoder() {
  leftTicks += digitalRead(LEFT_ENCODER_B_PIN) ? 1 : -1;
}

void onRightEncoder() {
  rightTicks += digitalRead(RIGHT_ENCODER_B_PIN) ? 1 : -1;
}

void copyTicks(long &left, long &right) {
  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();
}

float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

void updateOdometry() {
  long left;
  long right;
  copyTicks(left, right);
  const long deltaLeftTicks = left - previousLeftTicks;
  const long deltaRightTicks = right - previousRightTicks;
  previousLeftTicks = left;
  previousRightTicks = right;

  const float leftDistance = deltaLeftTicks * MM_PER_TICK;
  const float rightDistance = deltaRightTicks * MM_PER_TICK;
  const float distance = 0.5f * (leftDistance + rightDistance);
  const float deltaTheta =
      (rightDistance - leftDistance) / WHEEL_BASE_MM;
  const float middleTheta = thetaRad + 0.5f * deltaTheta;

  xMm += distance * cos(middleTheta);
  yMm += distance * sin(middleTheta);
  thetaRad = normalizeAngle(thetaRad + deltaTheta);
}

void resetOdometry() {
  copyTicks(previousLeftTicks, previousRightTicks);
  xMm = 0.0f;
  yMm = 0.0f;
  thetaRad = 0.0f;
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_ENCODER_A_PIN, INPUT);
  pinMode(LEFT_ENCODER_B_PIN, INPUT);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT);
  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENCODER_A_PIN), onLeftEncoder, RISING);
  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), onRightEncoder, RISING);
  resetOdometry();
}

void loop() {
  static unsigned long previousUpdateMs = 0;
  static unsigned long previousPrintMs = 0;
  const unsigned long nowMs = millis();

  if (nowMs - previousUpdateMs >= 50) {
    previousUpdateMs = nowMs;
    updateOdometry();
  }

  if (nowMs - previousPrintMs >= 100) {
    previousPrintMs = nowMs;
    Serial.print(F("X:"));
    Serial.print(xMm, 1);
    Serial.print(F(" Y:"));
    Serial.print(yMm, 1);
    Serial.print(F(" Theta:"));
    Serial.println(thetaRad, 4);
  }

  if (Serial.available() && Serial.read() == 'R') {
    resetOdometry();
    Serial.println(F("RESET"));
  }
}
