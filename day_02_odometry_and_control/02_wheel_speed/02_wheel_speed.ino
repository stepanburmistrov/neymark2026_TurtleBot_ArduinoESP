// День 2. Скорость каждого колеса в мм/с.
// Колёса можно вращать руками или поднять робот над столом.

const uint8_t LEFT_ENCODER_A_PIN = 2;
const uint8_t LEFT_ENCODER_B_PIN = 8;
const uint8_t RIGHT_ENCODER_A_PIN = 3;
const uint8_t RIGHT_ENCODER_B_PIN = 10;

const float WHEEL_DIAMETER_MM = 44.0f;
const float ENCODER_TICKS_PER_REV = 358.0f;
const float MM_PER_TICK =
    PI * WHEEL_DIAMETER_MM / ENCODER_TICKS_PER_REV;
const unsigned long SPEED_PERIOD_MS = 100;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
long previousLeftTicks = 0;
long previousRightTicks = 0;

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
}

void loop() {
  static unsigned long previousMs = 0;
  const unsigned long nowMs = millis();
  if (nowMs - previousMs < SPEED_PERIOD_MS) return;

  const float dt = (nowMs - previousMs) * 0.001f;
  previousMs = nowMs;

  long left;
  long right;
  copyTicks(left, right);
  const long deltaLeft = left - previousLeftTicks;
  const long deltaRight = right - previousRightTicks;
  previousLeftTicks = left;
  previousRightTicks = right;

  const float leftSpeed = deltaLeft * MM_PER_TICK / dt;
  const float rightSpeed = deltaRight * MM_PER_TICK / dt;

  Serial.print(F("L:"));
  Serial.print(leftSpeed, 1);
  Serial.print(F(" R:"));
  Serial.println(rightSpeed, 1);
}
