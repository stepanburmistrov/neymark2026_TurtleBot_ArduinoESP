// День 2. Перевод тиков энкодеров в путь каждого колеса.

const uint8_t LEFT_ENCODER_A_PIN = 2;
const uint8_t LEFT_ENCODER_B_PIN = 8;
const uint8_t RIGHT_ENCODER_A_PIN = 3;
const uint8_t RIGHT_ENCODER_B_PIN = 10;

const float WHEEL_DIAMETER_MM = 44.0f;
const float ENCODER_TICKS_PER_REV = 358.0f;
const float MM_PER_TICK =
    PI * WHEEL_DIAMETER_MM / ENCODER_TICKS_PER_REV;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

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
  static unsigned long previousPrintMs = 0;
  if (millis() - previousPrintMs < 100) return;
  previousPrintMs = millis();

  long left;
  long right;
  copyTicks(left, right);

  Serial.print(F("L:"));
  Serial.print(left * MM_PER_TICK, 1);
  Serial.print(F(" R:"));
  Serial.println(right * MM_PER_TICK, 1);
}
