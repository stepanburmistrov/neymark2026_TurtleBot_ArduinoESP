// Проверка энкодеров.
// При вращении колеса вперёд его счётчик должен увеличиваться.
// Если счётчик уменьшается, меняем местами провода A и B.

const int LEFT_ENCODER_A = 2;
const int LEFT_ENCODER_B = 8;
const int RIGHT_ENCODER_A = 3;
const int RIGHT_ENCODER_B = 10;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

void leftEncoder() {
  if (digitalRead(LEFT_ENCODER_B) == HIGH) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

void rightEncoder() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A),
                  leftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A),
                  rightEncoder, RISING);
}

void loop() {
  noInterrupts();
  long left = leftTicks;
  long right = rightTicks;
  interrupts();

  Serial.print("LEFT = ");
  Serial.print(left);
  Serial.print("   RIGHT = ");
  Serial.println(right);

  delay(100);
}
