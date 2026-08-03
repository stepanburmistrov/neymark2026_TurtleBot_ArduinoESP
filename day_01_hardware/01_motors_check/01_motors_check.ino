// Проверка моторов.
// Если направление неправильное, меняем провода мотора, а не код.

const int LEFT_DIR = 4;
const int LEFT_PWM = 5;
const int RIGHT_PWM = 6;
const int RIGHT_DIR = 7;

const int SPEED = 120;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  delay(1000);

  Serial.println("Левое колесо вперёд");
  digitalWrite(LEFT_DIR, LOW);
  analogWrite(LEFT_PWM, SPEED);
  delay(1500);
  analogWrite(LEFT_PWM, 0);
  delay(500);

  Serial.println("Левое колесо назад");
  digitalWrite(LEFT_DIR, HIGH);
  analogWrite(LEFT_PWM, SPEED);
  delay(1500);
  analogWrite(LEFT_PWM, 0);
  delay(500);

  Serial.println("Правое колесо вперёд");
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(RIGHT_PWM, SPEED);
  delay(1500);
  analogWrite(RIGHT_PWM, 0);
  delay(500);

  Serial.println("Правое колесо назад");
  digitalWrite(RIGHT_DIR, HIGH);
  analogWrite(RIGHT_PWM, SPEED);
  delay(1500);
  analogWrite(RIGHT_PWM, 0);
  delay(500);

  Serial.println("Оба колеса вперёд");
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(LEFT_PWM, SPEED);
  analogWrite(RIGHT_PWM, SPEED);
  delay(1500);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  delay(500);

  Serial.println("Оба колеса назад");
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(RIGHT_DIR, HIGH);
  analogWrite(LEFT_PWM, SPEED);
  analogWrite(RIGHT_PWM, SPEED);
  delay(1500);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  Serial.println("Тест завершён");
}

void loop() {
}
