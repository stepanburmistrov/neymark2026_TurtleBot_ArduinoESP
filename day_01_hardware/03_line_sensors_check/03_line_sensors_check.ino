// Проверка датчиков линии.
// Записываем значения над белой и чёрной поверхностью.

const int LEFT_LINE = A0;
const int RIGHT_LINE = A1;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int left = analogRead(LEFT_LINE);
  int right = analogRead(RIGHT_LINE);

  Serial.print("LEFT = ");
  Serial.print(left);
  Serial.print("   RIGHT = ");
  Serial.println(right);

  delay(200);
}
