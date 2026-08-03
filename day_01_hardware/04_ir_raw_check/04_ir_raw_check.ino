// Первое измерение ИК-дальномера.
// Выводим только аналоговое значение от 0 до 1023.

const int IR_SENSOR = A2;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int value = analogRead(IR_SENSOR);
  Serial.println(value);
  delay(200);
}
