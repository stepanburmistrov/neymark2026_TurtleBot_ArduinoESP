const int IR_SENSOR = A2;

const float K = 3782.0;
const float C = 96.0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(IR_SENSOR);

  Serial.print("RAW = ");
  Serial.print(raw);

  if (raw >= 700) {
    // Значения на 3...6 см почти одинаковые.
    Serial.println("   DISTANCE = 5.0 cm");
  } else if (raw < 160) {
    Serial.println("   DISTANCE > 60 cm");
  } else {
    float distance = K / (raw - C);

    Serial.print("   DISTANCE = ");
    Serial.print(distance, 1);
    Serial.println(" cm");
  }

  delay(200);
}
