// Кнопка подключена между D11 и GND.

const int BUTTON = 11;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(BUTTON) == LOW) {
    Serial.println("Кнопка нажата");

    while (digitalRead(BUTTON) == LOW) {
    }

    delay(50);
  }
}
