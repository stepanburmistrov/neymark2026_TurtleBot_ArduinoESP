// День 2. Первый текстовый протокол через обычный USB Serial.
// Команды:
// PING
// SERVO 45
// GET

#include <Servo.h>
#include <stdlib.h>
#include <string.h>

const uint8_t SERVO_PIN = 9;
const uint8_t LEFT_LINE_PIN = A0;
const uint8_t RIGHT_LINE_PIN = A1;
const uint8_t IR_DISTANCE_PIN = A2;
const float IR_K = 3782.0f;
const float IR_C = 96.0f;
const uint8_t BUFFER_SIZE = 48;

Servo scannerServo;
int servoAngle = 90;

int readIrDistanceCm() {
  const int raw = analogRead(IR_DISTANCE_PIN);
  if (raw >= 700) return 5;
  if (raw < 160) return 60;
  return constrain(
      (int)(IR_K / (raw - IR_C) + 0.5f), 5, 60);
}

int servoPhysicalAngle(int logicalAngle) {
  return 180 - logicalAngle;
}
char inputBuffer[BUFFER_SIZE];
uint8_t inputLength = 0;

void sendSensors() {
  Serial.print(F("L:"));
  Serial.print(analogRead(LEFT_LINE_PIN));
  Serial.print(F(" R:"));
  Serial.print(analogRead(RIGHT_LINE_PIN));
  Serial.print(F(" IR_cm:"));
  Serial.print(readIrDistanceCm());
  Serial.print(F(" Servo:"));
  Serial.println(servoAngle);
}

void processCommand(char *line) {
  char *save = NULL;
  char *command = strtok_r(line, " ", &save);
  if (command == NULL) return;

  if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
  } else if (strcmp(command, "GET") == 0) {
    sendSensors();
  } else if (strcmp(command, "SERVO") == 0) {
    char *value = strtok_r(NULL, " ", &save);
    if (value == NULL) {
      Serial.println(F("ERR SERVO_NEEDS_ANGLE"));
      return;
    }
    servoAngle = constrain(atoi(value), 20, 160);
    scannerServo.write(servoPhysicalAngle(servoAngle));
    Serial.print(F("OK SERVO "));
    Serial.println(servoAngle);
  } else {
    Serial.println(F("ERR UNKNOWN_COMMAND"));
  }
}

void readCommands() {
  while (Serial.available()) {
    const char symbol = Serial.read();
    if (symbol == '\n' || symbol == '\r') {
      if (inputLength > 0) {
        inputBuffer[inputLength] = '\0';
        processCommand(inputBuffer);
        inputLength = 0;
      }
    } else if (inputLength < BUFFER_SIZE - 1) {
      inputBuffer[inputLength++] = symbol;
    } else {
      inputLength = 0;
      Serial.println(F("ERR LINE_TOO_LONG"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  scannerServo.attach(SERVO_PIN);
  scannerServo.write(servoPhysicalAngle(servoAngle));
  Serial.println(F("READY USB_PROTOCOL"));
}

void loop() {
  readCommands();
}
