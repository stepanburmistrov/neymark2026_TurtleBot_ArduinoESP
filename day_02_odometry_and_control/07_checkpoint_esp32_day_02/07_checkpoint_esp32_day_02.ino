// Финальная прошивка ESP32-C3 дня 2.
// Роль платы намеренно ограничена транспортом:
// точка доступа + DHCP + статусная веб-страница + TCP/UART мост.
// Моторы, датчики, одометрия и watchdog всегда остаются в Arduino.

#include <WebServer.h>
#include <WiFi.h>

const char *AP_NAME = "NEYMARK_01";
const char *AP_PASSWORD = "neymark123";
const uint16_t TCP_PORT = 8888;
const uint8_t ROBOT_RX_PIN = 4;
const uint8_t ROBOT_TX_PIN = 5;

HardwareSerial robotSerial(1);
WebServer server(80);
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
char tcpLine[192];
uint8_t tcpLength = 0;
char uartLine[192];
uint8_t uartLength = 0;
unsigned long commandsFromPc = 0;
unsigned long linesFromArduino = 0;

void sendTcp(const char *text) {
  if (tcpClient && tcpClient.connected()) {
    tcpClient.println(text);
  }
}

void showStatus() {
  String page =
      "<!doctype html><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width'>"
      "<style>body{font-family:sans-serif;max-width:600px;margin:30px auto}"
      "b{color:#1677d2}</style><h1>NEYMARK Robot</h1>"
      "<p>Wi-Fi: <b>";
  page += AP_NAME;
  page += "</b></p><p>TCP: <b>192.168.4.1:";
  page += String(TCP_PORT);
  page += "</b></p><p>Клиент: <b>";
  page += (
      tcpClient && tcpClient.connected() ? "подключён" : "нет");
  page += "</b></p><p>Команд от ПК: ";
  page += String(commandsFromPc);
  page += "</p><p>Строк от Arduino: ";
  page += String(linesFromArduino);
  page += "</p>";
  server.send(200, "text/html; charset=utf-8", page);
}

void acceptTcpClient() {
  if (tcpClient && tcpClient.connected()) return;
  tcpClient = tcpServer.available();
  if (tcpClient) {
    tcpClient.setNoDelay(true);
    tcpLength = 0;
  }
}

void readTcpClient() {
  while (tcpClient && tcpClient.available()) {
    const char symbol = tcpClient.read();
    if (symbol == '\n' || symbol == '\r') {
      if (tcpLength > 0) {
        tcpLine[tcpLength] = '\0';
        robotSerial.println(tcpLine);
        commandsFromPc++;
        tcpLength = 0;
      }
    } else if (tcpLength < sizeof(tcpLine) - 1) {
      tcpLine[tcpLength++] = symbol;
    } else {
      tcpLength = 0;
      sendTcp("ERR LINE_TOO_LONG");
    }
  }
}

void readRobotSerial() {
  while (robotSerial.available()) {
    const char symbol = robotSerial.read();
    if (symbol == '\n' || symbol == '\r') {
      if (uartLength > 0) {
        uartLine[uartLength] = '\0';
        sendTcp(uartLine);
        linesFromArduino++;
        uartLength = 0;
      }
    } else if (uartLength < sizeof(uartLine) - 1) {
      uartLine[uartLength++] = symbol;
    } else {
      uartLength = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  robotSerial.begin(
      38400, SERIAL_8N1, ROBOT_RX_PIN, ROBOT_TX_PIN);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(300);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAP(AP_NAME, AP_PASSWORD);

  tcpServer.begin();
  server.on("/", HTTP_GET, showStatus);
  server.begin();

  Serial.println("READY ESP32_DAY2");
  Serial.println("Wi-Fi: NEYMARK_01 / neymark123");
  Serial.println("TCP: 192.168.4.1:8888");
}

void loop() {
  server.handleClient();
  acceptTcpClient();
  readTcpClient();
  readRobotSerial();
}
