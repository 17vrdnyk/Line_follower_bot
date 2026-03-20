#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "LFR_Master_AP";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define RXD2 3
#define TXD2 1

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    // 1. Turn LED ON (Start of receiving data)
    digitalWrite(LED_BUILTIN, HIGH);

    String command = "";
    for(size_t i=0; i<len; i++) command += (char)data[i];
    Serial2.println(command); // Send tuning to Uno
    Serial.println("To Uno: " + command);

    // 2. Simple "Blink" effect
    // We use a tiny delay so your eyes can actually see the flash
    delay(50); 
    digitalWrite(LED_BUILTIN, LOW); // Turn LED OFF
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Start with LED ON (Active HIGH)

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // --- WiFi Power Optimization ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  // Lowering TX Power helps prevent power-surge resets from motors
  // WIFI_POWER_8_5dBm is usually enough for a 5-10 meter range
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  
  Serial.println("AP Started. IP: 192.168.4.1");
}

void loop() {
  // Check how many devices are connected to the ESP32
  int connectedClients = WiFi.softAPgetStationNum();

  if (connectedClients > 0) {
    // ACTIVE MODE: Only process Serial data if someone is connected
    if (Serial2.available()) {
      String data = Serial2.readStringUntil('\n');
      ws.textAll(data); 
    }
  } else {
    // IDLE MODE: Clear the Serial buffer so it doesn't get "clogged" 
    // while no one is watching, but don't broadcast anything.
    while(Serial2.available()>0) {
      Serial2.read(); 
    }
  }

  ws.cleanupClients();
  
  // Tiny delay to keep the CPU from running at 100% and heating up
  delay(1); 
}

