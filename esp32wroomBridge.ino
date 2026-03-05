#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "LFR_Master_AP";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define RXD2 16
#define TXD2 17

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    // 1. Turn LED ON (Start of receiving data)
    digitalWrite(LED_BUILTIN, LOW);
    
    String command = "";
    for(size_t i=0; i<len; i++) command += (char)data[i];
    Serial2.println(command); // Send tuning to Uno
    Serial.println("To Uno: " + command);

    // 2. Simple "Blink" effect
    // We use a tiny delay so your eyes can actually see the flash
    delay(50); 
    digitalWrite(LED_BUILTIN, HIGH); // Turn LED OFF
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Start with LED OFF (Active Low)
  
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  WiFi.softAP(ssid, password);
  
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  
  Serial.println("AP Started. IP: 192.168.4.1");
}

void loop() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    ws.textAll(data); // Broadcast LFR data to all connected web pages
  }
  ws.cleanupClients();
}
