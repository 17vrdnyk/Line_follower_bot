#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SoftwareSerial.h>

#define LED_PIN 2 // Built-in LED on GPIO2 / D4
// Define pins for SoftwareSerial
// D5 (GPIO14) as RX, D6 (GPIO12) as TX
SoftwareSerial swSerial(14, 12); 

const char* ssid = "LFR_Master_AP";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    // 1. Turn LED ON (Start of receiving data)
    digitalWrite(LED_PIN, LOW);

    String command = "";
    for(size_t i=0; i<len; i++) command += (char)data[i];
    
    // Send data to Arduino Uno via SoftwareSerial
    swSerial.println(command); 
    
    // Debug output to Serial Monitor
    Serial.println("Sent to Uno: " + command);

    // 2. Simple "Blink" effect
    // We use a tiny delay so your eyes can actually see the flash
    delay(50); 
    digitalWrite(LED_PIN, HIGH); // Turn LED OFF
  }
}

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Start with LED OFF (Active Low)

  // Hardware Serial for Debugging
  Serial.begin(115200);
  
  // Software Serial for Uno Communication (Match Uno's baud rate)
  swSerial.begin(9600);

  // Start Access Point
  WiFi.softAP(ssid, password);
  
  // Setup WebSocket
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  
  Serial.println("");
  Serial.println("AP Started.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  // Read data coming FROM the Uno
  if (swSerial.available()) {
    String data = swSerial.readStringUntil('\n');
    // Broadcast data to all connected WebSocket clients
    ws.textAll(data); 
    Serial.println("From Uno: " + data);
  }
  
  // Clean up disconnected clients
  ws.cleanupClients();
}
