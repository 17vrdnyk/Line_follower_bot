#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "LFR Master"
#define BLYNK_AUTH_TOKEN    "YOUR_AUTH_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASS";

// PID & Speed variables to sync
float Kp, Ki, Kd;
int baseSpeed;

BlynkTimer timer;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Talk to Uno
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Optional: Check if we are connected
  if (Blynk.connected()) {
    Serial.println("Blynk is Online! Let's Race!");
  
  // Update data every 100ms
  timer.setInterval(100L, handleTelemetry);
}

void loop() {
  Blynk.run();
  timer.run();
}

// 1. DATA FROM ROBOT -> BLYNK (Monitoring)
void handleTelemetry() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    
    // Splitting the CSV data: error,sensor_byte,left_speed,right_speed,initial_speed
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    int c3 = data.indexOf(',', c2 + 1);
    int c4 = data.indexOf(',', c3 + 1);

    if (c4 != -1) {
      // 1. Extract values
      float errorVal = data.substring(0, c1).toFloat();
      int sensor     = data.substring(c1 + 1, c2).toInt();
      int left       = data.substring(c2 + 1, c3).toInt();
      int right      = data.substring(c3 + 1, c4).toInt();

      // 2. Push to Blynk
      Blynk.virtualWrite(V0, errorVal);  // <--- YOUR ERROR GRAPH PIN
      Blynk.virtualWrite(V5, sensor); 
      Blynk.virtualWrite(V2, left);   
      Blynk.virtualWrite(V3, right);  
    }
  }
}

// 2. DATA FROM BLYNK -> ROBOT (Tuning Sliders)
// When any slider moves, send the full command to Arduino
void sendToUno() {
  // Format: P[Kp],I[Ki],D[Kd],S[Speed]
  String cmd = "P" + String(Kp) + ",I" + String(Ki) + ",D" + String(Kd) + ",S" + String(baseSpeed);
  Serial2.println(cmd);
}

BLYNK_WRITE(V10) { Kp = param.asFloat(); sendToUno(); }
BLYNK_WRITE(V11) { Ki = param.asFloat(); sendToUno(); }
BLYNK_WRITE(V12) { Kd = param.asFloat(); sendToUno(); }
BLYNK_WRITE(V13) { baseSpeed = param.asInt(); sendToUno(); }
