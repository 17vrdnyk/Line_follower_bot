unsigned long lastSendTime = 0;

int leftSensorL1 , leftSensorL2 ,leftSensorL3, leftSensorL4, rightSensorR4, rightSensorR3, rightSensorR2, rightSensorR1;
int w = HIGH; // blackline, if "Inverted" (White line on Black background), you only have to change one line of code: int w = LOW;

unsigned long blackBoxTimer = 0;   // To track how long we see all black
unsigned long whiteGapTimer = 0;   // To track how long we see all white
bool potentialGap = false;         // To flag if we are in a "peek" state

// --- Pin Definitions ---
// Sensors (Using Digital GPIOs)
const int PIN_L1 = 7; // Left Outer
const int PIN_L2 = 4;
const int PIN_L3 = 3;
const int PIN_L4 = 2; // Left Inner
const int PIN_R4 = 13; // Right Inner
const int PIN_R3 = 12;
const int PIN_R2 = 11; 
const int PIN_R1 = 8; // Right Outer (VN pin)

// Motors
// Left Motor
const int ENA = 10; // PWM Pin
const int IN1 = A0;
const int IN2 = A1;
// Right Motor
const int ENB = 9; // PWM Pin
const int IN3 = A2;
const int IN4 = A3;

// --- Shared Global Variables for Telemetry ---
float error = 0;              // Updated in read_sensor_values()
byte sensor_byte_binary = 0;  // Updated in read_sensor_values()
int left_motor_speed = 0;     // Updated in motor_control()
int right_motor_speed = 0;    // Updated in motor_control()
int initial_motor_speed = 105; //Initial Speed of Motor

// PID Constants
float Kp = 35.96; //35.86
float Ki = 0.21; //0.13
float Kd = 9.074; //9.088

float P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
byte current_sensor_byte = 0; // Global to share with Serial

int flag = 0;

void setup() 
{
   // Sensor Pins
  pinMode(PIN_L1, INPUT);
  pinMode(PIN_L2, INPUT);
  pinMode(PIN_L3, INPUT);
  pinMode(PIN_L4, INPUT);
  pinMode(PIN_R4, INPUT);
  pinMode(PIN_R3, INPUT);
  pinMode(PIN_R2, INPUT);
  pinMode(PIN_R1, INPUT);

  // Motor Control Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //pwm control
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

 Serial.begin(9600);
 }

void loop()
{
  read_sensor_values();
  calculate_pid();
 motor_control();

 // New: Handle communication with ESP32
  check_for_tuning();
  // Only send to website every 50 milliseconds
  if (millis() - lastSendTime > 50) {
    send_telemetry();
    lastSendTime = millis();
  }
 }

// --- NEW FUNCTION: Send data for Website Graphing ---
void send_telemetry() {
  // Format: error,sensor_byte,left_speed,right_speed,initial_speed
  // This CSV format is easy for Javascript to split using .split(',')
  // We send a comma-separated string: error, sensor_byte, L_speed, R_speed, Initial_speed
  Serial.print(error);                Serial.print(",");
  Serial.print(sensor_byte_binary);   Serial.print(","); // Sends as a decimal number (0-255)
  Serial.print(left_motor_speed);     Serial.print(",");
  Serial.print(right_motor_speed);    Serial.print(",");
  Serial.println(initial_motor_speed); // println adds the '\n' which tells ESP32 the line is done
}

// --- NEW FUNCTION: Receive PID/Speed changes from Website ---
void check_for_tuning() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    // Check if the string starts with 'P' (our custom protocol)
    // Example format: P35.96,I0.21,D9.07,S105
    if (input.startsWith("P")) {
       // Using simple parsing to avoid memory heavy sscanf
       int comma1 = input.indexOf(',');
       int comma2 = input.indexOf(',', comma1 + 1);
       int comma3 = input.indexOf(',', comma2 + 1);

       Kp = input.substring(1, comma1).toFloat();
       Ki = input.substring(comma1 + 2, comma2).toFloat(); // +2 skips the 'I'
       Kd = input.substring(comma2 + 2, comma3).toFloat(); // +2 skips the 'D'
       initial_motor_speed = input.substring(comma3 + 2).toInt(); // +2 skips the 'S'
       
       // Reset I to prevent integral windup after tuning
       I = 0; 
    }
  }
}

void read_sensor_values()
{
  // 1. Read all sensors into an array
  int s[8];
  s[0] = digitalRead(PIN_L1); // Left Outer
  s[1] = digitalRead(PIN_L2);
  s[2] = digitalRead(PIN_L3);
  s[3] = digitalRead(PIN_L4); // Left Inner
  s[4] = digitalRead(PIN_R4); // Right Inner
  s[5] = digitalRead(PIN_R3);
  s[6] = digitalRead(PIN_R2);
  s[7] = digitalRead(PIN_R1); // Right Outer

  // 2. Convert to Bitmask (a single number representing the pattern)
  // Example: 00011000 means the middle two sensors are on the line
  sensor_byte_binary = 0;
  for (int i = 0; i < 8; i++) {
    if (s[i] == w) {
      sensor_byte_binary |= (1 << (7 - i)); 
    }
  }

  // 3. HYBRID LOGIC
  
  // --- CASE A: All sensors see the line (Crossroad or Black Box) ---
  if (sensor_byte_binary == 0b11111111) {
    if (blackBoxTimer == 0) blackBoxTimer = millis(); // Start timing only once
    
    // Check if we've been on black long enough to stop
    if (millis() - blackBoxTimer > 1000) { 
      stop_bot(); 
      while(1); // Stop forever
    }
    
    error = 0; // If under 1000ms, just treat as a crossroad (go straight)
    
    // IMPORTANT: Since we ARE in a black box, we don't reset the timer here.
    // We also reset the GAP flags just in case we came from a gap.
    potentialGap = false;
    whiteGapTimer = 0;
  }
  
  // CASE B: All sensors see white (Gap or Off Track)
  // --- CASE B: All sensors see white (Gap or Lost Line) ---
  else if (sensor_byte_binary == 0b00000000) {
    blackBoxTimer = 0; // <--- RESET BlackBox timer here (because we aren't seeing black)
    
    if (!potentialGap) {
      whiteGapTimer = millis();
      potentialGap = true;
    }

    if (millis() - whiteGapTimer < 1000) {
      error = 0; // "Peek" forward
    } 
    // Out of Time? Hard Turn Phase
    else {
      if (previous_error < 0) error = -9; 
      else if (previous_error > 0) error = 9;
    }
  }

  // CASE C: Sharp 90-degree turns (Optional Pattern Matching)
  else if (sensor_byte_binary == 0b11100000) { error = -7; } // Sharp Left
  else if (sensor_byte_binary == 0b00000111) { error = 7; }  // Sharp Right

  // CASE E: Sharp exceptional turns (Optional Pattern Matching)
  else if (sensor_byte_binary == 0b01100000) { error = -5; } // mild Left
  else if (sensor_byte_binary == 0b00000110) { error = 5; }  // mild Right

  else if (sensor_byte_binary == 0b00110000) { error = -3; } // slight Left
  else if (sensor_byte_binary == 0b00001100) { error = 3; }  // slight Right

  else if (sensor_byte_binary == 0b0111000) { error = -5; } // mild Left
  else if (sensor_byte_binary == 0b00001110) { error = 5; }  // mild Right

  else if (sensor_byte_binary == 0b11110000) { error = -9; } // Sharp Left
  else if (sensor_byte_binary == 0b00001111) { error = 9; }  // Sharp Right

  else if (sensor_byte_binary == 0b01101000) { error = -3; } // Sharp Left
  else if (sensor_byte_binary == 0b00010110) { error = 3; }  // Sharp Right

  else if (sensor_byte_binary == 0b01111000) { error = -5; } // mild Left
  else if (sensor_byte_binary == 0b00011110) { error = 5; }  // mild Right

  // CASE D: Normal Line Following (Weighted Average)
  else {
    // Reset ALL flags because we are safely on a line pattern
    blackBoxTimer = 0;
    potentialGap = false;
    whiteGapTimer = 0;

    float total_weight = 0;
    int active_sensors = 0;
    
    // Assigning weights from far-left to far-right
    // L1(-7), L2(-5), L3(-3), L4(-1), R4(1), R3(3), R2(5), R1(7)
    int weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

    for (int i = 0; i < 8; i++) {
      if (s[i] == w) {
        total_weight += weights[i];
        active_sensors++;
      }
    }

    if (active_sensors > 0) {
      error = total_weight / active_sensors; // The smooth PID error
    }
  }

  // Debugging (Keep Serial prints minimal during racing to save speed)
  // Serial.print("Pattern: "); Serial.print(sensor_byte, BIN);
  // Serial.print(" Error: "); Serial.println(error);
 
  forward(75);

} 

void calculate_pid() {
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, -255, 255);
  right_motor_speed = constrain(right_motor_speed, -255, 255);

  /*Serial.print(PID_value);
    Serial.print("\t");
    Serial.print(left_motor_speed);
    Serial.print("\t");
    Serial.println(right_motor_speed);*/


    if(left_motor_speed>0)
  {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(IN3, HIGH);   
    digitalWrite(IN4, LOW);
    analogWrite(ENA,left_motor_speed);
  }
  else 
  {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, (left_motor_speed));
    //analogWrite(ENA, abs(left_motor_speed));
  }
  
 
  if(right_motor_speed>0)
  {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENB, right_motor_speed );
  }
  else 
  {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENB, (right_motor_speed));
    //analogWrite(ENB, (right_motor_speed));
  }

  
  //following lines of code are to make the bot move forward
  
}


void forward(int s)
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  setspeed(s);
  
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH); 
}

void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3,LOW);
 digitalWrite(IN4,HIGH);
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,LOW); 
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
}
void sharpRightTurn() 
{
  /*The pin numbers and high, low values might be different depending on your connections */
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,HIGH);
 digitalWrite(IN1,HIGH);
 digitalWrite(IN2,LOW); 
}
void sharpLeftTurn() 
{
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void setspeed(int Speed)
{
  analogWrite(ENA,Speed);
  analogWrite(ENB,Speed);  
}
