

int leftSensorL1 , leftSensorL2 ,leftSensorL3, leftSensorL4, rightSensorR4, rightSensorR3, rightSensorR2, rightSensorR1;
int w = HIGH; // blackline, if "Inverted" (White line on Black background), you only have to change one line of code: int w = LOW;


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

//Initial Speed of Motor
int initial_motor_speed = 105;

// PID Constants
float Kp = 35.96; //35.86
float Ki = 0.21; //0.13
float Kd = 9.074; //9.088

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

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
  byte sensor_byte = 0;
  for (int i = 0; i < 8; i++) {
    if (s[i] == w) {
      sensor_byte |= (1 << (7 - i)); 
    }
  }

  // 3. HYBRID LOGIC
  
  // CASE A: All sensors see the line (Crossroad / T-Junction)
  if (sensor_byte == 0b11111111) {
    error = 0; 
    // You could also add code here to stop or count a lap
  }
  
  // CASE B: All sensors see white (Off the track)
  else if (sensor_byte == 0b00000000) {
    // Memory logic: Keep turning in the last known direction to find the line
    if (previous_error < 0) error = -10; 
    else if (previous_error > 0) error = 10;
  }

  // CASE C: Sharp 90-degree turns (Optional Pattern Matching)
  else if (sensor_byte == 0b11100000) { error = -7; } // Sharp Left
  else if (sensor_byte == 0b00000111) { error = 7; }  // Sharp Right

  // CASE D: Normal Line Following (Weighted Average)
  else {
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
    digitalWrite(IN1, HIGH);   
    digitalWrite(IN2, LOW);
    analogWrite(ENA,left_motor_speed);
  }
  else 
  {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, (left_motor_speed));
    //analogWrite(ENA, abs(left_motor_speed));
  }
  
 
  if(right_motor_speed>0)
  {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, right_motor_speed );
  }
  else 
  {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, (right_motor_speed));
    //analogWrite(ENB, (right_motor_speed));
  }

  
  //following lines of code are to make the bot move forward
  
}


void forward(int s)
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  setspeed(s);
  
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
}

void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN1,LOW);
 digitalWrite(IN2,HIGH);
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,LOW); 
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}
void sharpRightTurn() 
{
  /*The pin numbers and high, low values might be different depending on your connections */
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,HIGH);
 digitalWrite(IN3,HIGH);
 digitalWrite(IN4,LOW); 
}
void sharpLeftTurn() 
{
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setspeed(int Speed)
{
  analogWrite(ENA,Speed);
  analogWrite(ENB,Speed);  
}
