int leftSensorL1;
int leftSensorL2;
int middleSensor;
int rightSensorR1;
int rightSensorR2;
int b=LOW;  // blackline
int w=HIGH;   // whiteline

//Initial Speed of Motor
int initial_motor_speed = 90;

// Initial Values of Sensors
int sensor[5] = {0, 0, 0, 0, 0};


// PID Constants
float Kp = 35.86;
float Ki = 0.13;
float Kd = 9.088; //9.088

float error = 0, P = 0, I = 0, D = 0, PID_value , bob = 0;
float previous_error = 0, previous_I = 0;

int flag = 0;

void setup() 
{
 pinMode(2,INPUT);
 pinMode(3,INPUT);
 pinMode(4,INPUT);
 pinMode(5,INPUT);
 pinMode(7,INPUT);
 pinMode(6,OUTPUT);
 pinMode(9,OUTPUT);
 pinMode(10,OUTPUT);
 pinMode(11,OUTPUT);
 pinMode(12,OUTPUT);
 pinMode(13,OUTPUT);
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
 leftSensorL1=digitalRead(2);
 leftSensorL2=digitalRead(3);
 middleSensor=digitalRead(7);
 rightSensorR1=digitalRead(4);
 rightSensorR2=digitalRead(5);

/*
d1 right most
d2 right
d4 middle
d5 left
d6 left most
 
 */

  /*
    Serial.print(sensor[0]);
    Serial.print("\t");
    Serial.print(sensor[1]);
    Serial.print("\t");
    Serial.print(sensor[2]);
    Serial.print("\t");
    Serial.println(sensor[3]);*/


  if(leftSensorL1==w  && leftSensorL2==w  && middleSensor==w && rightSensorR1==b  && rightSensorR2==b  )//hard left
 {
  error = -3;  //***
  }
  
 if(leftSensorL1==w  && leftSensorL2==b && middleSensor==w && rightSensorR1==b  && rightSensorR2==b  )//left
 {  
  error = -6;
 }
 

 if(leftSensorL1==w && leftSensorL2==w && middleSensor==w && rightSensorR1==w  && rightSensorR2==b  )//left**
 {  
  error = -3;//30
 }

  if(leftSensorL1==w  && leftSensorL2==w && middleSensor==b && rightSensorR1==b  && rightSensorR2==b  )//left 
 {  
 error = -2;
 }
   if(leftSensorL1==w  && leftSensorL2==b && middleSensor==b && rightSensorR1==b  && rightSensorR2==b  )//left
 {  
  error = -2;
 }
   if(leftSensorL1==b  && leftSensorL2==w && middleSensor==w && rightSensorR1==b  && rightSensorR2==b  )//left  /// CHANGES
 {
 error = -1;
 }
  if(leftSensorL1==b  && leftSensorL2==w && middleSensor==b && rightSensorR1==b  && rightSensorR2==b  )//left
 {  
 error = -1;
 } 

if(leftSensorL1==b  && leftSensorL2==b && middleSensor==w && rightSensorR1==b  && rightSensorR2==b  )//forward
 {
   error = 0;

}
if(leftSensorL1==b  && leftSensorL2==w && middleSensor==w && rightSensorR1==w  && rightSensorR2==b  )//forward
 {
   error = 0;

}
  if(leftSensorL1==b  && leftSensorL2==b && middleSensor==b && rightSensorR1==w  && rightSensorR2==b  )//right
 {
  error = 1;
 }
 if(leftSensorL1==b  && leftSensorL2==b && middleSensor==w && rightSensorR1==w  && rightSensorR2==b  )//right  /// CHANGES
 {
  error = 1;
 }
 if(leftSensorL1==b  && leftSensorL2==b && middleSensor==b && rightSensorR1==b  && rightSensorR2==w  )//right
 {
  error = 2;
 }
  if(leftSensorL1==b  && leftSensorL2==b && middleSensor==b && rightSensorR1==w  && rightSensorR2==w  )//right
 {
 error = 2;
 }
 if(leftSensorL1==b  && leftSensorL2==w && middleSensor==w && rightSensorR1==w  && rightSensorR2==w  )//right****
 {
 error = 3;//30
 }

 if(leftSensorL1==b  && leftSensorL2==b && middleSensor==w && rightSensorR1==b  && rightSensorR2==w  )//right
 {
  error = 6;
 }
  if(leftSensorL1==b  && leftSensorL2==b && middleSensor==w && rightSensorR1==w  && rightSensorR2==w  )//hard right
 {
  error = 3;
 }
if(leftSensorL1==w  && leftSensorL2==w && middleSensor==w && rightSensorR1==w  && rightSensorR2==w  )//hard left*****
 {
 error= -25;
 }
 if(leftSensorL1==w  && leftSensorL2==b && middleSensor==w && rightSensorR1==w  && rightSensorR2==w  )//hard left*****
 {
  error = 30;
 }
 if(leftSensorL1==w  && leftSensorL2==w && middleSensor==w && rightSensorR1==b  && rightSensorR2==w  )//hard right*****
 {
  error = 30;
 }
  if(leftSensorL1==w  && leftSensorL2==b && middleSensor==w && rightSensorR1==w  && rightSensorR2==w  )//hard left*****
 {
  error = 30;
 }
 if(leftSensorL1==w  && leftSensorL2==w && middleSensor==w && rightSensorR1==b  && rightSensorR2==w  )//hard right*****
 {
  error = 30;
 }


 // important conditions
if(leftSensorL1==w  && leftSensorL2==w && middleSensor==w && rightSensorR1==w  && rightSensorR2==w  )//hard left*****
 {
 error= -25;
 }
 
  forward();



} 

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
 
  previous_I = I;
  previous_error = error;
}

void motor_control()
{
       if(error==0){
    while(1){
      initial_motor_speed +=1;
    }
  } else initial_motor_speed=80;
  Serial.print(error);
  Serial.print(" ");
  Serial.print(initial_motor_speed);
  Serial.println(" ");
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

  /*analogWrite(6, left_motor_speed); //Left Motor Speed
  analogWrite(9, right_motor_speed); //Right Motor Speed
  */

    if(left_motor_speed>0)
  {
    digitalWrite(12, HIGH);   
    digitalWrite(13, LOW);
    analogWrite(6,left_motor_speed);
  }
  else 
  {
    digitalWrite(12, LOW); 
    digitalWrite(13, HIGH);
    analogWrite(6, (left_motor_speed));
  }
  
 
  if(right_motor_speed>0)
  {
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);
    analogWrite(9,right_motor_speed );
  }
  else 
  {
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);
    analogWrite(9, (right_motor_speed) );
  }

  
  //following lines of code are to make the bot move forward
  
}


void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(10,HIGH);
  digitalWrite(11,LOW);
  digitalWrite(12,HIGH);
  digitalWrite(13,LOW);
  setspeed(75);
  
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(10,LOW);
  digitalWrite(11,HIGH);
  digitalWrite(12,LOW);
  digitalWrite(13,HIGH); 
}

void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(10,LOW);
 digitalWrite(11,HIGH);
 digitalWrite(12,LOW);
 digitalWrite(13,LOW); 
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  digitalWrite(12,LOW);
  digitalWrite(13,HIGH);
}
void sharpRightTurn() 
{
  /*The pin numbers and high, low values might be different depending on your connections */
 digitalWrite(10,LOW);
 digitalWrite(11,HIGH);
 digitalWrite(12,HIGH);
 digitalWrite(13,LOW); 
}
void sharpLeftTurn() 
{
  digitalWrite(10,HIGH);
  digitalWrite(11,LOW);
  digitalWrite(12,LOW);
  digitalWrite(13,HIGH);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}

void setspeed(int Speed)
{
  analogWrite(6,Speed);
  analogWrite(9,Speed);  
}
