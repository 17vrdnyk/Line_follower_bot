# Line_follower_bot
Code Documentation
Overview
This Arduino code is designed for a line-following robot. The robot uses five infrared sensors to detect a black line on a white surface. The proportional-integral-derivative (PID) algorithm is implemented to control the motors and keep the robot following the line. The motor speed is adjusted based on the error calculated from the sensor readings.

`Global Variables`
`leftSensorL1`, `leftSensorL2`, `middleSensor`, `rightSensorR1`, `rightSensorR2`:            Variables to store the digital readings from the infrared sensors.

`b`,` w`:                                                                              Constants representing black and white line readings, respectively.

`initial_motor_speed`:                                                               Initial speed of the motors.

sensor[5]:                                                                         Array to store sensor values.

`Kp`,`Ki`, `Kd`:                                                                        PID constants for proportional, integral, and derivative terms.

`error`, `P`,`I`, `D`, `PID_value`:                                                         Variables for error calculation and PID control.

`previous_error`, `previous_I`:                                                        Variables to store the previous error and integral term for PID control.

flag: Flag variable (not currently used).

Setup Function:              The setup function initializes the pin modes for sensor inputs and motor outputs and sets the initial conditions for the robot. It also starts 
                             the serial communication for debugging.

Loop Function:               The loop function is the main execution loop. It calls three main functions in sequence:

`read_sensor_values`:          Reads the sensor values and determines the robot's position relative to the line.

`calculate_pid`:               Applies the PID algorithm to calculate the motor speed adjustments based on the error.

`motor_control`:               Adjusts the motor speeds according to the PID values and controls the robot's movement.

`read_sensor_values Function`: This function reads the digital values from the infrared sensors and interprets them to determine the robot's position relative to the line.
                             It sets the error variable based on the sensor readings.

`calculate_pid Function`:      This function calculates the PID value based on the error, proportional, integral, and derivative terms. The calculated PID value is used to 
                              adjust the motor speeds.

`motor_control` Function:      This function adjusts the motor speeds based on the PID values and controls the robot's movement. It includes logic to prevent the robot from 
                              stopping completely when the error is zero.

Movement Functions
`forward`, `reverse`, `right`, `left`, `sharpRightTurn`, `sharpLeftTurn`, `stop_bot`, `setspeed`: Functions to control specific movements of the robot.

Additional Notes
The code uses the Serial communication for debugging purposes, printing error and motor speed values.
The robot's behavior is determined by the conditions set in the read_sensor_values function, which maps sensor readings to specific error values.
The PID constants (Kp, Ki, Kd) may need to be tuned based on the specific characteristics of the robot and the environment.
