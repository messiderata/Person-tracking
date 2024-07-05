#include <Servo.h>

Servo servo;
int prevX = 0;  // Previous x-coordinate
const int centerX = 320;  // Assuming frame width is 640
const int angleIncrement = 3.5;  // Angle increment for each step

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Attach the servo to pin 9
  servo.attach(9);
}

void loop() {
  // Check if data is available to read from serial
  if (Serial.available() > 0) {
    // Read the raw coordinates from serial
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    
    // Parse x-coordinate
    float x = data.substring(0, commaIndex).toFloat();
    
    // Implementing PID controller
    float kp = 0.1;
    float ki = 0.01;
    float kd = 0.01;
    float targetX = 320;  // Target x-coordinate, adjust as needed
    
    float error = targetX - x;
    float integral = error; // Initial integral term
    float prev_error = error;
    
    float output = kp * error + ki * integral + kd * (error - prev_error);
    
    // Calculate deviation from the center
    int devX = output;
    
    // Move the servo based on deviation
    int servoAngleX = map(devX, -centerX, centerX, 180, 0);
    
    // Adjust servo movement increment based on deviation
    int angleIncrementX = servoAngleX - prevX;
    if (angleIncrementX > angleIncrement) {
      servoAngleX = prevX + angleIncrement;
    } else if (angleIncrementX < -angleIncrement) {
      servoAngleX = prevX - angleIncrement;
    }
    prevX = servoAngleX;
    
    // Set the servo angle
    servo.write(servoAngleX);
    
    // Reduce delay for more responsiveness
    delay(5);  // Delay to allow servo to reach position
  }
}
