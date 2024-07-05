#include <Servo.h>

Servo servo;
int prevX = 0;  // Previous x-coordinate
const int centerX = 320;  // Assuming frame width is 640
const int threshold = 20;  // Dead zone range
const int angleIncrement = 1.5;  // Angle increment for each step

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Attach the servo to pin 9
  servo.attach(9);
  servo.write(90);
}

void loop() {
  // Check if data is available to read from serial
  if (Serial.available() > 0) {
    // Read the raw coordinates from serial
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    
    // Parse x-coordinate
    int x = data.substring(0, commaIndex).toInt();
    
    // Calculate deviation from the center
    int devX = x - centerX;

    // Apply dead zone
    if (abs(devX) < threshold) {
      devX = 0;
    }
    
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
    delay(15);  // Delay to allow servo to reach position
  }
}


// #include <Servo.h>

// Servo servo;
// int targetX = 320;      // Target x-coordinate (center of the frame)
// double kp = 0.1;        // Proportional gain
// double ki = 0.01;       // Integral gain
// double kd = 0.005;      // Derivative gain
// int deadZone = 10;      // Dead zone range

// int prevError = 0;
// int integral = 0;

// void setup() {
//   // Initialize serial communication
//   Serial.begin(9600);
  
//   // Attach the servo to pin 9
//   servo.attach(9);
//   servo.write(90); // Set servo to initial position
// }

// void loop() {
//   // Read the x-coordinate from serial
//   if (Serial.available() > 0) {
//     int x = Serial.parseInt();
    
//     // Apply dead zone
//     if (abs(x - targetX) < deadZone) {
//       x = targetX;
//     }
    
//     // Calculate the error
//     int error = targetX - x;
    
//     // Calculate PID terms
//     int proportional = kp * error;
//     integral += error;
//     int derivative = kd * (error - prevError);
    
//     // Calculate PID output
//     int output = proportional + (ki * integral) + derivative;
    
//     // Limit the output to avoid sudden movements
//     output = constrain(output, -10, 10);
    
//     // Update servo position
//     int newPos = servo.read() + output;
//     servo.write(constrain(newPos, 0, 180));
    
//     // Save current error for the next iteration
//     prevError = error;
//   }
// }
