#include <Servo.h>

Servo servo;
int prevX = 0;  // Previous x-coordinate

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
    int x = data.substring(0, commaIndex).toInt();
    
    // Calculate the center of the frame
    int centerX = 320;  // Assuming frame width is 640
    
    // Calculate deviation from the center
    int devX = x - centerX;
    
    // Move the servo based on deviation
    int servoAngleX = map(devX, -centerX, centerX, 180, 0);
    
    // Rotate servo by 10 degrees for every coordinate movement towards the center
    if (x != prevX) {
      int angleIncrementX = servoAngleX - prevX;
      if (angleIncrementX > 5) {
        servoAngleX = prevX + 5;
      } else if (angleIncrementX < -5) {
        servoAngleX = prevX - 5;
      }
      prevX = servoAngleX;
      
      // Set the servo angle
      servo.write(servoAngleX);
      delay(15);  // Delay to allow servo to reach position
    }
  }
}
