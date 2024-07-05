from ultralytics import YOLO
import serial
import time

# PID controller parameters
kp = 0.2
ki = 0.01
kd = 0.01
target_x = 320  # Target x-coordinate, adjust as needed

# Initialize PID variables
prev_error = 0
integral = 0

# Load your weight
model = YOLO('best5.pt')

# Set up the serial connection to the Arduino board
port = '/dev/ttyUSB0'  # Update with your port
baudrate = 9600  # Update with your baudrate
ser = serial.Serial(port, baudrate)

# PID function
def pid_control(current_x):
    global prev_error, integral
    error = target_x - current_x
    integral += error
    derivative = error - prev_error
    prev_error = error
    output = kp * error + ki * integral + kd * derivative
    return output

# Run inference on live camera
while True:
    results = model.predict(source="1", save=False, imgsz=640, conf=0.5, show=True, stream=True)
    for r in results:
        boxes = r.boxes
        # Convert the tensor to a list of lists
        bounding_boxes = boxes.xyxy.tolist()
        # Send the processed x-coordinate to Arduino using PID control
        for bbox in bounding_boxes:
            x_center = bbox[0] + (bbox[2] - bbox[0]) / 2  # Calculate the x-coordinate of the object center
            pid_output = pid_control(x_center)
            # Send the processed x-coordinate to Arduino as bytes
            ser.write(f"{pid_output}\n".encode())
            # Add a small delay to avoid flooding the serial port
            time.sleep(0.1)

# Close the serial connection
ser.close()
