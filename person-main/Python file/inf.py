from ultralytics import YOLO
import serial
import time


# Load your weight
model = YOLO('best5.pt')

# Set up the serial connection to the Arduino board
port = 'COM6'  # Update with your port
baudrate = 9600  # Update with your baudrate
ser = serial.Serial(port, baudrate)

# Run inference on live camera
while True:
    results = model.predict(source="1", save=False, imgsz=640, conf=0.60, show=True, stream=True)
    for r in results:
        boxes = r.boxes
        # Convert the tensor to a list of lists
        bounding_boxes = boxes.xyxy.tolist()
        # Send only the x-coordinates to Arduino
        for bbox in bounding_boxes:
            x_center = bbox[0] + (bbox[2] - bbox[0]) / 2  # Calculate the x-coordinate of the object center
            # x_center=(bbox[0]+bbox[2])/2
            
            # Send the x-coordinate to Arduino as bytes
            ser.write(f"{int(x_center)}\n".encode())
            # Add a small delay to avoid flooding the serial port
            time.sleep(0.1) 

# Close the serial connection
ser.close()

# from ultralytics import YOLO
# import serial
# import time

# class PIDController:
#     def __init__(self, Kp, Ki, Kd, setpoint):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.setpoint = setpoint
#         self.prev_error = 0
#         self.integral = 0

#     def calculate(self, feedback):
#         error = self.setpoint - feedback

#         # Proportional term
#         P = self.Kp * error

#         # Integral term
#         self.integral += error
#         I = self.Ki * self.integral

#         # Derivative term
#         derivative = error - self.prev_error
#         D = self.Kd * derivative
#         self.prev_error = error

#         return P + I + D

# # Load your weight
# model = YOLO('best5.pt')

# # Set up the serial connection to the Arduino board
# port = '/dev/ttyUSB0'  # Update with your port
# baudrate = 9600  # Update with your baudrate
# ser = serial.Serial(port, baudrate)

# # Initialize PID controller
# pid = PIDController(Kp=0.09, Ki=0.3, Kd=0.09, setpoint=320)  # Adjust Kp, Ki, Kd values as needed

# # Run inference on live camera
# while True:
#     results = model.predict(source="0", save=False, imgsz=640, conf=0.55, show=True, stream=True)
#     for r in results:
#         boxes = r.boxes
#         # Convert the tensor to a list of lists
#         bounding_boxes = boxes.xyxy.tolist()
#         # Send only the x-coordinates to Arduino
#         for bbox in bounding_boxes:
#             x_center = bbox[0] + (bbox[2] - bbox[0]) / 2  # Calculate the x-coordinate of the object center
            
#             # Use PID controller to calculate servo position
#             servo_position = pid.calculate(x_center)
#             print(servo_position)
#             # Send the servo position to Arduino as bytes
#             ser.write(f"{int(servo_position)}\n".encode())
            
#             # Add a small delay to avoid flooding the serial port
#             time.sleep(0.1)

# # Close the serial connection
# ser.close()
