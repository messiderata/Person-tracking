from ultralytics import YOLO
from pyfirmata import Arduino, util
import threading
import time

# Load your weight
model = YOLO('Actual.pt')

# Set up the Firmata connection to the Arduino board
board = Arduino('COM7') # Adjust baudrate and timeout as needed


# Start an iterator thread so that serial buffer doesn't overflow
it = util.Iterator(board)
it.start()

# Define the pin for the servo motor
servo_pin = 9 # Change this to the pin number connected to your servo
servo = board.get_pin('d:{}:s'.format(servo_pin))

# Define the center position of the camera
camera_center = 320  # Assuming the camera resolution width is 640 pixels

# Define the proportional, integral, and derivative gains (adjust as needed)
Kp = 0.005
Ki = 0.01
Kd = 0.05

# Counter to track consecutive frames without detecting any object
no_object_count = 0

# Initialize variables for PID controller
prev_error = 0
integral = 0

update_delay = 0.05 
# Function to run inference and servo control
def run_inference_and_servo_control():
    global no_object_count, prev_error, integral
    
    while True:
        results = model.predict(source="1", save=False, imgsz=640, conf=0.5, show=True, stream=True,iou=0)
        object_detected = False  # Flag to indicate if any object is detected in the current frame
        
        for r in results:
            boxes = r.boxes
            bounding_boxes = boxes.xyxy.tolist()
            
            if bounding_boxes:
                no_object_count = 0
                
                bbox = bounding_boxes[0]
                x_center = (bbox[0] + bbox[2]) / 2
                
                error = x_center - camera_center
                
                integral = integral + error
                derivative = error - prev_error
                
                servo_adjustment = Kp * error + Ki * integral + Kd * derivative
                servo_angle = 90 - int(servo_adjustment)
                servo_angle = max(0, min(180, servo_angle))
                
                servo.write(servo_angle)
                print("Servo angle:", servo_angle)
                
                prev_error = error
                
                object_detected = True
                
        if not object_detected:
            no_object_count += 1
            
            if no_object_count >= 20:
                servo.write(90)
                print("No object detected for 100 consecutive frames. Stopping servo at 90 degrees.")
                no_object_count = 0
        
        time.sleep(update_delay)  # Adjusted delay for smoother movements

# Create and start the thread
inference_thread = threading.Thread(target=run_inference_and_servo_control)
inference_thread.daemon = True  # Daemonize the thread so it automatically closes when the main program exits
inference_thread.start()

# The main thread can continue doing other tasks if needed
# For example, you can add code here to interact with the user interface, sensors, etc.

# The main thread will wait here until the inference thread finishes
inference_thread.join()
