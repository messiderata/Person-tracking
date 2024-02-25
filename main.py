from ultralytics import YOLO
from pyfirmata import Arduino, util
import threading
import time

# Load your weight
model = YOLO('Actual.pt')

# Set up the Firmata connection to the Arduino board
board = Arduino('COM5')  # Update with your port

# Start an iterator thread so that serial buffer doesn't overflow
it = util.Iterator(board)
it.start()

# Define the pin for the servo motor
servo_pin = 9  # Change this to the pin number connected to your servo
servo = board.get_pin('d:{}:s'.format(servo_pin))

# Define the center position of the camera
camera_center = 320  # Assuming the camera resolution width is 640 pixels

# Define the threshold for centering the object and avoiding noise
centering_threshold = 10  # Adjust as needed 50
noise_threshold = 10 # Adjust as needed 10

# Counter to track consecutive frames without detecting any object
no_object_count = 0

# Function to run inference and servo control
def run_inference_and_servo_control():
    global no_object_count
    
    while True:
        results = model.predict(source="2", save=False, imgsz=640, conf=0.8, show=True, stream=True)
        object_detected = False  # Flag to indicate if any object is detected in the current frame
        
        for r in results:
            boxes = r.boxes
            # Convert the tensor to a list of lists
            bounding_boxes = boxes.xyxy.tolist()
            
            if bounding_boxes:
                # Reset the counter if an object is detected
                no_object_count = 0
                
                # Get the bounding box of the first detected object
                bbox = bounding_boxes[0]
                x_center = (bbox[0] + bbox[2]) / 2  # Calculate the x-coordinate of the object center
                
                # Calculate the error between the object center and the camera center
                error = x_center - camera_center
                
                # Adjust the servo angle only if the error exceeds the noise threshold                                                     
                if abs(error) > noise_threshold:
                    # Set the angle of the servo based on the error, in the opposite direction
                    servo_angle = 90 - int(error * 0.09)  # P-control
                    
                    # Make sure the servo angle is within the valid range
                    servo_angle = max(0, min(180, servo_angle))
                    
                    # Move the servo to the calculated angle
                    servo.write(servo_angle)
                    print("Servo angle:", servo_angle)
                else:
                    print("Object close to center, no movement needed.")
                
                object_detected = True
                
        if not object_detected:
            # Increment the counter if no object is detected
            no_object_count += 1
            
            # If no object is detected for a prolonged period, reset the servo to center position
            if no_object_count >= 20:
                servo.write(90)  # Move the servo to the middle position
                print("No object detected for 100 consecutive frames. Stopping servo at 90 degrees.")
                no_object_count = 0  # Reset the counter
        
        # Add a small delay to avoid excessive servo movements
        time.sleep(0.1)

# Create and start the thread
inference_thread = threading.Thread(target=run_inference_and_servo_control)
inference_thread.daemon = True  # Daemonize the thread so it automatically closes when the main program exits
inference_thread.start()

# The main thread can continue doing other tasks if needed
# For example, you can add code here to interact with the user interface, sensors, etc.

# The main thread will wait here until the inference thread finishes
inference_thread.join()
