from pyfirmata import Arduino, util
from ultralytics import YOLO
import threading
import time

# Load your weight
model = YOLO('Actual.pt')

# Set up the Firmata connection to the Arduino board
board = Arduino('COM6')  # Update with your port

# Start an iterator thread so that serial buffer doesn't overflow
it = util.Iterator(board)
it.start()

# Define the pin for the servo motor
servo_pin = 9  # Change this to the pin number connected to your servo
servo = board.get_pin('d:{}:s'.format(servo_pin))

# Define the center position of the camera
camera_center = 320  # Assuming the camera resolution width is 640 pixels

# Counter to track consecutive frames without detecting any object
no_object_count = 0

# Define increment values for different slices
increment_slices = {
    1: 3,  # Increment for the first slice from the left edge
    2: 2,  # Increment for the second slice
    3: 1,  # Increment for the third slice
    4: 2,  # Increment for the fourth slice from the right edge
}

# Function to calculate slice number based on object position
def get_slice_number(object_center):
    slice_width = camera_center / 2  # Divide the camera width into two equal slices
    if object_center < slice_width:
        return 1
    elif object_center < camera_center:
        return 2
    elif object_center < camera_center + slice_width:
        return 3
    else:
        return 4

# Function to run inference and servo control
def run_inference_and_servo_control():
    global no_object_count
    
    while True:
        results = model.predict(source="0", save=False, imgsz=640, conf=0.70, show=True, stream=True,iou=1)
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
                
                # Determine slice number based on object position
                slice_number = get_slice_number(x_center)
                
                # Get increment value for the slice
                increment = increment_slices[slice_number]
                
                # Calculate desired servo angle based on object position
                desired_servo_angle = 90 + increment
                
                # Ensure the servo angle is within the valid range
                desired_servo_angle = max(0, min(180, desired_servo_angle))
                
                # Incrementally move the servo to the desired angle
                servo.write(desired_servo_angle)
                
                object_detected = True
                
        if not object_detected:
            # Increment the counter if no object is detected
            no_object_count += 1
            
            # If no object is detected for a prolonged period, reset the servo to center position
            if no_object_count >= 100:
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
# from pyfirmata import Arduino, util
# from ultralytics import YOLO
# import threading
# import time

# # Load your weight
# model = YOLO('Actual.pt')

# # Set up the Firmata connection to the Arduino board
# board = Arduino('COM5')  # Update with your port

# # Start an iterator thread so that serial buffer doesn't overflow
# it = util.Iterator(board)
# it.start()

# # Define the pin for the servo motor
# servo_pin = 9  # Change this to the pin number connected to your servo
# servo = board.get_pin('d:{}:s'.format(servo_pin))

# # Define the center position of the camera
# camera_center = 320  # Assuming the camera resolution width is 640 pixels

# # Counter to track consecutive frames without detecting any object
# no_object_count = 0

# # Function to run inference and servo control
# def run_inference_and_servo_control():
#     global no_object_count
    
#     while True:
#         results = model.predict(source="2", save=False, imgsz=640, conf=0.70, show=True, stream=True)
#         object_detected = False  # Flag to indicate if any object is detected in the current frame
        
#         for r in results:
#             boxes = r.boxes
#             # Convert the tensor to a list of lists
#             bounding_boxes = boxes.xyxy.tolist()
            
#             if bounding_boxes:
#                 # Reset the counter if an object is detected
#                 no_object_count = 0
                
#                 # Get the bounding box of the first detected object
#                 bbox = bounding_boxes[0]
#                 x_center = (bbox[0] + bbox[2]) / 2  # Calculate the x-coordinate of the object center
                
#                 # Calculate error between the object center and the camera center
#                 error = x_center - camera_center
                
#                 # Calculate desired servo angle based on error
#                 desired_servo_angle = 90 - int(error * 0.08)  # Adjust multiplier as needed
                
#                 # Ensure the servo angle is within the valid range
#                 desired_servo_angle = max(0, min(180, desired_servo_angle))
                
#                 # Move the servo to the calculated angle
#                 servo.write(desired_servo_angle)
                
#                 object_detected = True
                
#         if not object_detected:
#             # Increment the counter if no object is detected
#             no_object_count += 1
            
#             # If no object is detected for a prolonged period, reset the servo to center position
#             if no_object_count >= 100:
#                 servo.write(90)  # Move the servo to the middle position
#                 print("No object detected for 100 consecutive frames. Stopping servo at 90 degrees.")
#                 no_object_count = 0  # Reset the counter
        
#         # Add a small delay to avoid excessive servo movements
#         time.sleep(0.1)

# # Create and start the thread
# inference_thread = threading.Thread(target=run_inference_and_servo_control)
# inference_thread.daemon = True  # Daemonize the thread so it automatically closes when the main program exits
# inference_thread.start()

# # The main thread can continue doing other tasks if needed
# # For example, you can add code here to interact with the user interface, sensors, etc.

# # The main thread will wait here until the inference thread finishes
# inference_thread.join()
