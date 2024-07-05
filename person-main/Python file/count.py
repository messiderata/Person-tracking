from ultralytics import YOLO
import serial
import time

consecutive_count = 0
max_consecutive_count = 2  # Set the maximum number of consecutive counts for the same x_center value

# Load your weight
model = YOLO('best5.pt')

# Set up the serial connection to the Arduino board
port = '/dev/ttyUSB0'  # Update with your port
baudrate = 9600  # Update with your baudrate
ser = serial.Serial(port, baudrate)

# Initialize previous x_center
prev_x_center = None

# Run inference on live camera
while True:
    results = model.predict(source="0", save=False, imgsz=640, conf=0.65, show=True, stream=True)
    for r in results:
        boxes = r.boxes
        # Convert the tensor to a list of lists
        bounding_boxes = boxes.xyxy.tolist()
        # Extract x_center from the first bounding box
        if bounding_boxes:
            x_center = (bounding_boxes[0][0] + bounding_boxes[0][2]) / 2
            # Check if x_center is the same as the previous one
            if x_center == prev_x_center:
                consecutive_count += 1
                
            else:
                consecutive_count = 1
                prev_x_center = x_center
            
            # If x_center remains the same for 5 consecutive iterations, send it to Arduino
            if consecutive_count >= max_consecutive_count:
                
                ser.write(f"{int(x_center)}\n".encode())
                consecutive_count = 0  # Reset consecutive count
        else:
            prev_x_center = None  # Reset previous x_center if no bounding box is found

    # Add a small delay to avoid flooding the serial port
    time.sleep(0.1)

# Close the serial connection
ser.close()
