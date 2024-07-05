from ultralytics import YOLO
import serial
import time

# Load your weight
model = YOLO('best.pt')

# Set up the serial connection to the Arduino board
port = '/dev/ttyUSB0'  # Update with your port
baudrate = 9600  # Update with your baudrate
ser = serial.Serial(port, baudrate)

# Run inference on live camera
while True:
    results = model.predict(source="1", save=False, imgsz=640, conf=0.5, show=True, stream=True)
    for r in results:
        boxes = r.boxes
        # Convert the tensor to a list of lists
        bounding_boxes = boxes.xyxy.tolist()
        # Send only the x-coordinates to Arduino
        for bbox in bounding_boxes:
            x_center = bbox[0] + (bbox[2] - bbox[0]) / 2  # Calculate the x-coordinate of the object center
            # Send the x-coordinate to Arduino as bytes
            ser.write(f"{int(x_center)}\n".encode())
            # Add a small delay to avoid flooding the serial port
            time.sleep(0.1)

# Close the serial connection
ser.close()
