from ultralytics import YOLO
import serial
import time
import cv2

# Load your weight
model = YOLO('fire.pt')

# Set up the serial connection to the Arduino board
port = '/dev/ttyUSB0'  # Update with your port
baudrate = 9600  # Update with your baudrate
ser = serial.Serial(port, baudrate)

# Run inference on live camera
while True:
    results = model.predict(source="1", save=False, imgsz=640, conf=0.1, show=False, stream=True)
    for r in results:
        img = r.orig_img
        boxes = r.boxes
        # Convert the tensor to a list of lists
        bounding_boxes = boxes.xyxy.tolist()
        
        # Draw line in the middle of each bounding box and send its coordinates to Arduino
        for bbox in bounding_boxes:
            x1, y1, x2, y2, conf = bbox[:4]  # Extract coordinates and confidence
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            line_length = min(int(x2 - x1), int(y2 - y1))
            
            # Draw line
            cv2.line(img, (x_center - line_length // 2, y_center), (x_center + line_length // 2, y_center), (0, 255, 0), 2)
            
            # Send the coordinates of the line to Arduino
            ser.write(f"{x_center},{y_center}\n".encode())
            
        # Display image with lines
        cv2.imshow('Object Detection', img)
        cv2.waitKey(1)

# Close the serial connection
ser.close()
