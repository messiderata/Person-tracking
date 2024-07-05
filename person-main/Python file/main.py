from ultralytics import YOLO
from PIL import Image
import cv2

# Load your weight
model = YOLO('best5.pt')

# Run inference on 'image or video' with arguments adjust confidence threshold accordingly
#results= model.predict('infer.mp4', save=True, imgsz=720, conf=0.8, show=True)
# for r in results:
#    boxes=r.boxes
#    print (boxes.xyxy)
# Run inference on live camera

results = model.predict (source="1",save=False, imgsz = 640, conf=0.6, show=True,stream=True, iou =0)
for r in results:
    boxes=r.boxes
    print (boxes.xyxy) # prints a tensor of 1x4 which means [[x_coord],[y_coord],[width],[height]]