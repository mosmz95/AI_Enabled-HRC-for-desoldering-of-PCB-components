import cv2
from tkinter import *
from tkinter import ttk
import numpy as np
import requests
from matplotlib import pyplot as plt
from PIL import ImageTk, Image
import io
import time
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("/home/mostafa/workspace/aiRedgio_ws/src/safety_check/scripts/best.pt", verbose = False)
print(model.names)
class_group = []

while True:
    key = input('Enter the class you want to detect: \n')
    if key.isdigit():
        class_group.append(int(key))
    elif key == 'q':
        break

# Open the video file
# video_path = "path/to/your/video/file.mp4"
# cap = cv2.VideoCapture(0)
result = None
# Loop through the video frames
while True:
    # print('frame reading start')
    resp=None
    try:
        resp = requests.get("http://192.168.0.100:4242/current.jpg?type=color").content
    except:
        pass
    
    #Check the response
    if resp == None:
        break
    else:
        imageData = np.asarray(bytearray(resp), dtype="uint8")
        pilImage=np.array(Image.open(io.BytesIO(imageData)))
        results = model(pilImage, classes = class_group, verbose = False)
        # print("run_until_here")
        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        if result is None:
            # print(annotated_frame.shape)
            size = (int(annotated_frame.shape[1]), int(annotated_frame.shape[0]))
            result = cv2.VideoWriter('filename.avi', 
						cv2.VideoWriter_fourcc(*'MJPG'), 
						5, size)
        result.write(annotated_frame)  
	

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            result.release() 
            break

# Release the video capture object and close the display window
# cap.release()
cv2.destroyAllWindows()
