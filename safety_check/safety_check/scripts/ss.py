import cv2
import os
import numpy as np
import time
from ultralytics import YOLO
from matplotlib import pyplot as plt
from PIL import ImageTk, Image
import io
from safety_check.realsense_class import RealSense_Cam, get_realsense_devices
from ament_index_python.packages import get_package_share_directory
package_share_directory = get_package_share_directory('safety_check')
yolo_model_path = os.path.join(package_share_directory, 'configs','best.pt')

# Load the YOLOv8 model
# model = YOLO("./best.pt", verbose = False)
# print(model.names) # this print the model components id equivalent in model

#input is loaded model, frame, and list of componenets you want to detect

def detect_model(model, frame, component_id=[1]):
    result = None
    results = model(frame, classes = component_id, verbose = False)
    annotated_frame = results[0].plot()
    location = [( (float(bbox[0] + bbox[2])/2) , float((bbox[1] + bbox[3])/2) ) for bbox in results[0].boxes.xyxy]
    classes_id = [float(value) for value in results[0].boxes.cls]
    return annotated_frame, location, classes_id

# annotated_frame, location of comoponent, class_id of detected components


list_of_devices = get_realsense_devices()
for index, device in  enumerate(list_of_devices):
    
    cam = RealSense_Cam(device["serial_number"])
    pipeline, config = cam.start_real_sense()
    # Start streaming
    pipeline.start(config)


# Load the YOLOv8 model
model = YOLO(yolo_model_path, verbose = False)
print(model.names)
class_group = []
result = None

# while True:
#     key = input('Enter the class you want to detect: \n')
#     if key.isdigit():
#         class_group.append(int(key))
#     elif key == 'q':
#         break
while True:

    depth, frame = cam.get_frame_from_realsense(pipeline,aligned_frame=False)

    # imageData = np.asarray(bytearray(resp), dtype="uint8")
    # pilImage=np.array(Image.open(io.BytesIO(imageData)))
    # results = model(frame, classes = class_group, verbose = False)
    # print("hhh")
    # print("run_until_here")
    # Visualize the results on the frame
    # annotated_frame = results[0].plot()
    annotated_frame, location, classes_id =   detect_model(model, frame, component_id=[1])
    print(location)
    print(classes_id)
    print("=======================")
    cv2.imshow('Align Example', annotated_frame)
    # if result is None:
    #     # print(annotated_frame.shape)
    #     size = (int(annotated_frame.shape[1]), int(annotated_frame.shape[0]))
    #     result = cv2.VideoWriter('filename.avi', 
    #                 cv2.VideoWriter_fourcc(*'MJPG'), 
    #                 5, size)
    # result.write(annotated_frame)  
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break
        
        


pipeline.stop()


{0: 'Capacitor', 1: 'IC', 2: 'LED', 3: 'Resistor', 4: 'battery', 5: 'buzzer', 6: 'clock', 7: 'connector', 8: 'diode', 9: 'display', 10: 'fuse', 11: 'inductor', 12: 'potentiometer', 13: 'relay', 14: 'switch', 15: 'transistor'}