#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import os
import logging

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
parent_parent_dir = os.path.dirname(parent_dir)
folder_path = os.path.join(parent_dir, "configs")



MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green

HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult

class SafetyLayer:

    ''' check the safety by having the operators hand knuck and cobot tool'''

    def __init__(self,Num_hands=2,aruco_dict_type = aruco.DICT_4X4_100,marker_size_in_cm = 5,marker_id =0, task_path = folder_path):
        self.num_hands = Num_hands
        self.BaseOptions = mp.tasks.BaseOptions
        self.HandLandmarker = mp.tasks.vision.HandLandmarker
        self.HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        self.HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
        self.VisionRunningMode = mp.tasks.vision.RunningMode
        self.model_path = os.path.join(task_path, "hand_landmarker.task")
        self.options = self.HandLandmarkerOptions(
            base_options=self.BaseOptions(model_asset_path=self.model_path),
            running_mode=self.VisionRunningMode.LIVE_STREAM,
            num_hands = self.num_hands,
            result_callback=self.print_result)
        self.landmarker = None
        self.counter_mp = 0
        self.result_hand_detection = None
        self.annotated_frame_knuckle = None



        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters()  # Marker detection parameters
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.marker_coordinates = None
        self.marker_center = None
        self.marker_size = marker_size_in_cm
        self.marker_id = marker_id
        self.tool_center = None




    def hand_knuckles_frame(self):
       return self.annotated_frame_knuckle
    
    def hand_knuckles_frame_flip(self):
        return cv2.flip(self.annotated_frame_knuckle,1)
    
    def hand_knuckles_coordinate(self):
       return self.result_hand_detection
    
    def print_result(self,result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        # print('hand landmarker result: {}'.format(result.handedness))
        
        self.annotated_frame_knuckle = self.draw_landmarks_on_image(output_image.numpy_view(), result)
        self.result_hand_detection = result
        
    def create_landmarker(self):
        self.landmarker = self.HandLandmarker.create_from_options(self.options)


    def run_landmarker(self, frame_bgr):
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_rgb_flip = cv2.flip(frame_rgb,1)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb_flip)
        self.counter_mp +=1

        self.landmarker.detect_async(mp_image, self.counter_mp)


    def draw_landmarks_on_image(self,rgb_image, detection_result):
        hand_landmarks_list = detection_result.hand_landmarks
        handedness_list = detection_result.handedness
        annotated_image = np.copy(rgb_image)

        # Loop through the detected hands to visualize.
        for idx in range(len(hand_landmarks_list)):
            hand_landmarks = hand_landmarks_list[idx]
            handedness = handedness_list[idx]

            # Draw the hand landmarks.
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            solutions.hands.HAND_CONNECTIONS,
            solutions.drawing_styles.get_default_hand_landmarks_style(),
            solutions.drawing_styles.get_default_hand_connections_style())

            # Get the top left corner of the detected hand's bounding box.
            height, width, _ = annotated_image.shape
            x_coordinates = [landmark.x for landmark in hand_landmarks]
            y_coordinates = [landmark.y for landmark in hand_landmarks]
            text_x = int(min(x_coordinates) * width)
            text_y = int(min(y_coordinates) * height) - MARGIN

            # Draw handedness (left or right hand) on the image.
            # cv2.putText(annotated_image, f"{handedness[0].category_name}",
            #             (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
            #             FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)

        return annotated_image
    


    def track_marker(self,frame,theta,id_to_find=0):
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        # lists of ids and the corners belonging to each id
        corners, ids, rejected = self.detector.detectMarkers(image=gray)

        
        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                if i == id_to_find:
                    aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                    self.marker_coordinates = corners[0][0]
                    self.marker_coordinates = np.int_(self.marker_coordinates)
                    self.marker_center = [int(np.mean(corners[0], axis=1)[0][0]), int(np.mean(corners[0], axis=1)[0][1])]
                    frame = cv2.circle(frame, (self.marker_center[0], self.marker_center[1]), radius=5, color=(0, 0, 255),
                                    thickness=5)
                    

                    point1 = (self.marker_center[0], self.marker_center[1])  # marker center coordinate in pixel
                    # calculating the desoldering tool location in pixel

                    tool_offset = 120 # tool offset with respect to the marker center in pixel
                    self.tool_center=[]
                    self.tool_center.append(self.marker_center[0]+int(round(tool_offset * np.cos(theta-0.25*np.pi))))
                    self.tool_center.append(self.marker_center[1]+int(round( tool_offset* np.sin(theta-0.25*np.pi))))
                    point2 = ( self.tool_center[0], self.tool_center[1] )  # 
                    logging.debug("The tool point in [pixel] is: %s ", point2)
                    # Draw a line between the two points
                    cv2.line(frame, point1 , point2, (255, 0, 0), 2)  # Draws a green line with thickness 2
                    cv2.circle(frame,center=point2,radius=int(100/1.5336),color=(0, 0, 255),thickness=2)
        else:
            self.marker_coordinates = None
          

        return frame



    def check_safety(self, min_distance_stand = 100/1.5336):#detection_result :HandLandmarkerResult, marker_center=[0,0],min_distance_stand = 20):
        presence = False       

        if self.tool_center is not None:
            tool_center_flip = [0,0]
            tool_center_flip[0] = 640 - self.tool_center[0] -1      
            tool_center_flip[1] = self.tool_center[1]      
        else: 
            return  None
       
        if self.result_hand_detection is not None:

            hand_landmarks_list = self.result_hand_detection.hand_landmarks
            handedness_list = self.result_hand_detection.handedness
        
            # Loop through the detected hands .
            for idx in range(len(hand_landmarks_list)): # check number of hands
                hand_landmarks = hand_landmarks_list[idx]  # get land mark for a one hand
                handedness = handedness_list[idx]
                distance_knuckles_from_tool=[]

                for landmark in hand_landmarks:
                    x_pix = int(landmark.x * 640 )
                    y_pix = int(landmark.y * 480)

                    dis = np.sqrt(np.power((x_pix - tool_center_flip[0]),2) + np.power((y_pix-tool_center_flip[1]),2))
                    distance_knuckles_from_tool.append(dis)
                    # if np.isnan(dis):
                    #     print(x_pix)
                    #     print(y_pix)
                    #     print(tool_center_flip[0])
                    #     print(tool_center_flip[1])

                   
                    
                min_distance_from_tool = np.min(distance_knuckles_from_tool)
                # print(min_distance_from_tool)
                # print(distance_knuckles_from_tool)
                
                min_distance_index = np.argmin(distance_knuckles_from_tool)
                # print(min_distance_index)

                x_pix_min = int(hand_landmarks[min_distance_index].x * 640 )
                y_pix_min = int(hand_landmarks[min_distance_index].y * 480)
                point_min = (x_pix_min,y_pix_min)
                point_tool_flip=(tool_center_flip[0],tool_center_flip[1])
                cv2.line(self.annotated_frame_knuckle, point_tool_flip, point_min, (168, 255, 0), 2)  # Draws a green line with thickness 2
                if min_distance_from_tool < min_distance_stand:
                    cv2.line(self.annotated_frame_knuckle, point_tool_flip, point_min, (0, 0, 255), 2)  # Draws a green line with thickness 2

                    presence = True  
                    # print("danger")
                    break    
        else:
            return None 

        return presence
    



if __name__ == "__main__":

    ss = SafetyLayer()
    cap = cv2.VideoCapture(0)
    ss.create_landmarker()

    while cap.isOpened():
        ret,frame = cap.read()

        if cv2.waitKey(1) & 0xFF == ord('q'):
         break

        ss.run_landmarker(frame_bgr=frame)
    
        
        if ss.hand_knuckles_frame() is not None:
         cv2.imshow("hand_detection",cv2.cvtColor(ss.hand_knuckles_frame(),cv2.COLOR_RGB2BGR))
        else:
         cv2.imshow("hand_detection",frame)
        
        # if ss.hand_knuckles_coordinate() is not None:
        #     ss.check_safety(result_hand_detection)
        # # print(result_hand_detection)

    cap.release()
    cv2.destroyAllWindows()
