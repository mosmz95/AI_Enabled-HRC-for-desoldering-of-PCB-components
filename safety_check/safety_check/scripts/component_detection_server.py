import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ComponentDetection

from ament_index_python.packages import get_package_share_directory
from safety_check.realsense_class import RealSense_Cam, get_realsense_devices
from ultralytics import YOLO
import cv2
import os
from safety_check.realsense_class import RealSense_Cam, get_realsense_devices

from cv_bridge import CvBridge


class ComponentDetectionService(Node):

    def __init__(self,yolo_path):
        super().__init__('component_detection_service')
        self.detect_srv = self.create_service(ComponentDetection, 'component_bounding_box', self.component_boundingbox_callback)
        self.get_logger().info('The server is running.')
        self.imagebridge_request = CvBridge()
        self.imagebridge_response = CvBridge()
        self.yolo_path = yolo_path
        self.yolo_model = YOLO(self.yolo_path, verbose = False)
        self.get_logger().info('The Yolo model has been loaded.')


    def component_boundingbox_callback(self, request, response):
        self.get_logger().info('The request of component detection has been received.')
        rcv_component_id = request.component_id
        self.get_logger().info(f'The component id is:{rcv_component_id}')

        rcv_raw_frame = self.imagebridge_request.imgmsg_to_cv2(request.raw_frame, desired_encoding='bgr8') 
        print(rcv_raw_frame)
        annotated_frame, location, classes_id = self.detect_model(rcv_raw_frame,rcv_component_id)

        print(classes_id)
        response.annotated_frame = self.imagebridge_response.cv2_to_imgmsg( annotated_frame, encoding='bgr8')
        response.component_class = [int(id) for id in classes_id]
        response.location_x = [loc[0] for loc in location]
        response.location_y = [loc[1] for loc in location]

        return response

    def detect_model(self, frame, component_id=[1]):
        result = None
        results = self.yolo_model(frame, classes = component_id, verbose = False)
        annotated_frame = results[0].plot()
        location = [( (float(bbox[0] + bbox[2])/2) , float((bbox[1] + bbox[3])/2) ) for bbox in results[0].boxes.xyxy]
        classes_id = [float(value) for value in results[0].boxes.cls]
        return annotated_frame, location, classes_id
    
    # def detect_model(self, frame, component_id=[1]):
    #     result = None
    #     results =  self.yolo_model(frame, classes = component_id, verbose = False)
    #     # annotated_frame = results[0].plot()
    #     for bbox in results[0].boxes.xyxy :
    #         cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), color=(255, 0, 0), thickness=10) 

    #     location = [( int((bbox[0] + bbox[2])/2) , int((bbox[1] + bbox[3])/2) ) for bbox in results[0].boxes.xyxy]
    #     classes_id = [float(value) for value in results[0].boxes.cls]
    #     font = cv2.FONT_HERSHEY_SIMPLEX

    #     for i in range(len(classes_id)):
    #         cv2.putText(frame, str(i+1), location[i], font, 5, (0, 0, 255), 10, cv2.LINE_AA)
            
    #     return frame, location, classes_id
def main():
    package_share_directory = get_package_share_directory('safety_check')
    yolo_model_path = os.path.join(package_share_directory, 'configs','best.pt')
    rclpy.init()
    print("hiiiiii")
    node = ComponentDetectionService(yolo_model_path)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()