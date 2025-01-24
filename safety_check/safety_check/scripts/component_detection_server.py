import rclpy
from rclpy.node import Node
from custome_interfaces.srv import ComponentDetection

from ament_index_python.packages import get_package_share_directory
from safety_check.realsense_class import RealSense_Cam, get_realsense_devices
from ultralytics import YOLO
import cv2
import os
from safety_check.realsense_class import RealSense_Cam, get_realsense_devices



class ComponentDetectionService(Node):

    def __init__(self,yolo_path):
        super().__init__('component_detection_service')
        self.detect_srv = self.create_service(ComponentDetection, 'component_bounding_box', self.add_two_ints_callback)
        self.yolo_path = yolo_path
        self.yolo_model = YOLO(self.yolo_path, verbose = False)


    def component_boundingbox_callback(self, request, response):
        self.get_logger().info('Incoming request')
        rcv_component_id = request.component_id
        annotated_frame, location, classes_id = self.detect_model(request.raw_frame,rcv_component_id)
        response.annotated_frame = annotated_frame
        response.component_class = classes_id
        response.location_x = [loc[0] for loc in location]
        response.location_y = [loc[1] for loc in location]

        return response

    def detect_model(self, frame, component_id=[1]):
        result = None
        results = self.model(frame, classes = component_id, verbose = False)
        annotated_frame = results[0].plot()
        location = [( (float(bbox[0] + bbox[2])/2) , float((bbox[1] + bbox[3])/2) ) for bbox in results[0].boxes.xyxy]
        classes_id = [float(value) for value in results[0].boxes.cls]
        return annotated_frame, location, classes_id
    

def main():
    package_share_directory = get_package_share_directory('safety_check')
    yolo_model_path = os.path.join(package_share_directory, 'configs','best.pt')
    rclpy.init()
    node = ComponentDetectionService(yolo_model_path)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()