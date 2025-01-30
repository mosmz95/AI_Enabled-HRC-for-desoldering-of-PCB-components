import cv2
import numpy as np
import requests
from PIL import ImageTk, Image
import io
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool,String,Int16
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge



class WristCamera(Node):
    def __init__(self):
        super().__init__("Wrist_camera_publisher_node")
        self.get_logger().info("The wrist camera publisher node has been started!")
        # self.timer = self.create_timer(0.5, self.timer_callback)  # 10 Hz timer
        self.rawimages_publisher = self.create_publisher(ROSImage , "wristcamera",10)
        self.imagebridge = CvBridge() # for conversion of cv images to ros2 images
        self.rawframe = None
        self.raw_imagebridge = CvBridge() 
        self.run_me()

    def timer_callback(self):
        # self.get_logger().info(" Timer callback!")
        resp=None
        try:
            self.get_logger().error("NNNNNNNNNNNNNN")

            resp = requests.get("http://192.168.0.100:4242/current.jpg?type=color").content
            
        except:
            self.get_logger().error("No response from the port!")
        
        #Check the response
        if resp == None:
            self.get_logger().error("Failed to capture frame from Wrist camera!")
        else:
            imageData = np.asarray(bytearray(resp), dtype="uint8")
            pilImage=np.array(Image.open(io.BytesIO(imageData)))

            rawImage_tobesent = self.raw_imagebridge.cv2_to_imgmsg( pilImage, encoding='bgr8')
            self.rawimages_publisher.publish(rawImage_tobesent)
            
    def run_me(self):

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
                rawImage_tobesent = self.raw_imagebridge.cv2_to_imgmsg( pilImage, encoding='bgr8')
                self.rawimages_publisher.publish(rawImage_tobesent)




def main():
    
    rclpy.init(args=None)
    node = WristCamera()
    rclpy.spin(node)
   
   

if __name__=="__main__":

    main()







