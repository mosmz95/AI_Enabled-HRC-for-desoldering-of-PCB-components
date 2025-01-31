#!/usr/bin/env python3
import sys
import os
sys.path.append('...')
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool,String,Int16
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge


# from robot_custom_msgs.msg import gui_msg
import cv2

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from safety_check.realsense_class import RealSense_Cam, get_realsense_devices
from safety_check.hand_landmark_class import SafetyLayer

from custom_interfaces.msg import Safetycheck

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
  
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
os.environ["QT_QPA_PLATFORM"] = "xcb"

# script_dir = os.path.dirname(os.path.abspath(__file__))
# parent_dir = os.path.dirname(script_dir)
# parent_parent_dir = os.path.dirname(parent_dir)

# folder_path = os.path.join(parent_parent_dir, "configs")
package_share_directory = get_package_share_directory('safety_check')
landmark_task_path = os.path.join(package_share_directory, 'configs')
# print(package_share_directory)
config_filename = os.path.join(package_share_directory, 'configs', 'control_loop_configuration.xml')
print(config_filename)
ROBOT_HOST = "192.168.0.100"
ROBOT_PORT = 30004
# config_filename = os.path.join(folder_path, "control_loop_configuration.xml")


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def image_resize(img, scale):
    image = Image.fromarray(img)
    new_width, new_height = int(image.width * scale), int(image.height * scale)
    img_res = image.resize((new_width, new_height), Image.LANCZOS)
    return img_res


class GUIROS(Node):
    def __init__(self):
        super().__init__("hand_landmark_monitoring")

        self.pub_handpresence_ = self.create_publisher(Bool,'/handpresence/flag', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz timer
        self.annotated_images_publisher = self.create_publisher(Safetycheck , "knuckle_image",10)
       
        self.rawimages_publisher = self.create_publisher(Image , "rawframe_topcamera",10)

        self.imagebridge = CvBridge() # for conversion of cv images to ros2 images
        self.rawframe = None
        self.raw_imagebridge = CvBridge() 

    def timer_callback(self):
        msg_tobe_sent = Safetycheck()
        depth, frame = cam.get_frame_from_realsense(pipeline,aligned_frame=False)
        if frame is None:
            self.get_logger().error("Failed to capture frame from RealSense!")
            return
        else:    

            self.rawframe = frame
            rawImage_tobesent = self.raw_imagebridge.cv2_to_imgmsg( self.rawframe, encoding='bgr8')
            self.rawimages_publisher.publish(rawImage_tobesent)

            state = con.receive()
            frame = safty_lr.track_marker(frame=frame,theta = state.target_q[5])

            safty_lr.run_landmarker(frame_bgr=frame)

            sf_output = safty_lr.check_safety()

            if sf_output is not None:
                hand_presence = sf_output
                if hand_presence:
                    print("hand is detected bitch")

                msg_tobe_sent.hand_presence.data = hand_presence

            if safty_lr.hand_knuckles_frame() is not None:
                final_image = cv2.cvtColor(safty_lr.hand_knuckles_frame_flip(),cv2.COLOR_RGB2BGR)
                # print("landmark")
            else:
                final_image = frame
                # print("aruco")
                msg_tobe_sent.hand_presence.data = False

            
            
            msg_tobe_sent.annotated_image =  self.imagebridge.cv2_to_imgmsg(final_image, encoding='bgr8')
            
            self.annotated_images_publisher.publish(msg_tobe_sent)
            
            
            # cv2.imshow('Align Example', final_image)
            # key = cv2.waitKey(1)
            # if key & 0xFF == ord('q'):
            #     cv2.destroyAllWindows()
            #     rclpy.shutdown()
            #     exit()
        



conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

watchdog.input_int_register_0 = 0
# start data synchronization
if not con.send_start():
    sys.exit()


list_of_devices = get_realsense_devices()
for index, device in  enumerate(list_of_devices):
    
    cam = RealSense_Cam(device["serial_number"])
    pipeline, config = cam.start_real_sense()
    # Start streaming
    pipeline.start(config)

safty_lr = SafetyLayer(task_path = landmark_task_path )

# Start streaming
depth_scale, depth_intrin = cam.depth_information(pipeline)
safty_lr.create_landmarker()

def main():
    
    rclpy.init(args=None)
    node = GUIROS()
    rclpy.spin(node)
   
    con.disconnect()
    pipeline.stop()

if __name__=="__main__":

    main()