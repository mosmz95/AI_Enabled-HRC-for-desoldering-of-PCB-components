import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,UInt8,String
import tkinter as tk
from cv_bridge import CvBridge
import cv2
import os
from PIL import Image, ImageTk
from functools import partial
from custom_interfaces.srv import ComponentDetection
from custom_interfaces.msg import Componentdata, Llmfeedback

import threading
from sensor_msgs.msg import Image as ROSImage
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('safety_check')
folder_path = os.path.join(package_share_directory, "configs")

# Component dictionary

COMPONENTS = {
    0: 'Capacitor', 1: 'IC', 2: 'LED', 3: 'Resistor', 4: 'battery', 5: 'buzzer',
    6: 'clock', 7: 'connector', 8: 'diode', 9: 'display', 10: 'fuse', 11: 'inductor',
    12: 'potentiometer', 13: 'relay', 14: 'switch', 15: 'transistor'
}

# ROS 2 Node
class ComponentPublisherNode(Node):

    def __init__(self):

        super().__init__('component_publisher_node')
        self.get_logger().info("The component detection client node has been started.")
        self.publisher = self.create_publisher(UInt8, 'component_id', 10)
        self.boundingboximages_publisher = self.create_publisher(ROSImage, "bounding_box_image",10)

        # self.rawframe_subscription  = self.create_subscription( ROSImage,"rawframe_topcamera", self.callback_from_raw_frame,10 )
        self.component_info_from_llm_subscription  = self.create_subscription( Llmfeedback,"com_id", self.callback_info_from_llm,1 )

        self.recording_command_to_vosk_publisher = self.create_publisher(String, '/system_need', 1  )
        self.detected_componentdata_publisher = self.create_publisher(Componentdata, '/detected_component_data', 1  )
        self.wristcamera_subscription  = self.create_subscription( ROSImage,"wristcamera", self.callback_wristcamera_frame,10 )

        self.go_to_snapshot_position_publisher = self.create_publisher(String, '/go_snapshot_position', 1  )

        self.fromvosk_subscription  = self.create_subscription( String,"received_command", self.callback_fromvosk,10 )

        
        self.component_detection_client = self.create_client(ComponentDetection,'component_bounding_box')
        self.rawframe_subscription = None
        self.raw_frame = None # frame comming from camera
        self.imagebridge = CvBridge()
        self.component_id = None
        self.bbx_frame = None
        self.bbx_imagebridge = CvBridge()
        self.bbx_frame_flag = False

        self.wrist_camera_image = None

        self.displayedtxt = None

        while not self.component_detection_client.wait_for_service(1):
            self.get_logger().warn("Waiting for server of component detection ...")

    def callback_fromvosk(self,msg:String):
        self.displayedtxt = msg.data
        print(self.displayedtxt)


    def callback_info_from_llm(self,msg:Llmfeedback):
        self.get_logger().warn("The info from llm has been received ...")

        component_or_class = msg.type

        if component_or_class == "ComponentClass" :
            self.get_logger().warn("The class of component will be determined for bounding box ...")
            component_class = msg.number_id
            if component_class >= 0 or component_class < 16:
                self.get_logger().info(f"The class of {COMPONENTS[component_class]} has been determined for bounding box ...")
                if self.wrist_camera_image is not None:
                    self.send_detection_request(self.wrist_camera_image, component_class )
                    self.get_logger().info(" The request of component detection has been sent")
            else:
                self.get_logger().warn("The class of component has been chosen wrongly..")

        elif component_or_class == "ComponentCounter" :
            self.get_logger().warn("The counter of component has been specified for desoldering ...")
        else:
            self.get_logger().warn("N valied info from llm has been received...")
        return
    
    def callback_wristcamera_frame(self, msg:ROSImage):
        # self.get_logger().info(" A  wrist camera frame has been received.")
        self.wrist_camera_image = msg
        if self.component_id is not None:
            self.send_detection_request(self.wrist_camera_image, self.component_id )
            self.get_logger().info(" The request of component detection has been sent")
            self.component_id = None
        
    
    def callback_from_raw_frame(self, msg:ROSImage):
        # self.get_logger().info(f" A raw frame has been received.")
        
        self.raw_frame = msg #self.imagebridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 

        if self.component_id is not None:
            self.send_detection_request(self.raw_frame, self.component_id )
            self.get_logger().info(" The request of component detection has been sent")
            self.component_id = None
            self.clean_raw_frame()

    def clean_raw_frame(self):
        self.get_logger().info("Raw frame is change to None")
        self.raw_frame = None


    def send_detection_request(self, img:ROSImage, component_id:UInt8)-> None:
           
            request = ComponentDetection.Request()
            request.raw_frame = img
            request.component_id = component_id

            future_ = self.component_detection_client.call_async(request)
            future_.add_done_callback(partial(self.callback_done_callback,component_id=component_id))
        
    def callback_done_callback(self,future_,component_id):
        response_ = future_.result()
        self.get_logger().info(" The future object has been received.")
        try:
            response_ = future_.result()
            self.get_logger().info(" The future object has been received.")
            frame_with_bbx = response_.annotated_frame # frame with bouding box
            location_x  = response_.location_x#x coordinate of center of component in pixel
            location_y = response_.location_y #x coordinate of center of component in pixel
            component_class = response_.component_class#x coordinate of cen
            self.boundingboximages_publisher.publish(frame_with_bbx)
            self.bbx_frame =  self.bbx_imagebridge.imgmsg_to_cv2(frame_with_bbx, desired_encoding='bgr8') 


            self.bbx_frame_flag = True
            cmp_data = Componentdata()
            cmp_data.location_x = location_x
            cmp_data.location_y = location_y
            cmp_data.component_class = component_class

            self.detected_componentdata_publisher.publish(cmp_data)
            print(type(location_x))
            self.get_logger().info("Bounding box frame received:")
            self.get_logger().info(f"Component Class IDs: {list(component_class)}")
            self.get_logger().info(f"X Coordinates in pixel: {list(location_x)}")
            self.get_logger().info(f"Y Coordinates in pixel: {list(location_y)}")
            for idx, cls_id in enumerate(component_class):
                self.get_logger().info(f"Component {idx + 1}: Class ID={cls_id}, X_pix={location_x[idx]}, Y_pix={location_y[idx]}")

            # self.get_logger().info(" x pixel " + str(type(location_x)))
        except Exception as e:
            self.get_logger().error("Component detection Service call failed %r"%(e,))


    def publish_component_id(self, component_id):
        msg = UInt8()
        msg.data = component_id
        self.publisher.publish(msg)
        self.get_logger().info(f"Published component ID: {component_id}")

# GUI Application
class ComponentGUI:
    def __init__(self, ros_node,file_path):
        self.ros_node = ros_node
        self.file_path = file_path
        # Initialize tkinter
        self.root = tk.Tk()
        self.root.title("Component Selector")

        # Set up GUI layout
        self.create_gui()

        self.frame_label = tk.Label(self.root, image=None)
        self.frame_label.place(relx=0.6, rely=0.05, anchor='nw')

    

        self.voskicon = ImageTk.PhotoImage(self.photo_resize(os.path.join(self.file_path,"vosk_icon.png")))#
        vosk_activation_button = tk.Button(self.root, text="   VOSK Activator   ", image=self.voskicon, compound=tk.TOP,
                        command=self.vosk_activation_button_cb, font=("Helvetica", 12))
        vosk_activation_button.place(relx=0.4, rely=0.01, anchor='nw')

        self.snapphoto = ImageTk.PhotoImage(self.photo_resize(os.path.join(self.file_path,"snapshot.jpg")))#
        snapshotposition_button = tk.Button(self.root, text="Go to Snap shot position", image= self.snapphoto, compound=tk.TOP,
                        command=self.snapshotposition_button_cb, font=("Helvetica", 12))
        snapshotposition_button.place(relx=0.4, rely=0.2, anchor='nw')

        self.text_frame = tk.Frame(self.root,width=100, height=50)
        
        # self.text_frame.pack(fill=tk.BOTH, expand=True,padx=20, pady=20)  # Fill the entire window and expand
        self.text_frame.place(relx=0.6, rely=0.6, anchor='nw')
        # Create a Scrollbar widget
        self.scrollbar = tk.Scrollbar(self.text_frame, orient=tk.VERTICAL)

        # Create a Text widget to display and edit the text message
        self.text_field = tk.Text(self.text_frame, wrap=tk.NONE, yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.text_field.yview)

        # # Pack the Text widget and Scrollbar widget inside the Frame
        self.text_field.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def insert_text_and_scroll(self, text):
        self.text_field.insert(tk.END, text)
        self.scroll_to_end()

    def scroll_to_end(self, event=None):
        self.text_field.see(tk.END)


    def snapshotposition_button_cb(self):
        self.ros_node.get_logger().info(f"The cobot is going to snap shot position")
        self.insert_text_and_scroll("The cobot is going to snap shot position\n")
        msg = String()
        msg.data = "snapshot"
        self.ros_node.go_to_snapshot_position_publisher.publish(msg)


    def vosk_activation_button_cb(self):
        self.ros_node.get_logger().info(f"The vosk is listenning to you")
        self.insert_text_and_scroll("The vosk is listenning to you\n")
        msg_to_be_sent_vosk = String()
        msg_to_be_sent_vosk.data = "Activate the VOSK"
        self.ros_node.recording_command_to_vosk_publisher.publish(msg_to_be_sent_vosk)
        return
    
    def photo_resize(self,image_path):
        image = Image.open(image_path)
        image = image.resize((100, 100), Image.Resampling.LANCZOS)
        # photo = ImageTk.PhotoImage(image)
        return image

    def create_gui(self):
        # Create a frame to hold the components
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.BOTH, expand=True)

        # Load and display component images with buttons
        for idx, component_name in COMPONENTS.items():
            # Load image
            # image_path_= f'/home/mostafa/workspace/aiRedgio_ws/src/safety_check/configs/{component_name.lower()}.jpg'  # Ensure the images are named properly
            image_path = os.path.join(self.file_path,f'{component_name.lower()}.jpg')  # Ensure the images are named properly

            try:
                image = Image.open(image_path)
                image = image.resize((100, 100), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(image)
            except FileNotFoundError:
                # Placeholder if image is missing
                photo = tk.PhotoImage(width=100, height=100)

            # Create button
            button = tk.Button(
                frame, image=photo, text=component_name, compound=tk.TOP,
                command=lambda idx=idx: self.on_component_click(idx)
            )
            button.image = photo  # Keep a reference to avoid garbage collection
            button.grid(row=idx // 4, column=idx % 4, padx=10, pady=10)


    def on_component_click(self, component_id):
        # Callback when a component is clicked
        self.ros_node.publish_component_id(component_id)
        print(f"Button clicked for component ID: {component_id}")
        self.insert_text_and_scroll(f"Button clicked for component ID: {component_id}\n")
        # self.ros_node.start_subscription() # subscribe to the image topic
       

        self.ros_node.component_id  = component_id

    def update_bbx_frame(self, frame_with_bbx):
        # Convert OpenCV image to PIL format
        frame_rgb = cv2.cvtColor(frame_with_bbx, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        img = Image.fromarray(frame_rgb)
        imgtk = ImageTk.PhotoImage(image=img)

        # Update the frame_label with the new image
        self.frame_label.imgtk = imgtk  # Keep a reference to avoid garbage collection
        self.frame_label.configure(image=imgtk)

    def gui_bbx_periodic_update(self):
        # print("HIIII")
        if self.ros_node.bbx_frame is not None and self.ros_node.bbx_frame_flag:
            self.update_bbx_frame(self.ros_node.bbx_frame)
            self.ros_node.bbx_frame_flag = False
            # print("in sd")
        # rclpy.spin_once(self.rosnode, timeout_sec=0.1)
        self.root.after(100, self.gui_bbx_periodic_update)

    def displaytexttogui(self):
        if self.ros_node.displayedtxt is not None:
            self.insert_text_and_scroll(self.ros_node.displayedtxt + "\n")
            self.ros_node.displayedtxt = None
        self.root.after(100, self.displaytexttogui)   

    def run_gui(self):

        self.gui_bbx_periodic_update()
        self.displaytexttogui()
        self.root.mainloop()



def ros_spin(rosnode):
    # print("Starting ros_spin loop")
    # print("rclpy.ok() status:", rclpy.ok())
    while rclpy.ok():
        # print("Before spin_once")
        rclpy.spin_once(rosnode, timeout_sec=0.1)
    #     print("After spin_once")
    # print("Exiting ros_spin...")

# Main function
def main():

    rclpy.init(args=None)
    rosnode = ComponentPublisherNode()
    mygui = ComponentGUI(rosnode,folder_path)
    ros_thread = threading.Thread(target=ros_spin, args=(rosnode,))
    ros_thread.daemon = True
    ros_thread.start()

    try:
        mygui.run_gui()
    except KeyboardInterrupt:
        pass

    # Cleanup
    rclpy.shutdown()
    ros_thread.join()
   

if __name__ == '__main__':
    main()