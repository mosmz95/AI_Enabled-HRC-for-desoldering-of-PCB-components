import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,UInt8
import tkinter as tk
from cv_bridge import CvBridge
import cv2
from PIL import Image, ImageTk
from functools import partial
from custome_interfaces.srv import ComponentDetection
import threading
from sensor_msgs.msg import Image as ROSImage
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

        self.rawframe_subscription  = self.create_subscription( ROSImage,"rawframe_topcamera", self.callback_from_raw_frame,10 )
        self.component_detection_client = self.create_client(ComponentDetection,'component_bounding_box')
        self.rawframe_subscription = None
        self.raw_frame = None # frame comming from camera
        self.imagebridge = CvBridge()
        self.component_id = None
        self.bbxframe = None

        while not self.component_detection_client.wait_for_service(1):
            self.get_logger().warn("Waiting for server of component detection ...")

    
        
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
            self.bbxframe = frame_with_bbx
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
    def __init__(self, ros_node):
        self.ros_node = ros_node

        # Initialize tkinter
        self.root = tk.Tk()
        self.root.title("Component Selector")

        # Set up GUI layout
        self.create_gui()


    def create_gui(self):
        # Create a frame to hold the components
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.BOTH, expand=True)
      
        # Load and display component images with buttons
        for idx, component_name in COMPONENTS.items():
            # Load image
            image_path = f'/home/mostafa/workspace/aiRedgio_ws/src/safety_check/configs/{component_name.lower()}.jpg'  # Ensure the images are named properly
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
        # self.ros_node.start_subscription() # subscribe to the image topic
       

        self.ros_node.component_id  = component_id

    def update_frame(self, frame_with_bbx):
        # Convert OpenCV image to PIL format
        frame_rgb = cv2.cvtColor(frame_with_bbx, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        img = Image.fromarray(frame_rgb)
        imgtk = ImageTk.PhotoImage(image=img)

        # Update the frame_label with the new image
        self.frame_label.imgtk = imgtk  # Keep a reference to avoid garbage collection
        self.frame_label.configure(image=imgtk)

        


    def run_gui(self):
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
    mygui = ComponentGUI(rosnode)
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
