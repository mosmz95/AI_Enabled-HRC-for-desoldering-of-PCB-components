#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,String,Int16
from custome_interfaces.msg import Safetycheck
from cv_bridge import CvBridge

import sys
import os
sys.path.append('...')
from sensor_msgs.msg import Image as ROSImage
import tkinter as tk
from PIL import ImageTk, Image
from std_msgs.msg import Bool,String,Int16
# from robot_custom_msgs.msg import gui_msg
from functools import partial
import threading
import cv2
import copy
from ament_index_python.packages import get_package_share_directory


os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

# script_dir = os.path.dirname(os.path.abspath(__file__))
# parent_dir = os.path.dirname(script_dir)
package_share_directory = get_package_share_directory('safety_check')
folder_path = os.path.join(package_share_directory, "configs")

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def image_resize(img, scale):
    image = Image.fromarray(img)
    new_width, new_height = int(image.width * scale), int(image.height * scale)
    img_res = image.resize((new_width, new_height), Image.LANCZOS)
    return img_res

class LEDIndicator(tk.Canvas):
    def __init__(self, master=None, **kwargs):
        super().__init__(master, **kwargs)
        self.size = 30
        self.create_oval(5, 5, self.size, self.size, outline='black', fill='blue')  # LED body
        self.color = 'grey'

    def set_color(self, color):
        self.color = color
        self.itemconfig(1, fill=self.color)  # Change LED body color to the specified color

class MyGUI(tk.Tk):

    def __init__(self,img,lablogo,polimi_logo,num_leds, rosnode):
        super().__init__()

        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        self.title("PCB desoldering live update")
        self.geometry(f"{screen_width}x{screen_height}+0+0")
        
       
        self.num_leds = num_leds
        # Create widgets
        self.label = tk.Label(self, text="ROS GUI")
        self.label.pack(pady=10)

        # indicate pcb's photo
        self.photo = ImageTk.PhotoImage(img)
        self.photo_gui = tk.Label(self, image=None)
        self.photo_gui.place(relx=0.65, rely=0.05, anchor='nw')
        self.bbxtitle = tk.Label(self, text="Detected Components", font=("Helvetica", 18, "bold"))
        self.bbxtitle.place(relx=0.73, rely=0.01, anchor='nw')
        # lab logos
        self.lablogo = ImageTk.PhotoImage(lablogo)
        self.lablogo_gui = tk.Label(self, image=self.lablogo)
        self.lablogo_gui.place(relx=0.45, rely=0.35, anchor='nw')
        ## polimi_logo

        self.polimilogo = ImageTk.PhotoImage(polimi_logo)
        self.polimilogo_gui = tk.Label(self, image=self.polimilogo)
        self.polimilogo_gui.place(relx=0.45, rely=0.05, anchor='nw')
        ### frame from camera
        self.comingfromcamera = tk.Label(self)
        self.comingfromcamera.place(relx=0.02, rely=0.05, anchor='nw')
        self.cameratitle = tk.Label(self, text="Camera's frame", font=("Helvetica", 18, "bold"))
        self.cameratitle.place(relx=0.15, rely=0.01, anchor='nw')

        self.text_frame = tk.Frame(self,width=100, height=50)
        
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

      
       

        self.leds = {} # store led objects
        self.reset_buttons={}

        self.recovery_from_safety_flag = True
        # self.create_leds()
        self.rosnode = rosnode
        self.gui_update_interval = 1 #mss


    def insert_text_and_scroll(self, text):
        self.text_field.insert(tk.END, text)
        self.scroll_to_end()

    def scroll_to_end(self, event=None):
        self.text_field.see(tk.END)

    def create_leds(self, spacing=0.06):
        # Calculate initial position for the first LED and label
        init_pos_x = 0.8
        init_pos_y = 0.1

        for i in range(self.num_leds):
            # Create LED
            led = LEDIndicator(self, width=30, height=30)
            led.place(relx=init_pos_x, rely=init_pos_y, anchor='e')

            #self.leds.append(led)
            self.leds[f"led{i+1}"]=led
            # Create label
            label = tk.Label(self, text=f"Component {i+1}")
            label.place(relx=init_pos_x - 0.1, rely=init_pos_y, anchor='center')

            # Create buttons
            self.reset_buttons[f"led{i+1}"] =  tk.Button(self,text="Reset Button",command=partial(self.reset_button_callback,f"{i+1}"))
            self.reset_buttons[f"led{i+1}"].place(relx=init_pos_x + 0.1, rely=init_pos_y, anchor='center')
            
            # Update initial position for next LED and label
            init_pos_y += spacing
        
   
   

    def reset_button_callback(self,key):
        # it should change the color of led to green corresponding in appropriate heating 
        self.leds[f"led{key}"].set_color("yellow")
        msg = String()
        # msg.data = f"led {key}"
        msg.data = f"component_{key}"
        self.rosnode.reset_button_pub.publish(msg)
        self.rosnode.get_logger().info(f"Heating process of component {key} is not yet completed.")
        self.insert_text_and_scroll(f" Heating process of component {key} is not yet completed.\n\n")  # Insert the new text
        # print("mosi")
        # print(type(key))
       

    def pcb_status_indicator(self,lednumber,color="red")-> None:

        self.leds[f"led{lednumber}"].set_color(color)



   
    def update_incoming_frame(self,camera_frame):
        
        camera_frame_=cv2.cvtColor(camera_frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(camera_frame_)
        new_width, new_height = int(image.width * 1.2), int(image.height * 1.2)
        image_resized = image.resize((new_width, new_height), Image.LANCZOS)

        image_tk = ImageTk.PhotoImage(image_resized)
       # self.cameraframe = tk.Label(self, image=image_tk)
        self.comingfromcamera.imgtk = image_tk
        self.comingfromcamera.configure(image=image_tk)
        #self.cameraframe.place(relx=0.6, rely=0.4, anchor='nw')
    
    def gui_periodic_update(self):
        # print("HIIII")
        if self.rosnode.incomming_frame is not None and self.rosnode.incomming_frame_flag:
            self.update_incoming_frame(self.rosnode.incomming_frame)
            self.rosnode.incomming_frame_flag = False
            # print("in sd")
        # rclpy.spin_once(self.rosnode, timeout_sec=0.1)
        self.after(self.gui_update_interval, self.gui_periodic_update)

    def update_bbx_frame(self,bbx_frame):
        
        bbx_frame_=cv2.cvtColor(bbx_frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(bbx_frame_)
        new_width, new_height = int(image.width * 0.8), int(image.height * 0.8)
        image_resized = image.resize((new_width, new_height), Image.LANCZOS)

        image_tk = ImageTk.PhotoImage(image_resized)
        # self.cameraframe = tk.Label(self, image=image_tk)
        self.photo_gui.imgtk = image_tk
        self.photo_gui.configure(image=image_tk)
        #self.cameraframe.place(relx=0.6, rely=0.4, anchor='nw')

    def gui_bbx_periodic_update(self):
        # print("HIIII")
        if self.rosnode.bbx_frame is not None and self.rosnode.bbx_frame_flag:
            self.update_bbx_frame(self.rosnode.bbx_frame)
            self.rosnode.bbx_frame_flag = False
            # print("in sd")
        # rclpy.spin_once(self.rosnode, timeout_sec=0.1)
        self.after(self.gui_update_interval, self.gui_bbx_periodic_update)

    def run_gui(self):
        self.gui_periodic_update()
        self.gui_bbx_periodic_update()
        self.mainloop()
       



class GUIROS(Node):
    def __init__(self):
        super().__init__("gui_safety_monitoring")

        self.reset_button_pub = self.create_publisher(String, "heating_status_of_component",1)
        self.sub_knuckle_image = self.create_subscription(Safetycheck,'knuckle_image',self.sub_comingframe_cb,10)
        self.boundingboximages_sub = self.create_subscription(ROSImage, "bounding_box_image",self.sub_bbxframe_cb,10)

        self.get_logger().info("Gui node has been started.")
        self.imagebridge = CvBridge() # for conversion of cv images to ros2 images
        self.handpresence_ = None
        self.incomming_frame = None
        self.incomming_frame_flag = False
        self.bbx_frame = None
        self.bbx_frame_flag = False
        self.bbx_imagebridge = CvBridge()

    def sub_comingframe_cb(self,msg:Safetycheck):
        # print("recieving images")

        self.incomming_frame = self.imagebridge.imgmsg_to_cv2(msg.annotated_image, desired_encoding='bgr8') 
        # print(msg.annotated_image)
        self.handpresense_ = msg.hand_presence.data# for conversion of cv images to ros2 images
        self.incomming_frame_flag = True
    def sub_bbxframe_cb(self,msg:ROSImage):
        self.get_logger().info("Bounding box frame has been received.")
        self.bbx_frame = self.bbx_imagebridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 
        # print(msg.annotated_image)
        self.bbx_frame_flag = True

        

def ros_spin(rosnode, mygui):
    while rclpy.ok():
        rclpy.spin_once(rosnode, timeout_sec=0.1)
        # Update the GUI with the latest frame if available
        # if rosnode.incomming_frame is not None:
        #     mygui.update_incoming_frame(rosnode.incomming_frame)



def main():

    rclpy.init(args=None)
    rosnode = GUIROS()


    image = Image.open(os.path.join(folder_path, "Pcb_photo.jpg"))
    image_logo = Image.open(os.path.join(folder_path, "lab_logo.png"))
    image_polimilogo = Image.open(os.path.join(folder_path, "Polimi_logo.png"))

    image_logo = image_logo.resize((int(490*0.67), 95), Image.Resampling.NEAREST)
    image_polimilogo = image_polimilogo.resize((int(664*0.5), int(488*0.5)), Image.Resampling.NEAREST)

    mygui = MyGUI(image,lablogo=image_logo,polimi_logo = image_polimilogo ,num_leds=7,
                                  rosnode=rosnode)

   
    ros_thread = threading.Thread(target=ros_spin, args=(rosnode, mygui))
    ros_thread.daemon = True
    ros_thread.start()

    print("dd")
    try:
        mygui.run_gui()
    except KeyboardInterrupt:
        pass

    # Cleanup
    rclpy.shutdown()
    ros_thread.join()
if __name__=="__main__":
    main()
   
   