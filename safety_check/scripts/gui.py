#!/usr/bin/env python3
import sys
import os
sys.path.append("...")
import rospy
import tkinter as tk
from PIL import ImageTk, Image
from std_msgs.msg import Bool,String,Int16
# from robot_custom_msgs.msg import gui_msg
from functools import partial
import threading
import cv2
import copy

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

from desoldering_pcb import safety_class, realsense_class
from desoldering_pcb.realsense_class import RealSense_Cam
from desoldering_pcb.safety_class import SafetyLayer
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

    def __init__(self,img,lablogo,polimi_logo,num_leds,pipeline):
        super().__init__()

        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        self.title("PCB desoldering live update")
        self.geometry(f"{screen_width}x{screen_height}+0+0")
        
       
        self.num_leds = num_leds
        self.pipeline=pipeline
        # Create widgets
        self.label = tk.Label(self, text="ROS GUI")
        self.label.pack(pady=10)

        # indicate pcb's photo
        self.photo = ImageTk.PhotoImage(img)
        self.photo_gui = tk.Label(self, image=self.photo)
        self.photo_gui.place(relx=0.02, rely=0.65, anchor='nw')
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
        self.reset_button_pub = rospy.Publisher("/guiresetbutton",String,queue_size=1)
        self.listener = rospy.Subscriber('/handdetection/flag', Bool, self.sub_callback, queue_size=1)

        self.recovery_from_safety_flag = True
        self.create_leds()


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
        msg.data = f"led {key}"
        self.reset_button_pub.publish(msg)
        rospy.loginfo(f"Heating process of component {key} is not yet completed.")
        self.insert_text_and_scroll(f" Heating process of component {key} is not yet completed.\n\n")  # Insert the new text
        print("mosi")
        # custom_mssage = gui_msg()
        print(type(key))
        # custom_mssage.component_number = int(key)
        # custom_mssage.heating_success = False
        # pub_component.publish(custom_mssage)

    def pcb_status_indicator(self,lednumber,color="red")-> None:

        self.leds[f"led{lednumber}"].set_color(color)


    def sub_callback(self,data):
        handpresense = data.data
        if handpresense==True:
            rospy.loginfo(" ********************** Safety Action is Activated ********************.")
            mygui.pcb_status_indicator(lednumber=1)

   
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


        self.after(1, self.update_frame)
    
    def update_frame(self):
        depth, frame = cam.get_frame_from_realsense(pipeline)
        

        # print("mosi")
        if frame is not None:

            state = con.receive()
            frame = safty_lr.track_marker(frame=frame,theta = state.target_q[5])

            safty_lr.run_landmarker(frame_bgr=frame)
            

            sf_output = safty_lr.check_safety()
            if sf_output is not None:
                hand_presence = sf_output
               
            # robot angle
            

            # print(state.target_q[5])

            # Hand detection
                if hand_presence and self.recovery_from_safety_flag:
                    pub_handpresence.publish(True)
                    rospy.loginfo("Hand is in the dangerous Zone!!!!!!!")
                    self.insert_text_and_scroll("Hand is in the dangerous Zone!!!!!!!\n\n")  # Insert the new text
                    self.recovery_from_safety_flag = False

            if safty_lr.hand_knuckles_frame() is not None:
                final_image = cv2.cvtColor(safty_lr.hand_knuckles_frame_flip(),cv2.COLOR_RGB2BGR)
            else:
                final_image = frame
            self.update_incoming_frame(final_image)



######################## call back

def sub_message_cb(msg:String):
    txt = msg.data
    global mygui
    mygui.insert_text_and_scroll(f"{txt}\n\n") 


def sub_recover_from_safety_cb(msg:Bool):

    mygui.recovery_from_safety_flag = True
    
   

if __name__=="__main__":
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    folder_path = os.path.join(parent_dir, "configs")
    ##################################################### UR5e connection ###########################

    ROBOT_HOST = "192.168.0.100"
    ROBOT_PORT = 30004
    config_filename = os.path.join(folder_path, "control_loop_configuration.xml")




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


######################################################



    rospy.init_node("ros_gui_node", anonymous=False)
    # pub_component = rospy.Publisher('/component_heating/success', gui_msg, queue_size=1)
    pub_handpresence = rospy.Publisher('/handpresence/flag', Bool, queue_size=1)
    sub_message_from_heat = rospy.Subscriber('/message_to_gui', String,sub_message_cb,queue_size=1)
    recovery_from_safety_sub = rospy.Subscriber('/recovery_from_safety', Bool,sub_recover_from_safety_cb,queue_size=1)
#################################

    image = Image.open(os.path.join(folder_path, "Pcb_photo.jpg"))
    image_logo = Image.open(os.path.join(folder_path, "lab_logo.png"))
    image_polimilogo = Image.open(os.path.join(folder_path, "Polimi_logo.png"))

    image_logo = image_logo.resize((int(490*0.67), 95), Image.Resampling.NEAREST)
    image_polimilogo = image_polimilogo.resize((int(664*0.5), int(488*0.5)), Image.Resampling.NEAREST)

    mygui = MyGUI(image,lablogo=image_logo,polimi_logo = image_polimilogo ,num_leds=7,pipeline=None)

    safty_lr = SafetyLayer()
    cam = RealSense_Cam()

    pipeline, config = cam.start_real_sense()
    # Start streaming
    pipeline.start(config)
    depth_scale, depth_intrin = cam.depth_information(pipeline)



    safty_lr.create_landmarker()


    mygui.pipeline=pipeline

    mygui.update_frame()
    mygui.mainloop()
    con.disconnect()