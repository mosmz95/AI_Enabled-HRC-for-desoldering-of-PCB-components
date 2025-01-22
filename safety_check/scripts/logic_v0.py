#!/usr/bin/env python3
import os
import sys
from screeninfo import get_monitors

import rospy
import json
from typing import List, Tuple
import numpy as np
import tkinter as tk

from tkinter import simpledialog, messagebox
from desoldering_pcb.moveit_interface import robotplanninginterface
from std_msgs.msg import Bool, String
from desoldering_pcb.data_object import Pcb_component, SafeRobotConfig
from desoldering_pcb.moveit_interface import robotplanninginterface
from robot_custom_msgs.msg import gui_msg

import threading
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def on_enter_pressed(event):
    if event.keysym == "Return":  # Check if Enter key is pressed
        root.destroy()  # Close the window after Enter is pressed

def create_input_window(prompt):
    global root

    # Create the main window
    root = tk.Tk()
    root.title("Press Enter")

    window_width = 400
    window_height = 100

    # Get the primary screen's width and height
    if len(get_monitors())==1:
        monitor = get_monitors()[0]
        screen_width = monitor.width
        screen_height = monitor.height

    # Calculate the position to center the window
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
    elif    len(get_monitors())==2:
        monitor = get_monitors()[1]
        screen_width = monitor.width
        screen_height = monitor.height

    # Calculate the position to center the window
        x = monitor.x + (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2

    # Set the window position and size
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")

    # Create and place the label with the prompt
    label = tk.Label(root, text=prompt)
    label.pack(pady=10)

    # Bind the Return event to the on_enter_pressed function
    root.bind('<Return>', on_enter_pressed)

    # Start the main event loop
    root.mainloop()





def extract_data(file_name)-> Tuple[List[Pcb_component], List[SafeRobotConfig]]:

    '''This functions returns list of pcb components and cobot safety configurations'''
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    target_dir = os.path.join(parent_dir, "data_base")
    json_path = os.path.join(target_dir, file_name)
    with open(json_path, "r") as json_file:
        raw_data = json.load(json_file)


    list_of_components =[]
    for i in range(len(raw_data["pcb_components"])):
        raw_component = raw_data["pcb_components"][i]
        temporary_component = Pcb_component(name = raw_component["name"],
                                            position=raw_component["tool_position_xyz"],
                                            orientatin=raw_component["tool_orientation_xyzw"],
                                            cobot_config=raw_component["cobot_JointSpace"],
                                            heattime=raw_component["Heat_duration"])
        list_of_components.append(temporary_component)

   

    if "Safety_configs" in raw_data.keys():
        list_of_safetyconfigs =[]
        for j in range(len(raw_data["Safety_configs"])):
            raw_safety = raw_data["Safety_configs"][j]
            temporary_safety = SafeRobotConfig(position=raw_safety["safety_tool_position_xyz"],
                                            orientatin=raw_safety["safety_tool_orientation_xyzw"],
                                            cobot_config=raw_safety["safety_cobot_JointSpace"])
            list_of_safetyconfigs.append(temporary_safety)


    return list_of_components, list_of_safetyconfigs





  
        


def subgui_cb(data:gui_msg):

    comonent_number = data.component_number
    heating_status = data.heating_success


    global list_of_tuples
    component_name = f"component_{comonent_number}"
    for (component,dis) in list_of_tuples:
        print(component_name)
        print(component.component_name())
        if component.component_name() == component_name:
            print("Hhhhhhhhhhhhhhhhhhhhhhhhhhhhhh")
            component.removal_failure()
            rospy.loginfo(f"Removal of {component.component_name()} was not successful. {component.heat_status()}")
            break

    print(comonent_number,heating_status)

    return


def subsafety_cb(data:bool):

    global cobot,list_of_safetyconfigs

    if data.data:
        rospy.loginfo("Hand is in the dangerous zone")

        # Signal the task to pause
        pause_event.set()

        cobot.move_group.stop()
        cobot.go_to_joint_state(list_of_safetyconfigs[0].safecobot_config(),angletype="rad")
        rospy.sleep(5)
        msg = String()
        msg.data = "Recovering from the safety conditions"
        message_to_gui_pub.publish(msg)
        recovery_msg = Bool()
        recovery_msg.data = True
        recovery_from_safety_pub.publish(recovery_msg)

        # Signal the task to resume
        resume_event.set()
        # Clear the pause event to prepare for the next message
        pause_event.clear()

    return




def task():

    global list_of_tuples
    list_of_tuples = [(component,np.linalg.norm(np.array(component.toolposition()))) for component in list_of_components ]
   
    counter = 1
    done_counter = 0
    done_counter_prev = 0
    if_number= 0

    while not rospy.is_shutdown():

        if pause_event.is_set():
            rospy.loginfo("Task paused...")
            # Wait until the resume event is set
            resume_event.wait()
            rospy.loginfo("Task resumed...")
        
        filtered_tuples = [(component, distance) for component, distance in list_of_tuples if not component.heat_status()] # returns list of component with heatstatus of 'false'
        
        if len(filtered_tuples) == 0 :
           
            
            
            if ((counter -  if_number) !=1):
                message=String()
                message.data = "Heating sequence is done !!!"
                message_to_gui_pub.publish(message)
                cobot.go_to_joint_state(cobot_home_config,angletype="rad")
                done_counter_prev = done_counter
                done_counter = counter

                print(f"prev:{done_counter_prev}")
                print(f"current:{done_counter}")
            if_number= counter
        else:

            closest_tuple = min(filtered_tuples, key=lambda x: x[1]) # returns the tuple with closest distance to cobots origin

            cobot.go_to_joint_state(closest_tuple[0].cobotconfig(),angletype="rad")
            rospy.sleep(closest_tuple[0].heatduration)
            #print(f"counter:{counter}")
            closest_tuple_index = list_of_tuples.index(closest_tuple)
            list_of_tuples[closest_tuple_index][0].got_heated()
            
        #print(f"counter:{counter}")
        counter +=1


def pcb_3Dpoint_extractor():

    global list_of_safetyconfigs, cobot,  list_of_components, cobot_home_config
    list_of_components, list_of_safetyconfigs = extract_data(file_name="data_20240717_153332.json")
    cobot = robotplanninginterface("manipulator")
   

    cobot_home_config = [1.7249945402145386,               -1.4170697343400498,                1.7957208792315882,
                4.254665060634277,                4.630741119384766,                3.6119256019592285            ]
   
    initial_joint_value= cobot.move_group.get_current_joint_values()
    vec1 = np.array(cobot_home_config)
    vec2 = np.array(initial_joint_value)
    print(type(cobot_home_config[0]))
    print(type(initial_joint_value[0]))

    if np.linalg.norm(vec1 - vec2)> 0.01:
        cobot.go_to_joint_state(cobot_home_config,angletype="rad")
    else:
       create_input_window("The cobot is at Home configuration!!!")
       message=String()
       message.data = "The cobot is at Home configuration!!!"
       message_to_gui_pub.publish(message)





if __name__=="__main__":

    rospy.init_node("operation_node", anonymous=False) 

    sub_gui = rospy.Subscriber('/component_heating/success', gui_msg,subgui_cb,queue_size=1)
    sub_safety = rospy.Subscriber('/handpresence/flag', Bool,subsafety_cb,queue_size=1)
    message_to_gui_pub = rospy.Publisher('/message_to_gui', String, queue_size=1)
    recovery_from_safety_pub = rospy.Publisher('/recovery_from_safety', Bool, queue_size=1)

    # Create events

    pause_event = threading.Event()
    resume_event = threading.Event()
   
    list_of_tuples = []
    pcb_3Dpoint_extractor()
    task_thread = threading.Thread(target=task)
    task_thread.start()
    print("exit main")
    rospy.spin()
   