# Intuitive AI-driven Human Robot Collaboration in desoldering of printed circuit boards (PCBs) considering the operator's safety  


In the experiment, a collaborative robot (UR5e) has been equipted with a hot gun (desoldering tool), the cobot heats up the components and an operator removes the desoldered components. The setup consists of two modules: (i) Safety module which monitors the position of the desoldering tool with respect to operator's hand knuckls, (ii)  AI-Based graphical user interface(GUI) which enables the operator to select the type of components he wants to desolder through a vocal command and the LLMs interperation or a direct select with a 'Button' on the GUI. Below you can find each modules in more detail:

## Safey module

## AI-Based GUI

export ROS_DOMAIN_ID=1
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 launch_rviz:=false

export ROS_DOMAIN_ID=1
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:="ur5e" launch_rviz:=false


export ROS_DOMAIN_ID=1
ros2 run safety_check gui

export ROS_DOMAIN_ID=1
ros2 run safety_check handlandmark

export ROS_DOMAIN_ID=1
ros2 run safety_check wrist_camera_publisher

export ROS_DOMAIN_ID=1
ros2 run safety_check component_detection_server

export ROS_DOMAIN_ID=1
ros2 run safety_check component_selection_client 



export ROS_DOMAIN_ID=1
ros2 launch ur5e_move heat_logic.launch.py ur_type:="ur5e"

ros2 launch ur5e_move automatic_desoldering.launch.py ur_type:="ur5e"


ros2 topic pub --once /com_id custom_interfaces/msg/Llmfeedback "{number_id: 3, type: 'ComponentCounter'}"

ros2 topic pub --once /com_id custom_interfaces/msg/Llmfeedback "{number_id: 1, type: 'ComponentClass'}"