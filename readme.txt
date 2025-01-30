export ROS_DOMAIN_ID=1
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 launch_rviz:=false

export ROS_DOMAIN_ID=1
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:="ur5e" launch_rviz:=false


export ROS_DOMAIN_ID=1
ros2 run safety_check gui

ros2 run safety_check handlandmark
ros2 run safety_check wrist_camera_publisher

ros2 run safety_check component_detection_server
ros2 run safety_check component_selection_client 




ros2 launch ur5e_move heat_logic.launch.py ur_type:="ur5e"



ros2 topic pub --once /com_id custom_interfaces/msg/Llmfeedback "{number_id: 3, type: 'ComponentCounter'}"

ros2 topic pub --once /com_id custom_interfaces/msg/Llmfeedback "{number_id: 3, type: 'ComponentClass'}"