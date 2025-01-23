#include "heatsequencelogic.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>





HeatLogicNode::HeatLogicNode(std::vector<PcbComponent> components, std::vector<SafeRobotConfig> safety_configs ):Node("ur5e_moveit_commander") {
    RCLCPP_INFO(this->get_logger(),"Normal node has been initialized");
    
    safety_check_sb      = this->create_subscription<custome_interfaces::msg::Safetycheck> (
    "knuckle_image", 1,std::bind(&HeatLogicNode::callback_safetycheck,this,std::placeholders::_1));//  

    msg_to_gui_pb = this->create_publisher<std_msgs::msg::String>("msgtogui",10);
    safety_flag.data = false;
    this->list_of_components = components;
    this->list_of_safety_configs = safety_configs;
    this->Homeconfig_of_robot = std::vector<double> {1.7249945402145386, -1.4170697343400498, 1.7957208792315882, 4.254665060634277, 4.630741119384766, 3.6119256019592285 };
    this->go_to_home_config(); // the robot always goes to home config at first time.

}

    
    
void HeatLogicNode::callback_safetycheck(const std::shared_ptr<custome_interfaces::msg::Safetycheck> msg_safetycheck){

    // RCLCPP_INFO(this->get_logger(), "The subscription to safetycheck topic has been made.");

    custome_interfaces::msg::Safetycheck rcv_msg = *msg_safetycheck;
    if (this->safety_flag.data == false && rcv_msg.hand_presence.data== true){
        RCLCPP_INFO(this->get_logger(), "The safety mechanism must be activated.");
        std_msgs::msg::String msg_;
        msg_.data = "The safety mechanism must be activated.";
        this->msg_to_gui_pb->publish(msg_);
    }

    if (this->safety_flag.data == true && rcv_msg.hand_presence.data== false){
        RCLCPP_INFO(this->get_logger(), "The recovery from safety must be adopted.");
        std_msgs::msg::String msg_;
        msg_.data = "The recovery from safety must be adopted.";
        this->msg_to_gui_pb->publish(msg_);
    }

    if ( this->safety_flag.data != rcv_msg.hand_presence.data ){ // if the previous flag was different with current flag, update the flag status
        this->safety_flag.data = rcv_msg.hand_presence.data;
    }

    // RCLCPP_INFO(this->get_logger(), "The hand presense status is: %s", this->safety_flag.data ? "True": "False");

}


void HeatLogicNode::safety_measure(){
    RCLCPP_INFO(this->get_logger(), "The safety measure is running.");
}

void HeatLogicNode::go_to_home_config(){
    RCLCPP_INFO(this->get_logger(), "The robot is going to the predefined home config.");
}


void HeatLogicNode::heatlogic(){
    RCLCPP_INFO(this->get_logger(), "The heating process has been started.");
}







