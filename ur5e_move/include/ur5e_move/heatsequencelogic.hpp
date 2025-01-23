#ifndef HEATSEQUENCELOGIC_HPP
#define HEATSEQUENCELOGIC_HPP


#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include <thread>
#include "ur5e_move/moverobotclass.hpp"
#include "ur5e_move/data_object.hpp"
#include "custome_interfaces/msg/safetycheck.hpp"



class HeatLogicNode : public rclcpp::Node{

    public:
        HeatLogicNode();

    
    private:
    std::shared_ptr<rclcpp::Subscription<custome_interfaces::msg::Safetycheck>> safety_check_sb;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> msg_to_gui_pb;

    std_msgs::msg::Bool safety_flag; 

    std::vector<PcbComponent> list_of_components;
    std::vector<SafeRobotConfig> list_of_safety_configs;
    std::vector<double> Homeconfig_of_robot;

    void callback_safetycheck(const std::shared_ptr<custome_interfaces::msg::Safetycheck> msg_safetycheck);


    void safety_measure();

    void go_to_home_config();

    void heatlogic();
};





#endif