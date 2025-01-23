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
        HeatLogicNode(std::shared_ptr<moveit_interface_cpp::MoveRobotClass>  robot, 
        std::vector<PcbComponent> components, std::vector<SafeRobotConfig> safety_configs);

    
    private:
    std::shared_ptr<moveit_interface_cpp::MoveRobotClass> robot;
    std::shared_ptr<rclcpp::Subscription<custome_interfaces::msg::Safetycheck>> safety_check_sb;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> msg_to_gui_pb;

    std::vector<double> Homeconfig_of_robot;

    std_msgs::msg::Bool safety_flag; 

    std::vector<PcbComponent> list_of_components;
    std::vector<SafeRobotConfig> list_of_safety_configs;
    std::vector<std::pair<PcbComponent, double>> list_of_tuples_;

    // pause_event_
    std::atomic<bool> pause_heating_process_;
    std::atomic<bool> resume_heating_process_;
    std::thread heatinglogic_thread_;

    void callback_safetycheck(const std::shared_ptr<custome_interfaces::msg::Safetycheck> msg_safetycheck);




    void safety_measure();

    void go_to_home_config();

    void initialize_tuples();
    void heatlogic();
};





#endif