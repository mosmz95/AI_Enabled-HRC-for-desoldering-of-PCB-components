#ifndef DESOLDERING_AUTOMATIC_SEQUENCE_HPP
#define DESOLDERING_AUTOMATIC_SEQUENCE_HPP


#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include <thread>
#include "ur5e_move/moverobotclass.hpp"
#include "ur5e_move/data_object.hpp"
#include "custom_interfaces/msg/safetycheck.hpp"
#include "custom_interfaces/msg/componentdata.hpp"
#include "custom_interfaces/msg/llmfeedback.hpp"





class HeatLogicNode : public rclcpp::Node{

    public:
        HeatLogicNode(std::shared_ptr<moveit_interface_cpp::MoveRobotClass>  robot, 
        std::vector<PcbComponent> components, std::vector<SafeRobotConfig> safety_configs);

    
    private:
    std::shared_ptr<moveit_interface_cpp::MoveRobotClass> robot;
    std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::Safetycheck>> safety_check_sb;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> msg_to_gui_pb;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> heating_status_of_component_check_sb;
    std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::Componentdata>> detected_component_data_sb;





    std::vector<double> Homeconfig_of_robot;
    std_msgs::msg::Bool safety_flag; 

    std::vector<PcbComponent> list_of_components;
    std::vector<SafeRobotConfig> list_of_safety_configs;
    std::vector<std::pair<PcbComponent, double>> list_of_tuples_;

    // pause_event_
    std::atomic<bool> pause_heating_process_;
    std::atomic<bool> resume_heating_process_;
    std::thread heatinglogic_thread_;
    std::atomic<bool> handpresence_;
    std::thread saftymonitoring_thread_;

    void callback_safetycheck(const std::shared_ptr<custom_interfaces::msg::Safetycheck> msg_safetycheck);

    void callback_heatingstatusofcomponent(const std::shared_ptr<std_msgs::msg::String> msg_heatingstatus);



    void safetymonitorying();
    void go_to_home_config();

    void initialize_tuples();
    void heatlogic();
};





#endif