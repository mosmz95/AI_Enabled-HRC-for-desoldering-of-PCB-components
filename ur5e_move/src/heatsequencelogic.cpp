#include "ur5e_move/heatsequencelogic.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"




HeatLogicNode::HeatLogicNode(std::shared_ptr<moveit_interface_cpp::MoveRobotClass>  robot, 
    std::vector<PcbComponent> components, std::vector<SafeRobotConfig> safety_configs )
    :Node("ur5e_moveit_commander"), robot{robot}, pause_heating_process_ {false},
      resume_heating_process_ {false} {
    RCLCPP_INFO(this->get_logger(),"Normal node has been initialized");
    
    safety_check_sb      = this->create_subscription<custom_interfaces::msg::Safetycheck> (
    "knuckle_image", 1,std::bind(&HeatLogicNode::callback_safetycheck,this,std::placeholders::_1));//  

    detected_component_data_sb = this->create_subscription<custom_interfaces::msg::Componentdata> (
    "/detected_component_data", 1,std::bind(&HeatLogicNode::callback_ComponentData,this,std::placeholders::_1));//  Receiving the new component data

    heating_status_of_component_check_sb = this->create_subscription<std_msgs::msg::String> (
    "heating_status_of_component", 10,std::bind(&HeatLogicNode::callback_heatingstatusofcomponent,this,std::placeholders::_1));//  

    info_from_llm_sb = this->create_subscription<custom_interfaces::msg::Llmfeedback> (
    "/com_id", 10,std::bind(&HeatLogicNode::callback_info_llm,this,std::placeholders::_1));// 
    snapconfig_request_sb = this->create_subscription<std_msgs::msg::String> (
    "/go_snapshot_position", 10,std::bind(&HeatLogicNode::callback_snap_config,this,std::placeholders::_1));// 

    msg_to_gui_pb = this->create_publisher<std_msgs::msg::String>("msgtogui",10);
    safety_flag.data = false;
    this->list_of_components = components;
    this->list_of_safety_configs = safety_configs;
    this->Homeconfig_of_robot = std::vector<double> {1.7249945402145386, -1.4170697343400498, 1.7957208792315882, 4.254665060634277, 4.630741119384766, 3.6119256019592285 };
    this->go_to_home_config(); // the robot always goes to home config at first time.
    this->snap_config_of_robot = std::vector<double>  {1.5275, -1.38848, 1.39203, 5.11988, 4.50771, 3.18132};
    this->list_of_tuples_ = {};
     // Initialize list_of_tuples
    initialize_tuples();
    // heatinglogic_thread_ = std::thread(&HeatLogicNode::heatlogic, this);
    
}

void HeatLogicNode::callback_snap_config(const std::shared_ptr<std_msgs::msg::String> msg_safetycheck){
    RCLCPP_INFO(this->get_logger(), "Going to the snap config.");
    std::string msg = msg_safetycheck->data;
    if ( msg == "snapshot"){
        robot->goToJointGoal(this->snap_config_of_robot,"Rad");

    }

}  
    
void HeatLogicNode::callback_safetycheck(const std::shared_ptr<custom_interfaces::msg::Safetycheck> msg_safetycheck){

    // RCLCPP_INFO(this->get_logger(), "The subscription to safetycheck topic has been made.");

    custom_interfaces::msg::Safetycheck rcv_msg = *msg_safetycheck;
    if (this->safety_flag.data == false && rcv_msg.hand_presence.data== true){
        RCLCPP_INFO(this->get_logger(), "The safety mechanism must be activated.");
        std_msgs::msg::String msg_;
        msg_.data = "The safety mechanism must be activated.";
        this->msg_to_gui_pb->publish(msg_);
        robot->move_group.stop();
        SafeRobotConfig sf_cnf = list_of_safety_configs.at(0);
        robot->goToJointGoal(sf_cnf.safeCobotConfig(),"Rad");

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
void HeatLogicNode::callback_info_llm(const std::shared_ptr<custom_interfaces::msg::Llmfeedback> msg_llminfo){
    RCLCPP_INFO(this->get_logger(), "A new Llm request from the operator has been received.");
    // type must be ComponentCounter or ComponentClass
    unsigned int component_counter {0};
    std::string class_or_component = msg_llminfo->type;
    
    if ( class_or_component == "ComponentCounter" ){

        component_counter = msg_llminfo->number_id;
        if(component_counter > 0 && component_counter <= this->list_of_components.size() ){
            PcbComponent selected_component = this->list_of_components.at(component_counter - 1);
            robot->goToJointGoal(selected_component.cobotConfig(),"Rad");
                // cobot_.go_to_joint_state(closest_tuple.first.cobotconfig(), "rad");
            std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(selected_component.heatDuration() * 1000)));

            robot->goToJointGoal(Homeconfig_of_robot,"Rad");
        }else{
            RCLCPP_ERROR(this->get_logger(), "Invalid component index received.");

        }

    }else if (class_or_component == "ComponentClass"){

            RCLCPP_INFO(this->get_logger(), "The class of components for desoldering has been sepcified.");

    }
    

}

void HeatLogicNode::callback_ComponentData(const std::shared_ptr<custom_interfaces::msg::Componentdata> msg_componentData){
    RCLCPP_INFO(this->get_logger(), "The new components' data has been received.");
    std::vector<PcbComponent> new_component_data =  createComponentsFromROSMsg(msg_componentData->location_x,
                                                     msg_componentData->location_y,
                                                     msg_componentData->component_class);
}

// This function changes the heating status of a component to false.
void HeatLogicNode::callback_heatingstatusofcomponent(const std::shared_ptr<std_msgs::msg::String> msg_heatingstatus){
    std_msgs::msg::String component_name = *msg_heatingstatus;
    RCLCPP_INFO(this->get_logger(), "Heating status of %s is going to be changed to False.", component_name.data.c_str());

    // Mark the component as heated
    auto it = std::find_if(this->list_of_tuples_.begin(), this->list_of_tuples_.end(),
                            [&component_name](const auto& tuple) {
                                return tuple.first.componentName() == component_name.data;
                            });
    if (it != list_of_tuples_.end()) {
        it->first.removalFailure();
        RCLCPP_INFO(this->get_logger(), "Heating status of component %s is changed to False", it->first.componentName().c_str());
    }else{
        RCLCPP_INFO(this->get_logger(), "The component with the %s name does no exist in database.", component_name.data.c_str());
    }
}

void HeatLogicNode::safety_measure(){
    RCLCPP_INFO(this->get_logger(), "The safety measure is running.");
}

void HeatLogicNode::go_to_home_config(){
    
    std::vector<double> initial_joint_values = robot->getCurrentJointValues("Rad");
    if (initial_joint_values.size() != Homeconfig_of_robot.size()) {
        throw std::invalid_argument("The home config and current joint values are not consistent.");
    }
    double sum = 0.0;
    for (size_t i = 0; i < initial_joint_values.size(); ++i) {
        double diff = initial_joint_values[i] - Homeconfig_of_robot[i];
        sum += diff * diff;
    }
    double norm = std::sqrt(sum);
    if (norm > 0.01){
            RCLCPP_INFO(this->get_logger(), "The robot is going to the predefined home config.");
            robot->goToJointGoal(Homeconfig_of_robot,"Rad");
    }else{
        RCLCPP_INFO(this->get_logger(), "The robot is at home config.");
        std_msgs::msg::String msg_;
        msg_.data = "The robot is at home config.";
        this->msg_to_gui_pb->publish(msg_);
    }
    
}



void HeatLogicNode::initialize_tuples(){
     RCLCPP_INFO(this->get_logger(), "Components list with their distance from cobot's origin.");
    list_of_tuples_.clear();

    for (const auto& component : list_of_components) {
        double distance = std::sqrt(
            std::pow(component.toolPosition()[0], 2) +
            std::pow(component.toolPosition()[1], 2) +
            std::pow(component.toolPosition()[2], 2)
        );
        list_of_tuples_.emplace_back(component, distance);
    }
}

void HeatLogicNode::heatlogic(){
    RCLCPP_INFO(this->get_logger(), "The heating process has been started.");
    int counter = 1;
    int done_counter = 0;
    int done_counter_prev = 0;
    int if_number = 0;

    while (rclcpp::ok()) {
        // Handle pausing
        if (pause_heating_process_.load()) {
            RCLCPP_INFO(this->get_logger(), "Task paused...");
            while (pause_heating_process_.load() && rclcpp::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait until resumed
            }
            RCLCPP_INFO(this->get_logger(), "Task resumed...");
        }

        // Filter components with `heat_status` == false
        std::vector<std::pair<PcbComponent, double>> filtered_tuples;
        for (const auto& tuple : list_of_tuples_) {
            if (!tuple.first.heatStatus()) {
                filtered_tuples.push_back(tuple);
            }
        }

        if (filtered_tuples.empty()) {
            if ((counter - if_number) != 1) {
                RCLCPP_INFO(this->get_logger(), "Heating process has been done !!!");
                std::cout <<  "Heating process has been done !!!"<< std::endl;
                std_msgs::msg::String msg_;
                msg_.data = "Heating sequence has been done !!!";
                this->msg_to_gui_pb->publish(msg_);

                robot->goToJointGoal(Homeconfig_of_robot,"Rad");

                done_counter_prev = done_counter;
                done_counter = counter;

                std::cout << "prev: " << done_counter_prev << std::endl;
                std::cout << "current: " << done_counter << std::endl;
            }
            if_number = counter;
        } else {
            // Find the closest component based on distance
            auto closest_tuple = *std::min_element(filtered_tuples.begin(), filtered_tuples.end(),
                                                   [](const auto& a, const auto& b) {
                                                       return a.second < b.second;
                                                   });

            // Move the cobot to the component's configuration and heat it
            robot->goToJointGoal(closest_tuple.first.cobotConfig(),"Rad");
            // cobot_.go_to_joint_state(closest_tuple.first.cobotconfig(), "rad");
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(closest_tuple.first.heatDuration() * 1000)));

            // Mark the component as heated
            auto it = std::find_if(list_of_tuples_.begin(), list_of_tuples_.end(),
                                   [&closest_tuple](const auto& tuple) {
                                       return tuple.first == closest_tuple.first;
                                   });
            if (it != list_of_tuples_.end()) {
                it->first.gotHeated();
                RCLCPP_INFO(this->get_logger(), "Heated component is %s",it->first.componentName().c_str());
            }
        }

        counter++;
    }

}



