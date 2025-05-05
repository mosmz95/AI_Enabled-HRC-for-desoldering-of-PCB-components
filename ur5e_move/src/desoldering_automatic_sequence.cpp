#include "ur5e_move/desoldering_automatic_sequence.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"




HeatLogicNode::HeatLogicNode(std::shared_ptr<moveit_interface_cpp::MoveRobotClass>  robot, 
    std::vector<PcbComponent> components, std::vector<SafeRobotConfig> safety_configs )
    :Node("ur5e_moveit_commander"), robot{robot}, pause_heating_process_ {false},
      resume_heating_process_ {false}, handpresence_{false} {
    RCLCPP_INFO(this->get_logger(),"Normal node has been initialized");
    
    safety_check_sb      = this->create_subscription<custom_interfaces::msg::Safetycheck> (
    "knuckle_image", 1,std::bind(&HeatLogicNode::callback_safetycheck,this,std::placeholders::_1));//  

    heating_status_of_component_check_sb = this->create_subscription<std_msgs::msg::String> (
    "heating_status_of_component", 10,std::bind(&HeatLogicNode::callback_heatingstatusofcomponent,this,std::placeholders::_1));//  


    msg_to_gui_pb = this->create_publisher<std_msgs::msg::String>("msgtogui",10);

    this->safety_flag.data = false;
    this->list_of_components = components;
    this->list_of_safety_configs = safety_configs;
    this->Homeconfig_of_robot = std::vector<double> {1.7249945402145386, -1.4170697343400498, 1.7957208792315882, 4.254665060634277, 4.630741119384766, 3.6119256019592285 };
    this->go_to_home_config(); // the robot always goes to home config at first time.
    this->list_of_tuples_ = {};
     // Initialize list_of_tuples
    initialize_tuples();
    this->heatinglogic_thread_ = std::thread(&HeatLogicNode::heatlogic, this);
    this->saftymonitoring_thread_ = std::thread(&HeatLogicNode::safetymonitorying, this);
    
}


    
void HeatLogicNode::callback_safetycheck(const std::shared_ptr<custom_interfaces::msg::Safetycheck> msg_safetycheck){

    // RCLCPP_INFO(this->get_logger(), "The subscription to safetycheck topic has been made.");

    custom_interfaces::msg::Safetycheck rcv_msg = *msg_safetycheck;
    if (this->safety_flag.data == false && rcv_msg.hand_presence.data== true){
        RCLCPP_INFO(this->get_logger(), "The safety mechanism must be activated.");
    
        std_msgs::msg::String msg_;
        msg_.data = "The safety mechanism must be activated.";
        handpresence_.store(true); // this activates the safey mechanism
        this->msg_to_gui_pb->publish(msg_);

        // rclcpp_action::Client<moveit_msgs::action::MoveGroup> active_movegroup = robot->getMoveGroupClient();

        // robot->move_group.stop();
        // SafeRobotConfig sf_cnf = list_of_safety_configs.at(0);
        // robot->goToJointGoal(sf_cnf.safeCobotConfig(),"Rad");

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
        } else {             // here the condition for pausing or resuming should be set
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

void HeatLogicNode::safetymonitorying(){

    RCLCPP_INFO(this->get_logger(), "The safety monitorying is running.");

    // rclcpp_action::Client<moveit_msgs::action::MoveGroup>& mv_grp_client = robot->move_group.getMoveGroupClient();

    while (rclcpp::ok()){


        if (handpresence_.load()){
            robot->move_group.stop();

            RCLCPP_INFO(this->get_logger(), "The robot is going to safety config.");
            double plantime = robot->move_group.getPlanningTime();

            std::cout<< "plantime is:"<<plantime<<std::endl;
            robot->move_group.setPlanningTime(0.1);
            plantime = robot->move_group.getPlanningTime();
            double velocity_scaling_factor = 1;
            double accel_scaling_factor = 1;

            robot->move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);
            robot->move_group.setMaxVelocityScalingFactor(accel_scaling_factor);

            std::cout<< "planning time is set to: "<<plantime<<std::endl;

            SafeRobotConfig sf_cnf = list_of_safety_configs.at(0);
            robot->goToJointGoal(sf_cnf.safeCobotConfig(),"Rad");
            handpresence_.store(false);
        }
        
    }
    
}



