#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include <thread>
#include "ur5e_move/moverobotclass.hpp"
#include "ur5e_move/data_object.hpp"
#include "custome_interfaces/msg/safetycheck.hpp"

class MoveitDummyNode : public rclcpp::Node{

    public:
        MoveitDummyNode(rclcpp::NodeOptions& node_opt):Node("dummy_move_group",node_opt){
            RCLCPP_INFO(this->get_logger(),"Dummy node has been initializesd");
        }
};


class NormalNode : public rclcpp::Node{

    public:
        NormalNode(std::vector<PcbComponent> components, std::vector<SafeRobotConfig> safety_configs ):Node("ur5e_moveit_commander") {
            RCLCPP_INFO(this->get_logger(),"Normal node has been initialized");
           
            safety_check_sb      = this->create_subscription<custome_interfaces::msg::Safetycheck> (
            "knuckle_image", 1,std::bind(&NormalNode::callback_safetycheck,this,std::placeholders::_1));//  

            msg_to_gui_pb = this->create_publisher<std_msgs::msg::String>("msgtogui",10);
            safety_flag.data = false;
            this->list_of_components = components;
            this->list_of_safety_configs = safety_configs;

        }

    
    private:
    std::shared_ptr<rclcpp::Subscription<custome_interfaces::msg::Safetycheck>> safety_check_sb;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> msg_to_gui_pb;

    std_msgs::msg::Bool safety_flag; 

    std::vector<PcbComponent> list_of_components;
    std::vector<SafeRobotConfig> list_of_safety_configs;
    
    void callback_safetycheck(const std::shared_ptr<custome_interfaces::msg::Safetycheck> msg_safetycheck){

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


    void safety_measure(){
        RCLCPP_INFO(this->get_logger(), "The safety measure is running.");
    }
};


int main(int argc, char **argv){

    char path[1024];
    std::string target_dir;
    if (realpath(__FILE__, path)) {
        std::string script_dir_ = path;
        std::string script_dir = script_dir_.substr(0, script_dir_.find_last_of("/\\"));;
        std::cout << "Script directory is: " << script_dir << std::endl;

        std::string parent_dir = script_dir.substr(0, script_dir.find_last_of("/\\"));
        std::cout << "Parent directory is: " << parent_dir << std::endl;

        target_dir = parent_dir + "/data_base/data_20240717_153332.json";
        std::cout << "Target file path: " << target_dir << std::endl;
    } else {
        std::cerr << "Error resolving the absolute path." << std::endl;
    }

    std::ifstream json_file(target_dir);
    if (!json_file.is_open()) {
        std::cerr << "Failed to open file: " << target_dir << std::endl;
        return 1;
    }


    try {
        auto [components, safety_configs] = extract_data(target_dir);
        std::cout << "Number of PCB components: " << components.size() << std::endl;
        std::cout << "Number of Safety Configs: " << safety_configs.size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }



    auto [components, safety_configs] = extract_data(target_dir);

    rclcpp::init(argc,argv);
    
    std::shared_ptr<NormalNode> node2 = std::make_shared<NormalNode>(components,safety_configs);

    std::cout<<"Main thread is blocked "<<std::endl;
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);
    executor2.spin();
    std::cout<<"end effector:main 2"<<std::endl;

    return 0;
    
}


