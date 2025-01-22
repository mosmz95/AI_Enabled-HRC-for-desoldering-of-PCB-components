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
        NormalNode(std::shared_ptr<moveit_interface_cpp::MoveRobotClass>  robot)
       :Node("ur5e_moveit_commander"), robot{robot} {
            RCLCPP_INFO(this->get_logger(),"Normal node has been initialized");
           
            safety_check_sb      = this->create_subscription<custome_interfaces::msg::Safetycheck> (
            "knuckle_image", 1,std::bind(&NormalNode::callback_safetycheck,this,std::placeholders::_1));//  
           safety_flag.data = false;

        }

    
    private:
    std::shared_ptr<rclcpp::Subscription<custome_interfaces::msg::Safetycheck>> safety_check_sb;
    // std::shared_ptr<moveit_interface_cpp::MoveRobotClass> robot;

    std_msgs::msg::Bool safety_flag; 

    
    void callback_safetycheck(const std::shared_ptr<custome_interfaces::msg::Safetycheck> msg_safetycheck){

        RCLCPP_INFO(this->get_logger(), "The subscription to safetycheck topic has been made.");

        custome_interfaces::msg::Safetycheck rcv_msg = *msg_safetycheck;
        this->safety_flag.data = rcv_msg.hand_presence.data;

        RCLCPP_INFO(this->get_logger(), "The hand presense status is: %s", this->safety_flag.data ? "True": "False");

    }


    
};


int main(int argc, char **argv){

    rclcpp::init(argc,argv);
    // rclcpp::NodeOptions node_options;
    // node_options.automatically_declare_parameters_from_overrides(true);

    // std::shared_ptr<MoveitDummyNode> node = std::make_shared<MoveitDummyNode>(node_options);
    
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // std::thread([&executor]() { executor.spin(); }).detach();
    

    // std::shared_ptr<moveit_interface_cpp::MoveRobotClass> ur5e = std::make_shared<moveit_interface_cpp::MoveRobotClass>("ur_manipulator", node);
    // std::shared_ptr<NormalNode> node2 = std::make_shared<NormalNode>(ur5e);
    std::shared_ptr<NormalNode> node2 = std::make_shared<NormalNode>(ur5e);



    std::cout<<"Main thread is blocked "<<std::endl;
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);
    executor2.spin();
    std::cout<<"end effector:main 2"<<std::endl;

    return 0;
    
}


