#include "moveitinterface_cpp/moverobotclass.hpp"

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath> 
#include <cstring>

//,move_group{node_val,PLANNING_GROUP}
namespace moveit_interface_cpp{

MoveRobotClass::MoveRobotClass(std::string planninggroup_val,std::shared_ptr<rclcpp::Node> node_val)
    :   node{node_val},
        PLANNING_GROUP{planninggroup_val},
        move_group{node_val,PLANNING_GROUP},
        joint_model_group{ move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP)},
        current_state{move_group.getCurrentState()}
        {
        RCLCPP_INFO(node_val->get_logger(),"move robot has been initialized");
    }




void MoveRobotClass::getPlanningFrame(){
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
}


std::vector<double> MoveRobotClass::getCurrentJointValues(std::string angletype){
   std::vector<double> joint_group_positions;
   std::ostringstream joint_positions_stream;
   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
   if (angletype=="Rad"){
    for (const auto& jointangle : joint_group_positions) {
        joint_positions_stream << jointangle  << " ";

    }
    RCLCPP_INFO(node->get_logger(), "Joint angles[Rad]: [ %s ]", joint_positions_stream.str().c_str());
    return joint_group_positions;
   }
   else if(angletype=="Deg"){
    std::vector<double> joint_group_positions_deg{};
    for (const auto& jointangle : joint_group_positions) {
        joint_group_positions_deg.push_back(jointangle * (180.0 / M_PI));
        joint_positions_stream << jointangle * (180.0 / M_PI) << " ";
    }
    RCLCPP_INFO(node->get_logger(), "Joint angles[deg]: [ %s ]", joint_positions_stream.str().c_str());
    return joint_group_positions_deg;
   }else {
       RCLCPP_WARN(node->get_logger(), "Invalid angletype: [ %s ], returning empty vector", angletype.c_str());
       return std::vector<double>{};
   }
}

void MoveRobotClass::getPlanningGroupes(){
    RCLCPP_INFO(node->get_logger(), "Available Planning Groups:");
    std::copy(this->move_group.getJointModelGroupNames().begin(), this->move_group.getJointModelGroupNames().end(),
        std::ostream_iterator<std::string>(std::cout, ", "));
}

void MoveRobotClass::goToJointGoal(std::vector<double> JointGoal_inp,std::string angletype){
    std::vector<double> JointGoal{};
    if (angletype=="Rad"){
        JointGoal=JointGoal_inp;
    }
    else if(angletype=="Deg"){
        for(double const &dd : JointGoal_inp){
            JointGoal.push_back(dd * (180.0 / M_PI));
        }
    }
    RCLCPP_INFO(node->get_logger(), "going to the joint goal:");
    bool within_bounds = this->move_group.setJointValueTarget(JointGoal);
    if (!within_bounds)
    {
    RCLCPP_WARN(node->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }
    bool success = (this->move_group.plan(this->my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "plan in joint space goal is: %s", success ? "" : "FAILED");
    if (success){
        this->move_group.execute(my_plan);
    }
}
    geometry_msgs::msg::PoseStamped MoveRobotClass::getCurrentPose(const std::string& end_effector_linkk){
        geometry_msgs::msg::PoseStamped cur_pose = this->move_group.getCurrentPose(end_effector_linkk);
        auto orientaion = cur_pose.pose.orientation;
        auto position = cur_pose.pose.position;
        std::ostringstream orientation_stream;
        std::ostringstream position_stream;

        orientation_stream << orientaion.x << "" <<orientaion.y<< "" << orientaion.z<< "" << orientaion.w;
        position_stream <<  position.x << "" << position.y<< "" <<  position.z;

        RCLCPP_INFO(node->get_logger(), "The current orientation in Quaternion is: [ %s ]", orientation_stream.str().c_str());
        RCLCPP_INFO(node->get_logger(), "The current position in xyz is: [ %s ]", position_stream.str().c_str());

        return cur_pose; 
    }

void MoveRobotClass::goToPoseGoal(const geometry_msgs::msg::Pose& goalpose){
    if (this->move_group.setPoseTarget(goalpose)){
        RCLCPP_INFO(node->get_logger(), "The pose target is valid");
        bool success = (this->move_group.plan(this->my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(node->get_logger(), "plan to pose goal is: %s", success ? "" : "FAILED");
        if (success){
            this->move_group.execute(my_plan);
        }


    }
    else {
        RCLCPP_INFO(node->get_logger(), "The pose target is invalid");
    }
}

// void MoveRobotClass::moveAlongAxis(int value,std::string direction){
//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     geometry_msgs::msg::Pose gg = this->move_group.getCurrentPose().pose;

// }
}