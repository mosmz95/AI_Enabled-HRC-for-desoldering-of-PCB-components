
#ifndef MOVEIT_INTERFACE_CPP_MOVE_ROBOT_CLASS_HPP
#define MOVEIT_INTERFACE_CPP_MOVE_ROBOT_CLASS_HPP

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath> 
namespace moveit_interface_cpp{
class MoveRobotClass{

    public:
        MoveRobotClass(std::string planninggroup_val,std::shared_ptr<rclcpp::Node> node_val);
        void getPlanningFrame();
        std::vector<double> getCurrentJointValues(std::string angletype);
        void getPlanningGroupes();
        void goToJointGoal(std::vector<double> JointGoal,std::string angletype);
        void goToPoseGoal(const geometry_msgs::msg::Pose& goalpose);
        geometry_msgs::msg::PoseStamped getCurrentPose(const std::string& end_effector_linkk = "");

        // void moveAlongAxis(int value,std::string direction);
    
        const std::shared_ptr<rclcpp::Node> node;
        const std::string PLANNING_GROUP;
        moveit::planning_interface::MoveGroupInterface move_group;
    private:   
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const moveit::core::JointModelGroup* joint_model_group;
        moveit::core::RobotStatePtr current_state;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //
  // Next get the current set of joint values for the group.
        

};
}
#endif