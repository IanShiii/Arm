#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include <string>
#include <vector>
#include <cmath>

#include "interfaces/msg/target.hpp"

#define TARGET_TOPIC "/target"
#define NAMED_TARGET_TOPIC "/named_target"

#define PLANNING_GROUP_NAME "arm"
#define PLANNING_FRAME_NAME "end_effector"

class Planner : public rclcpp::Node {
    public:
        Planner();

        void initialize_move_group();

        /**
         * @returns true if the planning and execution was successful, false otherwise.
         */
        bool plan_and_execute_to_target(interfaces::msg::Target target);

        bool plan_and_execute_to_predefined_position(std::string position_name);

    private:
        rclcpp::Subscription<interfaces::msg::Target>::SharedPtr target_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr named_target_subscriber_;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_;
};
