#pragma once

#include <string>
#include <vector>
#include <cmath>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#define JOY_TOPIC "/joy"

#define PLANNING_GROUP_NAME "arm"

std::map<int, std::string> button_to_position = {
    {0, "home"}, // X
    {1, "max"}, // A
    {2, "zero"}, // B
    {3, "UNDEFINED"}, // Y
    {4, "UNDEFINED"}, // LB
    {5, "UNDEFINED"}, // RB
    {6, "UNDEFINED"}, // LT
    {7, "UNDEFINED"}, // RT
    {8, "UNDEFINED"}, // BACK
    {9, "UNDEFINED"}, // START
    {10, "UNDEFINED"}, // LEFT STICK
    {11, "UNDEFINED"} // RIGHT STICK
};

class Planner : public rclcpp::Node {
    public:
        Planner();

        void initialize_move_group();

        /**
         * @returns true if the planning and execution was successful, false otherwise.
         */
        bool plan_and_execute_to_target(geometry_msgs::msg::Pose target_pose);

        /**
         * @returns true if the planning and execution was successful, false otherwise.
         */
        bool plan_and_execute_to_predefined_position(std::string position_name);

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

        geometry_msgs::msg::Pose current_target_pose_;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
};
