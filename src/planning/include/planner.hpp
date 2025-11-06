#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include <string>
#include <vector>
#include <cmath>

#include "interfaces/msg/target.hpp"

#define JOY_TOPIC "/joy"

#define PLANNING_GROUP_NAME "arm"
#define PLANNING_FRAME_NAME "end_effector"

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
        bool plan_and_execute_to_target(interfaces::msg::Target target);

        /**
         * @returns true if the planning and execution was successful, false otherwise.
         */
        bool plan_and_execute_to_predefined_position(std::string position_name);

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_;
};
