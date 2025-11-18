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
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

#define JOY_TOPIC "/joy"
#define SERVO_POSE_TARGET_TOPIC "/servo_node/pose_target_cmds"
#define SERVO_SET_COMMAND_TYPE_SERVICE "/servo_node/switch_command_type"

class Planner : public rclcpp::Node {
    public:
        Planner();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr servo_publisher_;
        rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_client;

        geometry_msgs::msg::PoseStamped current_target_pose_;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
};
