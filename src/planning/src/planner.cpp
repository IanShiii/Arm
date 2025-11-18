#include "planner.hpp"

Planner::Planner() : rclcpp::Node("planner_node") {
    joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC,
        10,
        std::bind(&Planner::joy_callback, this, std::placeholders::_1)
    );

    servo_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        SERVO_POSE_TARGET_TOPIC,
        10
    );

    switch_input_client = this->create_client<moveit_msgs::srv::ServoCommandType>(SERVO_SET_COMMAND_TYPE_SERVICE);

    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
    switch_input_client->wait_for_service(std::chrono::seconds(10));
    switch_input_client->async_send_request(request);

    current_target_pose_ = geometry_msgs::msg::PoseStamped();
    current_target_pose_.header.frame_id = "base_link";
    current_target_pose_.pose.position.x = 0.2;
    current_target_pose_.pose.position.y = 0.2;
    current_target_pose_.pose.position.z = 0.2;
}

void Planner::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    current_target_pose_.header.stamp = this->now();
    
    // Left stick Y controls X position
    current_target_pose_.pose.position.x += msg->axes[1] * 0.01;

    // Left stick X controls Y position
    current_target_pose_.pose.position.y += -msg->axes[0] * 0.01;

    // Right stick Y controls Z position
    current_target_pose_.pose.position.z += msg->axes[3] * 0.01;

    // Publish the updated target pose
    servo_publisher_->publish(current_target_pose_);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<Planner>();
    rclcpp::spin(planner);
    rclcpp::shutdown();
    return 0;
}
