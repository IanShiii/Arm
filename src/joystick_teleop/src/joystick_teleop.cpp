#include "joystick_teleop.hpp"

JoystickTeleopNode::JoystickTeleopNode() : rclcpp::Node("JoystickTeleopNode") {
    joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC,
        10,
        std::bind(&JoystickTeleopNode::joy_callback, this, std::placeholders::_1)
    );

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        SERVO_TWIST_TARGET_TOPIC,
        10
    );

    switch_input_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(SERVO_SET_COMMAND_TYPE_SERVICE);

    current_command_type_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    current_command_type_->command_type = moveit_msgs::srv::ServoCommandType_Request::TWIST;

    set_servo_command_type(current_command_type_);
}

void JoystickTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    double joystick_left_x = msg->axes[0];
    double joystick_left_y = msg->axes[0];
    double joystick_right_x = msg->axes[0];
    double joystick_right_y = msg->axes[0];

    int left_trigger = msg->buttons[0];
    int right_trigger = msg->buttons[0];
    int left_bumper = msg->buttons[0];
    int right_bumper = msg->buttons[0];

    if (current_command_type_->command_type != moveit_msgs::srv::ServoCommandType_Request::TWIST) {
        if (abs(joystick_left_x) > DEADBAND || abs(joystick_left_y) > DEADBAND || abs(joystick_right_y) > DEADBAND) {
            current_command_type_->command_type = moveit_msgs::srv::ServoCommandType_Request::TWIST;
            set_servo_command_type(current_command_type_);
        } else {
            return;
        }
    }

    geometry_msgs::msg::Twist twist_msg;

    // Left stick controls XY linear velocity
    twist_msg.linear.x = joystick_left_y;
    twist_msg.linear.y = joystick_left_x;

    // Right stick controls roll and pitch
    twist_msg.angular.x = joystick_right_x; // Roll
    twist_msg.angular.y = joystick_right_y; // Pitch

    // Triggers control Z linear velocity
    twist_msg.linear.z = right_trigger - left_trigger;

    // Bumpers control yaw
    twist_msg.angular.z = right_bumper - left_bumper;

    twist_publisher_->publish(twist_msg);
}

void JoystickTeleopNode::set_servo_command_type(moveit_msgs::srv::ServoCommandType::Request::SharedPtr command_type) {
    switch_input_client_->wait_for_service(std::chrono::seconds(5));
    auto result = switch_input_client_->async_send_request(command_type);
    std::string input_type_str;
    switch (command_type->command_type) {
        case moveit_msgs::srv::ServoCommandType_Request::JOINT_JOG:
            input_type_str = "Joint Jog";
            break;
        case moveit_msgs::srv::ServoCommandType_Request::TWIST:
            input_type_str = "Twist";
            break;
        case moveit_msgs::srv::ServoCommandType_Request::POSE:
            input_type_str = "Pose";
            break;
        default:
            input_type_str = "Unknown";
    }

    if (result.get()->success) {
        RCLCPP_INFO(get_logger(), "Switched to input type: %s", input_type_str.c_str());
    }
    else {
        RCLCPP_WARN(get_logger(), "Could not switch input to: %s", input_type_str.c_str());
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto servo_node = std::make_shared<JoystickTeleopNode>();
    rclcpp::spin(servo_node);
    rclcpp::shutdown();
    return 0;
}
