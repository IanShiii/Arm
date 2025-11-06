#include "planner.hpp"

Planner::Planner() : rclcpp::Node("planner_node") {
    joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC,
        10,
        std::bind(&Planner::joy_callback, this, std::placeholders::_1)
    );

    current_target_pose_ = geometry_msgs::msg::Pose();
    current_target_pose_.position.x = 0.2;
    current_target_pose_.position.y = 0.2;
    current_target_pose_.position.z = 0.2;
}

void Planner::initialize_move_group() {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP_NAME);

    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.8);
    move_group_->setMaxAccelerationScalingFactor(0.8);    

    move_group_->setGoalPositionTolerance(0.05);
    move_group_->setGoalOrientationTolerance(3.14); // Allow any orientation

    RCLCPP_INFO(get_logger(), "Move group '%s' initialized with end effector link '%s'.",
                PLANNING_GROUP_NAME, move_group_->getEndEffectorLink().c_str());
}

void Planner::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons[0] == 1) { // X button
        geometry_msgs::msg::PoseStamped target_pose = move_group_->getRandomPose();
        move_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_->execute(plan);
        } else {
            RCLCPP_WARN(get_logger(), "Failed to find valid random plan.");
        }
    }
    // for (float x = 0.1; x < 0.5; x += 0.05) {
    //     for (float y = 0.1; y < 0.5; y += 0.05) {
    //         for (float z = 0.0; z < 0.5; z += 0.05) {
    //             current_target_pose_.position.x = x;
    //             current_target_pose_.position.y = y;
    //             current_target_pose_.position.z = z;
    //             move_group_->setPositionTarget(x, y, z);
    //             moveit::planning_interface::MoveGroupInterface::Plan plan;
    //             bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //             if (success) {
    //                 RCLCPP_INFO(get_logger(), "Found valid plan to: [%.2f, %.2f, %.2f]", x, y, z);
    //                 return;
    //             }
    //         }
    //     }
    // }

    // if (msg->buttons[0] == 1) { // X button
    //     current_target_pose_.position.x -= 0.025;
    //     plan_and_execute_to_target(current_target_pose_);
    //     return;
    // } else if (msg->buttons[1] == 1) { // A button
    //     current_target_pose_.position.y -= 0.025;
    //     plan_and_execute_to_target(current_target_pose_);
    //     return;
    // } else if (msg->buttons[2] == 1) { // B button
    //     current_target_pose_.position.x += 0.025;
    //     plan_and_execute_to_target(current_target_pose_);
    //     return;
    // } else if (msg->buttons[3] == 1) { // Y button
    //     current_target_pose_.position.y += 0.025;
    //     plan_and_execute_to_target(current_target_pose_);
    //     return;
    // } else if (msg->axes[5] < -0.5) { // D pad down
    //     current_target_pose_.position.z -= 0.025;
    //     plan_and_execute_to_target(current_target_pose_);
    //     return;
    // } else if (msg->axes[5] > 0.5) { // D pad up
    //     current_target_pose_.position.z += 0.025;
    //     plan_and_execute_to_target(current_target_pose_);
    //     return;
    // }

    // for (size_t i = 0; i < msg->buttons.size(); i++) {
    //     if (msg->buttons[i] == 1) {
    //         RCLCPP_INFO(get_logger(), "Joystick button %zu pressed, moving to '%s' position.", i, button_to_position[i].c_str());
    //         plan_and_execute_to_predefined_position(button_to_position[i]);
    //         return;
    //     }
    // }
}

bool Planner::plan_and_execute_to_target(geometry_msgs::msg::Pose target_pose) {
    RCLCPP_INFO(get_logger(), "Planning to target pose: [%.2f, %.2f, %.2f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    move_group_->setStartStateToCurrentState();
    move_group_->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(get_logger(), "Planning successful, executing plan.");
        move_group_->execute(plan);
    } else {
        RCLCPP_WARN(get_logger(), "Planning failed.");
    }

    return success;
}

bool Planner::plan_and_execute_to_predefined_position(std::string position_name) {
    RCLCPP_INFO(get_logger(), "Planning to predefined position: %s", position_name.c_str());
    move_group_->setNamedTarget(position_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(get_logger(), "Planning successful, executing plan.");
        move_group_->execute(plan);
    } else {
        RCLCPP_WARN(get_logger(), "Planning failed.");
    }

    return success;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<Planner>();
    planner->initialize_move_group();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.2;
    planner->plan_and_execute_to_target(target_pose);

    rclcpp::spin(planner);
    rclcpp::shutdown();
    return 0;
}
