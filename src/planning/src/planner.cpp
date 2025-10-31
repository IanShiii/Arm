#include "planner.hpp"

Planner::Planner() : rclcpp::Node("planner_node") {
    target_subscriber_ = this->create_subscription<interfaces::msg::Target>(
        TARGET_TOPIC,
        10,
        std::bind(&Planner::plan_and_execute_to_target, this, std::placeholders::_1)
    );
}

void Planner::initialize_move_group() {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP_NAME);

    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setMaxVelocityScalingFactor(0.8);
    move_group_->setMaxAccelerationScalingFactor(0.8);
}

bool Planner::plan_and_execute_to_target(interfaces::msg::Target target) {
    RCLCPP_INFO(get_logger(), "Planning to target: x: %f, y: %f, z: %f",
                target.target_pose.pose.position.x,
                target.target_pose.pose.position.y,
                target.target_pose.pose.position.z);
    
    geometry_msgs::msg::PoseStamped target_pose = target.target_pose;
    geometry_msgs::msg::Pose tolerance = target.tolerance;

    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.link_name = move_group_->getEndEffectorLink();
    orientation_constraint.header.frame_id = PLANNING_FRAME_NAME;
    orientation_constraint.orientation = target_pose.pose.orientation;

    orientation_constraint.absolute_x_axis_tolerance = tolerance.orientation.x;
    orientation_constraint.absolute_y_axis_tolerance = tolerance.orientation.y;
    orientation_constraint.absolute_z_axis_tolerance = tolerance.orientation.z;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(orientation_constraint);

    move_group_->setPathConstraints(path_constraints);
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    move_group_->clearPathConstraints();

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
    rclcpp::spin(planner);
    rclcpp::shutdown();
    return 0;
}
