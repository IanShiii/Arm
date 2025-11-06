#include "hardware.hpp"

namespace hardware {
    hardware_interface::CallbackReturn RealHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
        for (size_t i = 0; i < 5; ++i) {
            servos_[i] = servo::Servo(
                std::stoi(params.hardware_info.joints.at(i).parameters.at("pin")), 
                std::stod(params.hardware_info.joints.at(i).parameters.at("min_pwm_width_microseconds")),
                std::stod(params.hardware_info.joints.at(i).parameters.at("max_pwm_width_microseconds")),
                std::stod(params.hardware_info.joints.at(i).parameters.at("min_position_radians")),
                std::stod(params.hardware_info.joints.at(i).parameters.at("max_position_radians")),
                std::stod(params.hardware_info.joints.at(i).parameters.at("starting_position_radians"))
            );
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface> RealHardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < 5; ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                "joint_" + std::to_string(i + 1), hardware_interface::HW_IF_POSITION, &servos_[i].target_angle_radians_));
        }
        return command_interfaces;
    }

    // Can't actually read state from servos, so just return the commanded positions
    std::vector<hardware_interface::StateInterface> RealHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < 5; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                "joint_" + std::to_string(i + 1), hardware_interface::HW_IF_POSITION, &servos_[i].target_angle_radians_));
        }
        return state_interfaces;
    }

    hardware_interface::CallbackReturn RealHardware::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        wiringPiSetupGpio();

        for (servo::Servo &servo : servos_) {
            servo.configure();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RealHardware::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        // Nothing needed for activation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RealHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        for (servo::Servo &servo : servos_) {
            servo.deactivate();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RealHardware::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RealHardware::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        // No state to read
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RealHardware::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        for (servo::Servo &servo : servos_) {
            servo.write_position();
        }

        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(hardware::RealHardware, hardware_interface::SystemInterface)
