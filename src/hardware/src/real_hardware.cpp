#include "hardware.hpp"

namespace hardware {
    hardware_interface::CallbackReturn RealHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to initialize pigpio library"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        servo::Servo *servos[5] = {&servo_1_, &servo_2_, &servo_3_, &servo_4_, &servo_5_};
        for (size_t i = 0; i < 5; ++i) {
            *servos[i] = servo::Servo(
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
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "joint_1", hardware_interface::HW_IF_POSITION, &servo_1_.target_angle_radians_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "joint_2", hardware_interface::HW_IF_POSITION, &servo_2_.target_angle_radians_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "joint_3", hardware_interface::HW_IF_POSITION, &servo_3_.target_angle_radians_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "joint_4", hardware_interface::HW_IF_POSITION, &servo_4_.target_angle_radians_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "joint_5", hardware_interface::HW_IF_POSITION, &servo_5_.target_angle_radians_));
        return command_interfaces;
    }

    // Can't actually read state from servos, so just return the commanded positions
    std::vector<hardware_interface::StateInterface> RealHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "joint_1", hardware_interface::HW_IF_POSITION, &servo_1_.target_angle_radians_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "joint_2", hardware_interface::HW_IF_POSITION, &servo_2_.target_angle_radians_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "joint_3", hardware_interface::HW_IF_POSITION, &servo_3_.target_angle_radians_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "joint_4", hardware_interface::HW_IF_POSITION, &servo_4_.target_angle_radians_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "joint_5", hardware_interface::HW_IF_POSITION, &servo_5_.target_angle_radians_));
        return state_interfaces;
    }

    hardware_interface::CallbackReturn RealHardware::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        servo_1_.configure();
        servo_2_.configure();
        servo_3_.configure();
        servo_4_.configure();
        servo_5_.configure();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RealHardware::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        // Nothing needed for activation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RealHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        // Nothing needed for deactivation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RealHardware::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        gpioTerminate();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RealHardware::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        // No state to read
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RealHardware::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        servo_1_.write_position();
        servo_2_.write_position();
        servo_3_.write_position();
        servo_4_.write_position();
        servo_5_.write_position();

        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(hardware::RealHardware, hardware_interface::SystemInterface)
