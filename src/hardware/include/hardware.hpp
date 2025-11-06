#pragma once

#include <wiringPi.h>

#ifdef TRUE
#undef TRUE
#endif

#ifdef FALSE
#undef FALSE
#endif

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "servo.hpp"

namespace hardware {
    class RealHardware : public hardware_interface::SystemInterface {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(RealHardware)

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            hardware_interface::CallbackReturn on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;
            hardware_interface::return_type write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;

        private:
            servo::Servo servos_[5];
    };

    class SimHardware : public hardware_interface::SystemInterface {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(SimHardware)

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            hardware_interface::CallbackReturn on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;
            hardware_interface::return_type write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;

        private:
            servo::Servo servo_1_;
            servo::Servo servo_2_;
            servo::Servo servo_3_;
            servo::Servo servo_4_;
            servo::Servo servo_5_;
    };
};
