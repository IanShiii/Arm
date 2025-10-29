#pragma once

#include <pigpio.h>
#include <stdexcept>
#include <cassert>

namespace servo {
    class Servo {
        public:
            double target_angle_radians_;
            Servo(
                int gpio_pin, 
                double min_pwm_width_microseconds, 
                double max_pwm_width_microseconds, 
                double min_angle_radians = 0.0, 
                double max_angle_radians = 3.1416,
                double starting_angle_radians = 1.5708
            );

            Servo();

            void configure();
            void write_position();

        private:
            int gpio_pin_;
            double min_pwm_width_microseconds_;
            double max_pwm_width_microseconds_;
            double min_angle_radians_;
            double max_angle_radians_;

            double position_to_pwm_width(double position);
    };
}
