#include "servo.hpp"

namespace servo {
    Servo::Servo(
        int gpio_pin, 
        double min_pwm_width_microseconds, 
        double max_pwm_width_microseconds, 
        double min_angle_radians, 
        double max_angle_radians,
        double starting_angle_radians
    ) : 
        target_angle_radians_(starting_angle_radians),
        gpio_pin_(gpio_pin),
        min_pwm_width_microseconds_(min_pwm_width_microseconds),
        max_pwm_width_microseconds_(max_pwm_width_microseconds),
        min_angle_radians_(min_angle_radians),
        max_angle_radians_(max_angle_radians)
    {}

    Servo::Servo() : 
        target_angle_radians_(0.0),
        gpio_pin_(-1),
        min_pwm_width_microseconds_(0.0),
        max_pwm_width_microseconds_(0.0),
        min_angle_radians_(0.0),
        max_angle_radians_(0.0)
    {}

    void Servo::configure() {
        gpioSetMode(gpio_pin_, PI_OUTPUT);
    }

    void Servo::write_position() {
        assert(target_angle_radians_ >= min_angle_radians_ && target_angle_radians_ <= max_angle_radians_);
        
        double pwm_width = position_to_pwm_width(target_angle_radians_);
        gpioServo(gpio_pin_, static_cast<unsigned int>(pwm_width));
    }

    double Servo::position_to_pwm_width(double position) {
        double angle_range = max_angle_radians_ - min_angle_radians_;
        double pwm_range = max_pwm_width_microseconds_ - min_pwm_width_microseconds_;
        return min_pwm_width_microseconds_ + (position - min_angle_radians_) / angle_range * pwm_range;
    }
}
