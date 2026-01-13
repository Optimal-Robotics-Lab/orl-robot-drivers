#pragma once

#include <array>
#include <cstdint>
#include <algorithm>

#include "src/digit/lib/utils/constants.h"
#include "src/digit/msgs/digit_msgs.h"


namespace robot::digit::utilities {
    
    inline const digit_interface::msg::DigitCommand& initialize_command() {
        static const digit_interface::msg::DigitCommand base_command = [] {
            digit_interface::msg::DigitCommand command{};

            command.apply_command = false;
            command.fallback_opmode = static_cast<int32_t>(robot::digit::constants::OpMode::Disabled);

            std::ranges::for_each(command.motors, [](auto& motor) {
                motor.position = 0.0;
                motor.velocity = 0.0;
                motor.torque = 0.0;
                motor.stiffness = 0.0;
                motor.damping = 0.0;
            });

            return command;
        }();

        return base_command;
    };

    // Damping Command for Digit Robot:
    inline digit_interface::msg::DigitCommand damping_command() {
        auto command = robot::digit::utilities::initialize_command();

        command.apply_command = true;
        command.fallback_opmode = static_cast<int32_t>(robot::digit::constants::OpMode::Damping);

        std::ranges::for_each(command.motors, [](auto& motor) {
            motor.damping = 5.0;
        });

        return command;
    };


}  // namespace robot::digit::utilities