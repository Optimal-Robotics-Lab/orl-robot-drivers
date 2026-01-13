#pragma once

#include <array>
#include <cstdint>

#include "src/digit/lib/utils/constants.h"


namespace robot::digit::containers {

    namespace constants = robot::digit::constants;

    struct MotorCommand {
        double position{0};
        double velocity{0};
        double torque{0};
        double stiffness{0};
        double damping{0};
    };

    struct DigitCommand {
        std::array<MotorCommand, constants::num_motors> motors{};
        std::int32_t fallback_opmode{0};
        bool apply_command{false};
    };

    struct BaseState {
        std::array<double, 3> translation{0, 0, 0};
        std::array<double, 4> orientation{1, 0, 0, 0};
        std::array<double, 3> linear_velocity{0, 0, 0};
        std::array<double, 3> angular_velocity{0, 0, 0};
    };

    struct ImuState {
        std::array<double, 4> orientation{1, 0, 0, 0};
        std::array<double, 3> angular_velocity{0, 0, 0};
        std::array<double, 3> linear_acceleration{0, 0, 0};
        std::array<double, 3> magnetic_field{0, 0, 0};
    };

    struct MotorState {
        std::array<double, constants::num_motors> position{};
        std::array<double, constants::num_motors> velocity{};
        std::array<double, constants::num_motors> torque{};
    };

    struct JointState {
        std::array<double, constants::num_joints> position{};
        std::array<double, constants::num_joints> velocity{};
    };

    struct DigitState {
        double time{0};
        BaseState base{};
        ImuState imu{};
        MotorState motor{};
        JointState joint{};
        std::int16_t battery_charge{0};
        bool error{false};
    };

}  // namespace robot::digit::containers