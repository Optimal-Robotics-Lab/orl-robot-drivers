#pragma once

#include <array>
#include <cstdint>

#include "src/go2/lib/utils/constants.h"
#include "src/go2/msgs/unitree_go_msgs.h"


namespace robot::go2::utilities {

    // Prototype for the LowCmd:
    inline const unitree_go::msg::LowCmd& initialize_command() {
        static const unitree_go::msg::LowCmd base_command = [] {
            unitree_go::msg::LowCmd command{};

            command.head[0] = 0xFE;
            command.head[1] = 0xEF;
            command.level_flag = 0xFF;
            command.gpio = 0;

            for (int i = 0; i < 20; ++i) {
                command.motor_cmd[i].mode = 0x01;
                command.motor_cmd[i].q = 2.146E+9f;
                command.motor_cmd[i].kp = 0;
                command.motor_cmd[i].dq = 16000.0f;
                command.motor_cmd[i].kd = 0;
                command.motor_cmd[i].tau = 0;
            }

            return command;
        }();

        return base_command;
    }

    // Create a default position command for Go2 Robot:
    inline unitree_go::msg::LowCmd default_position_command() {
        auto command = robot::go2::utilities::initialize_command();

        for (size_t i = 0; i < go2::constants::num_joints; ++i) {
            auto& motor_command = command.motor_cmd[i];

            // Set Header Fields:
            motor_command.mode = static_cast<uint8_t>(go2::constants::ControlMode::ENABLE);

            // Set Setpoints and Gains:
            motor_command.q = go2::constants::default_position[i];
            motor_command.dq = 0.0;
            motor_command.tau = 0.0;
            motor_command.kp = 0.0;
            motor_command.kd = 0.0;
        }

        return command;
    }

    // Damping Command for Go2 Robot:
    inline unitree_go::msg::LowCmd damping_command() {
        auto command = robot::go2::utilities::initialize_command();

        for (size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
            auto& motor_command = command.motor_cmd[i];

            // Set Header Fields:
            motor_command.mode = static_cast<uint8_t>(robot::go2::constants::ControlMode::ENABLE);

            // Set Setpoints and Gains:
            motor_command.q = robot::go2::constants::default_position[i];
            motor_command.dq = 0.0;
            motor_command.tau = 0.0;
            motor_command.kp = 0.0;
            motor_command.kd = 5.0;
        }

        return command;
    };

    // Hold current position command for Go2 Robot:
    inline unitree_go::msg::LowCmd hold_position_command(
        const Go2State& state,
        const std::array<float, robot::go2::constants::num_joints>& kp = { 0 },
        const std::array<float, robot::go2::constants::num_joints>& kd = { 0 }
    ) {
        auto command = robot::go2::utilities::initialize_command();

        for (size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
            auto& motor_command = command.motor_cmd[i];

            // Set Header Fields:
            motor_command.mode = static_cast<uint8_t>(robot::go2::constants::ControlMode::ENABLE);

            // Set Setpoints and Gains:
            motor_command.q = state.motor_state[i].q;
            motor_command.dq = 0.0;
            motor_command.tau = 0.0;
            motor_command.kp = kp[i];
            motor_command.kd = kd[i];
        }

        return command;
    };

    // Disable Command for Go2 Robot:
    inline unitree_go::msg::LowCmd disable_command() {
        auto command = robot::go2::utilities::initialize_command();

        for (size_t i = 0; i < go2::constants::num_joints; ++i) {
            auto& motor_command = command.motor_cmd[i];

            // Set Header Fields:
            motor_command.mode = static_cast<uint8_t>(go2::constants::ControlMode::DISABLE);

            // Set Setpoints and Gains:
            motor_command.q = go2::constants::default_position[i];
            motor_command.dq = 0.0;
            motor_command.tau = 0.0;
            motor_command.kp = 0.0;
            motor_command.kd = 5.0;
        }

        return command;
    };

} // namespace robot::go2::utilities