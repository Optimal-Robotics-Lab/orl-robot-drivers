#pragma once

#include <array>
#include <cstdint>

#include "src/h1_2/lib/utils/constants.h"
#include "src/h1_2/msgs/unitree_hg_msgs.h"


namespace robot::h1_2::utilities {

    // Create a default position command for H1-2 Robot:
    inline unitree_hg::msg::LowCmd default_position_command() {
        auto command = unitree_hg::msg::LowCmd();

        // Header Fields: (Only support AnkleControlType::PITCH_ROLL and ModeMachine::DEFAULT = 6)
        command.mode_pr = static_cast<uint8_t>(h1_2::constants::AnkleControlType::PITCH_ROLL);
        command.mode_machine = static_cast<uint8_t>(h1_2::constants::ModeMachine::DEFAULT);

        for (size_t i = 0; i < h1_2::constants::num_joints; ++i) {
            auto& motor_command = command.motor_cmd[i];

            // Set Header Fields:
            motor_command.mode = static_cast<uint8_t>(h1_2::constants::ControlMode::ENABLE);

            // Set Setpoints and Gains:
            motor_command.q = h1_2::constants::default_position[i];
            motor_command.dq = 0.0;
            motor_command.tau = 0.0;
            motor_command.kp = 0.0;
            motor_command.kd = 0.0;
        }

        return command;
    };

    // Damping Command for H1-2 Robot:
    inline unitree_hg::msg::LowCmd damping_command() {
        auto command = unitree_hg::msg::LowCmd();

        // Header Fields: (Only support AnkleControlType::PITCH_ROLL and ModeMachine::DEFAULT = 6)
        command.mode_pr = static_cast<uint8_t>(h1_2::constants::AnkleControlType::PITCH_ROLL);
        command.mode_machine = static_cast<uint8_t>(h1_2::constants::ModeMachine::DEFAULT);

        for (size_t i = 0; i < h1_2::constants::num_joints; ++i) {
            auto& motor_command = command.motor_cmd[i];

            // Set Header Fields:
            motor_command.mode = static_cast<uint8_t>(h1_2::constants::ControlMode::ENABLE);

            // Set Setpoints and Gains:
            motor_command.q = h1_2::constants::default_position[i];
            motor_command.dq = 0.0;
            motor_command.tau = 0.0;
            motor_command.kp = 0.0;
            motor_command.kd = 5.0;
        }

        return command;
    };

    // Hold current position command for H1-2 Robot:
    inline unitree_hg::msg::LowCmd hold_position_command(
        const H1_2_State& state,
        const std::array<float, h1_2::constants::num_joints>& kp = { 0 },
        const std::array<float, h1_2::constants::num_joints>& kd = { 0 }
    ) {
        auto command = unitree_hg::msg::LowCmd();

        // Header Fields: (Only support AnkleControlType::PITCH_ROLL and ModeMachine::DEFAULT = 6)
        command.mode_pr = static_cast<uint8_t>(h1_2::constants::AnkleControlType::PITCH_ROLL);
        command.mode_machine = static_cast<uint8_t>(h1_2::constants::ModeMachine::DEFAULT);

        for (size_t i = 0; i < h1_2::constants::num_joints; ++i) {
            auto& motor_command = command.motor_cmd[i];

            // Set Header Fields:
            motor_command.mode = static_cast<uint8_t>(h1_2::constants::ControlMode::ENABLE);

            // Set Setpoints and Gains:
            motor_command.q = state.motor_state[i].q;
            motor_command.dq = 0.0;
            motor_command.tau = 0.0;
            motor_command.kp = kp[i];
            motor_command.kd = kd[i];
        }

        return command;
    };

    // Disable Command for H1-2 Robot:
    inline unitree_hg::msg::LowCmd disable_command() {
        auto command = unitree_hg::msg::LowCmd();

        // Header Fields: (Only support AnkleControlType::PITCH_ROLL and ModeMachine::DEFAULT = 6)
        command.mode_pr = static_cast<uint8_t>(h1_2::constants::AnkleControlType::PITCH_ROLL);
        command.mode_machine = static_cast<uint8_t>(h1_2::constants::ModeMachine::DEFAULT);

        for (size_t i = 0; i < h1_2::constants::num_joints; ++i) {
            auto& motor_command = command.motor_cmd[i];

            // Set Header Fields:
            motor_command.mode = static_cast<uint8_t>(h1_2::constants::ControlMode::DISABLE);

            // Set Setpoints and Gains:
            motor_command.q = h1_2::constants::default_position[i];
            motor_command.dq = 0.0;
            motor_command.tau = 0.0;
            motor_command.kp = 0.0;
            motor_command.kd = 5.0;
        }

        return command;
    };

} // namespace robot::h1_2::utilities