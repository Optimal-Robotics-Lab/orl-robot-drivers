#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <numbers>

#include "Eigen/Dense"


namespace robot::h1_2::constants {

    // Joint identification enum:
    enum class Joint : std::uint8_t {
        LEFT_HIP_YAW,
        LEFT_HIP_PITCH,
        LEFT_HIP_ROLL,
        LEFT_KNEE,
        LEFT_ANKLE_PITCH,
        LEFT_ANKLE_ROLL,
        RIGHT_HIP_YAW,
        RIGHT_HIP_PITCH,
        RIGHT_HIP_ROLL,
        RIGHT_KNEE,
        RIGHT_ANKLE_PITCH,
        RIGHT_ANKLE_ROLL,
        WAIST_YAW,
        LEFT_SHOULDER_PITCH,
        LEFT_SHOULDER_ROLL,
        LEFT_SHOULDER_YAW,
        LEFT_ELBOW,
        LEFT_WRIST_ROLL,
        LEFT_WRIST_PITCH,
        LEFT_WRIST_YAW,
        RIGHT_SHOULDER_PITCH,
        RIGHT_SHOULDER_ROLL,
        RIGHT_SHOULDER_YAW,
        RIGHT_ELBOW,
        RIGHT_WRIST_ROLL,
        RIGHT_WRIST_PITCH,
        RIGHT_WRIST_YAW,
        COUNT
    };

    // Compile-time constant for the number of joints based on the Joint enum:
    constexpr std::size_t num_joints = static_cast<std::size_t>(Joint::COUNT);

    // Joint Name to Joint ID compile-time mapping:
    constexpr std::array<std::pair<Joint, std::string_view>, num_joints> joint_definitions = {{
        {Joint::LEFT_HIP_YAW, "LEFT_HIP_YAW"},
        {Joint::LEFT_HIP_PITCH, "LEFT_HIP_PITCH"},
        {Joint::LEFT_HIP_ROLL, "LEFT_HIP_ROLL"},
        {Joint::LEFT_KNEE, "LEFT_KNEE"},
        {Joint::LEFT_ANKLE_PITCH, "LEFT_ANKLE_PITCH"},
        {Joint::LEFT_ANKLE_ROLL, "LEFT_ANKLE_ROLL"},
        {Joint::RIGHT_HIP_YAW, "RIGHT_HIP_YAW"},
        {Joint::RIGHT_HIP_PITCH, "RIGHT_HIP_PITCH"},
        {Joint::RIGHT_HIP_ROLL, "RIGHT_HIP_ROLL"},
        {Joint::RIGHT_KNEE, "RIGHT_KNEE"},
        {Joint::RIGHT_ANKLE_PITCH, "RIGHT_ANKLE_PITCH"},
        {Joint::RIGHT_ANKLE_ROLL, "RIGHT_ANKLE_ROLL"},
        {Joint::WAIST_YAW, "WAIST_YAW"},
        {Joint::LEFT_SHOULDER_PITCH, "LEFT_SHOULDER_PITCH"},
        {Joint::LEFT_SHOULDER_ROLL, "LEFT_SHOULDER_ROLL"},
        {Joint::LEFT_SHOULDER_YAW, "LEFT_SHOULDER_YAW"},
        {Joint::LEFT_ELBOW, "LEFT_ELBOW"},
        {Joint::LEFT_WRIST_ROLL, "LEFT_WRIST_ROLL"},
        {Joint::LEFT_WRIST_PITCH, "LEFT_WRIST_PITCH"},
        {Joint::LEFT_WRIST_YAW, "LEFT_WRIST_YAW"},
        {Joint::RIGHT_SHOULDER_PITCH, "RIGHT_SHOULDER_PITCH"},
        {Joint::RIGHT_SHOULDER_ROLL, "RIGHT_SHOULDER_ROLL"},
        {Joint::RIGHT_SHOULDER_YAW, "RIGHT_SHOULDER_YAW"},
        {Joint::RIGHT_ELBOW, "RIGHT_ELBOW"},
        {Joint::RIGHT_WRIST_ROLL, "RIGHT_WRIST_ROLL"},
        {Joint::RIGHT_WRIST_PITCH, "RIGHT_WRIST_PITCH"},
        {Joint::RIGHT_WRIST_YAW, "RIGHT_WRIST_YAW"},
    }};

    // Convert a Joint enum value to its string representation.
    constexpr std::string_view to_string(Joint j) {
        if (static_cast<std::size_t>(j) < num_joints) {
            return joint_definitions[static_cast<std::size_t>(j)].second;
        }
        return "UNKNOWN";
    };

    // Ankle control type enum: 
    // N.B.: Only PITCH_ROLL is currently supported
    enum class AnkleControlType : std::uint8_t {
        PITCH_ROLL,
        A_B,
    };

    // Joint Control Mode enum:
    enum class ControlMode : std::uint8_t {
        DISABLE,
        ENABLE,
    };

    // Mode Machine enum: Some how correlates to the arm configuration...
    // Unitree documentation sucks and does not explain this (default value is 6 so thats what we will use)
    enum class ModeMachine : std::uint8_t {
        NONDEFAULT = 4,
        DEFAULT = 6,
    };

    // High Level Control Mode enum:
    enum class HighLevelControlMode : std::uint8_t {
        DEFAULT,
        HOLD_POSITION,
        DAMPING,
        POLICY,
        DISABLE,
    };

    // Default values for the H1-2 robot:
    constexpr std::array<float, num_joints> default_position = { 
        // Left Leg:
        0.0, -0.4, 0.0,
        0.86,
        -0.45, 0.0,
        // Right Leg:
        0.0, -0.4, 0.0,
        0.86,
        -0.45, 0.0,
        // Torso:
        0.0,
        // Left Arm:
        0.0, 0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        // Right Arm:
        0.0, 0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0
     };

    constexpr std::array<float, num_joints> q_lb = {
        -0.43, -3.14, -0.43, -0.12, -0.897334, -0.261799,
        -0.43, -3.14, -3.14, -0.12, -0.897334, -0.261799,
        -2.35,
        -3.14, -0.38, -2.66, -0.95, -3.01, -0.4625, -1.27,
        -3.14, -3.4, -3.01, -0.95, -2.75, -0.4625, -1.27
    };

    constexpr std::array<float, num_joints> q_ub = {
        0.43, 2.5, 3.14, 2.19, 0.523598, 0.261799,
        0.43, 2.5, 0.43, 2.19, 0.523598, 0.261799,
        2.35,
        1.57, 3.4, 3.01, 3.18, 2.75, 0.4625, 1.27,
        1.57, 0.38, 2.66, 3.18, 3.01, 0.4625, 1.27
    };

    constexpr float pi = std::numbers::pi;
    constexpr std::array<float, num_joints> qd_bounds = {
        pi, pi, pi, pi, pi, pi,
        pi, pi, pi, pi, pi, pi,
        pi,
        pi, pi, pi, pi, pi, pi, pi,
        pi, pi, pi, pi, pi, pi, pi
    };

    constexpr std::array<float, num_joints> torque_bounds = {
        200.0, 200.0, 200.0, 300.0, 60.0, 40.0,
        200.0, 200.0, 200.0, 300.0, 60.0, 40.0,
        200.0,
        40.0, 40.0, 18.0, 18.0, 19.0, 19.0, 19.0,
        40.0, 40.0, 18.0, 18.0, 19.0, 19.0, 19.0
    };

    constexpr std::array<float, num_joints> default_kp = {
        175.0, 175.0, 175.0, 300.0, 150.0, 150.0,
        175.0, 175.0, 175.0, 300.0, 150.0, 150.0,
        200.0,
        30.0, 30.0, 13.5, 13.5, 14.25, 14.25, 14.25,
        30.0, 30.0, 13.5, 13.5, 14.25, 14.25, 14.25,
    };

    constexpr std::array<float, num_joints> default_kd = {
        2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
        2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
        2.0,
        2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
        2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0
    };

    template<typename T>
    using MotorVector = Eigen::Vector<T, num_joints>;

} // namespace robot::h1_2::constants