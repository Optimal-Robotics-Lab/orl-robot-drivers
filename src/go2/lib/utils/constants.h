#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <numbers>

#include "Eigen/Dense"


namespace robot::go2::constants {

    // Joint identification enum:
    enum class Joint : std::uint8_t {
        FRONT_RIGHT_HIP,
        FRONT_RIGHT_THIGH,
        FRONT_RIGHT_CALF,
        FRONT_LEFT_HIP,
        FRONT_LEFT_THIGH,
        FRONT_LEFT_CALF,
        HIND_RIGHT_HIP,
        HIND_RIGHT_THIGH,
        HIND_RIGHT_CALF,
        HIND_LEFT_HIP,
        HIND_LEFT_THIGH,
        HIND_LEFT_CALF,
        COUNT
    };

    // Compile-time constant for the number of joints based on the Joint enum:
    constexpr std::size_t num_joints = static_cast<std::size_t>(Joint::COUNT);
    constexpr std::size_t num_actions = 2 * num_joints;

    // Joint Name to Joint ID compile-time mapping:
    constexpr std::array<std::pair<Joint, std::string_view>, num_joints> joint_definitions = {{
        {Joint::FRONT_RIGHT_HIP, "FRONT_RIGHT_HIP"},
        {Joint::FRONT_RIGHT_THIGH, "FRONT_RIGHT_THIGH"},
        {Joint::FRONT_RIGHT_CALF, "FRONT_RIGHT_CALF"},
        {Joint::FRONT_LEFT_HIP, "FRONT_LEFT_HIP"},
        {Joint::FRONT_LEFT_THIGH, "FRONT_LEFT_THIGH"},
        {Joint::FRONT_LEFT_CALF, "FRONT_LEFT_CALF"},
        {Joint::HIND_RIGHT_HIP, "HIND_RIGHT_HIP"},
        {Joint::HIND_RIGHT_THIGH, "HIND_RIGHT_THIGH"},
        {Joint::HIND_RIGHT_CALF, "HIND_RIGHT_CALF"},
        {Joint::HIND_LEFT_HIP, "HIND_LEFT_HIP"},
        {Joint::HIND_LEFT_THIGH, "HIND_LEFT_THIGH"},
        {Joint::HIND_LEFT_CALF, "HIND_LEFT_CALF"},
    }};

    // Convert a Joint enum value to its string representation.
    constexpr std::string_view to_string(Joint j) {
        if (static_cast<std::size_t>(j) < num_joints) {
            return joint_definitions[static_cast<std::size_t>(j)].second;
        }
        return "UNKNOWN";
    };

    // Joint Control Mode enum:
    enum class ControlMode : std::uint8_t {
        DISABLE,
        ENABLE,
    };

    // High Level Control Mode enum:
    enum class HighLevelControlMode : std::uint8_t {
        DEFAULT,
        DAMPING,
        POLICY,
        DISABLE,
    };

    // Default values for the Go2 robot:
    constexpr std::array<float, num_joints> default_position = { 
        // Front Right:
        0.0, 0.9, -1.8,
        // Front Left:
        0.0, 0.9, -1.8,
        // Hind Right:
        0.0, 0.9, -1.8,
        // Hind Left:
        0.0, 0.9, -1.8
    };
    
    constexpr std::array<float, num_actions> default_setpoints = { 
        // Front Right:
        0.0, 0.9, -1.8,
        // Front Left:
        0.0, 0.9, -1.8,
        // Hind Right:
        0.0, 0.9, -1.8,
        // Hind Left:
        0.0, 0.9, -1.8
        // Front Right:
        0.0, 0.0, 0.0,
        // Front Left:
        0.0, 0.0, 0.0,
        // Hind Right:
        0.0, 0.0, 0.0,
        // Hind Left:
        0.0, 0.0, 0.0
    };

    constexpr std::array<float, num_joints> q_lb = {
        -1.0472, -1.5708, -2.7227,
        -1.0472, -1.5708, -2.7227,
        -1.0472, -0.5236, -2.7227,
        -1.0472, -0.5236, -2.7227
    };

    constexpr std::array<float, num_joints> q_ub = {
        1.0472, 3.4097, -0.83776,
        1.0472, 3.4097, -0.83776,
        1.0472, 4.5379, -0.83776,
        1.0472, 4.5379, -0.83776
    };

    constexpr float pi = std::numbers::pi;
    constexpr std::array<float, num_joints> qd_bounds = {
        pi, pi, pi,
        pi, pi, pi,
        pi, pi, pi,
        pi, pi, pi
    };

    constexpr std::array<float, num_joints> torque_bounds = {
        23.7, 23.7, 45.43,
        23.7, 23.7, 45.43,
        23.7, 23.7, 45.43,
        23.7, 23.7, 45.43
    };

    constexpr std::array<float, num_joints> default_kp = {
        35.0, 35.0, 35.0,
        35.0, 35.0, 35.0,
        35.0, 35.0, 35.0,
        35.0, 35.0, 35.0
    };

    constexpr std::array<float, num_joints> default_kd = {
        0.5, 0.5, 0.5,
        0.5, 0.5, 0.5,
        0.5, 0.5, 0.5,
        0.5, 0.5, 0.5
    };

    template<typename T>
    using MotorVector = Eigen::Vector<T, num_joints>;
    using ActionVector = Eigen::Vector<T, num_actions>;

} // namespace robot::go2::constants