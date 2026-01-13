#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <numbers>


namespace robot::digit::constants {
    
    // Digit Actuated Joint Identification Enum:
    enum class Motors : std::uint8_t {
        LEFT_HIP_ROLL,
        LEFT_HIP_YAW,
        LEFT_HIP_PITCH,
        LEFT_KNEE,
        LEFT_TOE_A,
        LEFT_TOE_B,
        RIGHT_HIP_ROLL,
        RIGHT_HIP_YAW,
        RIGHT_HIP_PITCH,
        RIGHT_KNEE,
        RIGHT_TOE_A,
        RIGHT_TOE_B,
        LEFT_SHOULDER_ROLL,
        LEFT_SHOULDER_PITCH,
        LEFT_SHOULDER_YAW,
        LEFT_ELBOW,
        RIGHT_SHOULDER_ROLL,
        RIGHT_SHOULDER_PITCH,
        RIGHT_SHOULDER_YAW,
        RIGHT_ELBOW,
        COUNT
    };

    constexpr std::size_t num_motors = static_cast<std::size_t>(Motors::COUNT);

    constexpr std::array<std::pair<Motors, std::string_view>, num_motors> motor_definitions = {{
        {Motors::LEFT_HIP_ROLL, "LEFT_HIP_ROLL"},
        {Motors::LEFT_HIP_YAW, "LEFT_HIP_YAW"},
        {Motors::LEFT_HIP_PITCH, "LEFT_HIP_PITCH"},
        {Motors::LEFT_KNEE, "LEFT_KNEE"},
        {Motors::LEFT_TOE_A, "LEFT_TOE_A"},
        {Motors::LEFT_TOE_B, "LEFT_TOE_B"},
        {Motors::RIGHT_HIP_ROLL, "RIGHT_HIP_ROLL"},
        {Motors::RIGHT_HIP_YAW, "RIGHT_HIP_YAW"},
        {Motors::RIGHT_HIP_PITCH, "RIGHT_HIP_PITCH"},
        {Motors::RIGHT_KNEE, "RIGHT_KNEE"},
        {Motors::RIGHT_TOE_A, "RIGHT_TOE_A"},
        {Motors::RIGHT_TOE_B, "RIGHT_TOE_B"},
        {Motors::LEFT_SHOULDER_ROLL, "LEFT_SHOULDER_ROLL"},
        {Motors::LEFT_SHOULDER_PITCH, "LEFT_SHOULDER_PITCH"},
        {Motors::LEFT_SHOULDER_YAW, "LEFT_SHOULDER_YAW"},
        {Motors::LEFT_ELBOW, "LEFT_ELBOW"},
        {Motors::RIGHT_SHOULDER_ROLL, "RIGHT_SHOULDER_ROLL"},
        {Motors::RIGHT_SHOULDER_PITCH, "RIGHT_SHOULDER_PITCH"},
        {Motors::RIGHT_SHOULDER_YAW, "RIGHT_SHOULDER_YAW"},
        {Motors::RIGHT_ELBOW, "RIGHT_ELBOW"},
    }};

    // Convert a Motors enum value to its string representation.
    constexpr std::string_view to_string(Motors m) {
        if (static_cast<std::size_t>(m) < num_motors) {
            return motor_definitions[static_cast<std::size_t>(m)].second;
        }
        return "UNKNOWN";
    };

    // Digit Unactuated Joint Identification Enum:
    enum class Joints : std::uint8_t {
        LEFT_SHIN,
        LEFT_TARSUS,
        LEFT_TOE_PITCH,
        LEFT_TOE_ROLL,
        LEFT_HEEL_SPRING,
        RIGHT_SHIN,
        RIGHT_TARSUS,
        RIGHT_TOE_PITCH,
        RIGHT_TOE_ROLL,
        RIGHT_HEEL_SPRING,
        COUNT
    };

    constexpr std::size_t num_joints = static_cast<std::size_t>(Joints::COUNT);

    constexpr std::array<std::pair<Joints, std::string_view>, num_joints> joint_definitions = {{
        {Joints::LEFT_SHIN, "LEFT_SHIN"},
        {Joints::LEFT_TARSUS, "LEFT_TARSUS"},
        {Joints::LEFT_TOE_PITCH, "LEFT_TOE_PITCH"},
        {Joints::LEFT_TOE_ROLL, "LEFT_TOE_ROLL"},
        {Joints::LEFT_HEEL_SPRING, "LEFT_HEEL_SPRING"},
        {Joints::RIGHT_SHIN, "RIGHT_SHIN"},
        {Joints::RIGHT_TARSUS, "RIGHT_TARSUS"},
        {Joints::RIGHT_TOE_PITCH, "RIGHT_TOE_PITCH"},
        {Joints::RIGHT_TOE_ROLL, "RIGHT_TOE_ROLL"},
        {Joints::RIGHT_HEEL_SPRING, "RIGHT_HEEL_SPRING"},
    }};

    // Convert a Joints enum value to its string representation.
    constexpr std::string_view to_string(Joints m) {
        if (static_cast<std::size_t>(m) < num_joints) {
            return joint_definitions[static_cast<std::size_t>(m)].second;
        }
        return "UNKNOWN";
    };

    // Digit Low-level API Fallback Operation Mode Enum:
    enum class OpMode : std::int32_t {
        Disabled = 0,
        Damping = 1,
        Locomotion = 2,
    };

    constexpr std::array<double, num_motors> motor_position_limits_lower = {
        -1.0472, -0.698132, -1.0472,
        -1.39626,
        -0.807659838, -0.8009630097,
        -1.0472, -0.698132, -1.5708,
        -1.019272,
        -0.7850752775, -0.7949555864,
        -1.309, -2.53073, -1.74533, -1.35263,
        -1.309, -2.53073, -1.74533, -1.35263
    };

    constexpr std::array<double, num_motors> motor_position_limits_upper = {
        1.0472, 0.698132, 1.5708,
        1.019272,
        0.7850752775, 0.7949555864,
        1.0472, 0.698132, 1.0472,
        1.39626,
        0.807659838, 0.8009630097,
        1.309, 2.53073, 1.74533, 1.35263,
        1.309, 2.53073, 1.74533, 1.35263
    };

    constexpr std::array<double, num_joints> joint_position_limits_lower = {
        -1.57079632679, -0.8779006,
        -0.767945, -0.645772, 
        -0.10472,
        -1.57079632679, -1.249656,
        -0.593412, -0.575959, 
        -0.10472,
    };

    constexpr std::array<double, num_joints> joint_position_limits_upper = {
        1.57079632679, 1.249656,
        0.593412, 0.575959, 
        0.10472,
        1.57079632679, 0.8779006,
        0.767945, 0.645772, 
        0.10472,
    };

    constexpr std::array<double, num_motors> motor_velocity_limits = {
        2 * std::numbers::pi, 2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi,
        2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi, 2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi,
        2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi, 2 * std::numbers::pi, 2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi, 2 * std::numbers::pi, 2 * std::numbers::pi, 2 * std::numbers::pi
    };

    constexpr std::array<double, num_joints> joint_velocity_limits = {
        2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi, 
        2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi, 2 * std::numbers::pi,
        2 * std::numbers::pi
    };

    constexpr std::array<double, num_motors> motor_torque_limits = {
        80 * 1.583530725, 50 * 1.583530725, 16 * 13.557993625,
        16 * 14.457309375,
        50 * 0.8395188400000001, 50 * 0.8395188400000001,
        80 * 1.583530725, 50 * 1.583530725, 16 * 13.557993625,
        16 * 14.457309375,
        50 * 0.8395188400000001, 50 * 0.8395188400000001,
        80 * 1.583530725, 80 * 1.583530725, 50 * 1.583530725, 80 * 1.583530725,
        80 * 1.583530725, 80 * 1.583530725, 50 * 1.583530725, 80 * 1.583530725
    };

} // namespace robot::digit::constants