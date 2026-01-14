#include <cstddef>
#include <thread>
#include <chrono>
#include <optional>
#include <string>
#include <array>
#include <print>

#include "rclcpp/rclcpp.hpp"
#include "absl/status/status.h"
#include "absl/log/absl_check.h"

#include "src/digit/lib/driver/lowlevelapi_driver.h"
#include "src/digit/lib/utils/constants.h"
#include "src/digit/lib/utils/utilities.h"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LowLevelApiDriver>();

    absl::Status status = node->initialize();
    ABSL_CHECK(status.ok()) << "Failed to initialize LowLevelApiDriver: " << status.message();

    std::jthread ros_spinner([node]() {
        rclcpp::spin(node);
    });

    // Allow some time for initialization:
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Test get_limits:
    auto state = node->get_state();
    if (!state) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get state from LowLevelApi Driver.");
        return -1;
    }

    std::println(
        "Digit State:\n"
        "\t Time: {} \t Error: {} \t Battery Charge: {}",
        state->time,
        state->error,
        state->battery_charge
    );

    std::println(
        "Base Frame:\n"
        "\t Translation: {} \n \t Orientation: {} \n \t Linear Velocity: {} \n \t Angular Velocity: {}",
        state->base.translation,
        state->base.orientation,
        state->base.linear_velocity,
        state->base.angular_velocity
    );

    std::println(
        "IMU Frame:\n"
        "\t Orientation: {} \n \t Angular Velocity: {} \n \t Linear Acceleration: {} \n \t Magnetic Field: {}",
        state->imu.orientation,
        state->imu.angular_velocity,
        state->imu.linear_acceleration,
        state->imu.magnetic_field
    );

    for (std::size_t i = 0; i < robot::digit::constants::num_motors; ++i) {
        const auto motor_name = robot::digit::constants::to_string(
            static_cast<robot::digit::constants::Motors>(i)
        );
        std::println(
            "Motor {}:\n"
            "\t Position: {:.2f} rad \n \t Velocity: {:.2f} rad/s \n \t Torque: {:.2f} N-m",
            motor_name,
            state->motor[i].position,
            state->motor[i].velocity,
            state->motor[i].torque
        );
    }

    for (std::size_t i = 0; i < robot::digit::constants::num_joints; ++i) {
        const auto joint_name = robot::digit::constants::to_string(
            static_cast<robot::digit::constants::Joints>(i)
        );
        std::println(
            "Joint {}:\n"
            "\t Position: {:.2f} rad \n \t Velocity: {:.2f} rad/s",
            joint_name,
            state->joint[i].position,
            state->joint[i].velocity
        );
    }

    // Allow some time for any remaining operations to complete before shutdown.
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::shutdown();

    return 0;
}