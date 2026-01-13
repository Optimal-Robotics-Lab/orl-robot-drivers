#include <thread>
#include <chrono>
#include <optional>
#include <format>
#include <string>
#include <iostream>

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

    // Test get_limits:
    auto limits = node->get_limits();
    if (!limits) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get motor limits from LowLevelApiDriver.");
        return -1;
    }

    for (size_t i = 0; i < robot::digit::constants::num_motors; ++i) {
        const auto motor_name = robot::digit::constants::to_string(
            static_cast<robot::digit::constants::Motors>(i)
        );
        std::cout << std::format(
            "Motor {}: Torque Limit = {:.2f} N-m, Damping Limit = {:.2f} N-m/(rad/s), Velocity Limit = {:.2f} rad/s\n",
            motor_name,
            limits->torque_limit[i],
            limits->damping_limit[i],
            limits->velocity_limit[i]
        );
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    rclcpp::shutdown();

    return 0;
}