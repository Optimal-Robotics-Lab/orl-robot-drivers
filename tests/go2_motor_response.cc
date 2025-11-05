#include <iostream>
#include <filesystem>
#include <algorithm>
#include <thread>
#include <chrono>
#include <bit>
#include <cstdint>
#include <csignal>
#include <stop_token> 

#include "rclcpp/rclcpp.hpp"

#include "absl/status/status.h"
#include "absl/log/absl_check.h"

#include "src/go2/lib/driver/wireless_controller_driver.h"
#include "src/go2/lib/driver/go2_driver.h"

#include "src/go2/lib/utils/constants.h"
#include "src/go2/lib/utils/containers.h"
#include "src/go2/lib/utils/utilities.h"
#include "src/go2/msgs/unitree_go_msgs.h"

using namespace robot;
using rules_cc::cc::runfiles::Runfiles;

std::stop_source global_stop_source;


void signal_handler(int signal) {
    if (signal == SIGHUP)
        std::cout << "\nCaught SIGHUP (SSH disconnect). Initiating graceful shutdown..." << std::endl;
    else if (signal == SIGINT)
        std::cout << "\nCaught SIGINT (Ctrl+C). Initiating graceful shutdown..." << std::endl;
    else if (signal == SIGTERM)
        std::cout << "\nCaught SIGTERM. Initiating graceful shutdown..." << std::endl;

    global_stop_source.request_stop();
}


int main(int argc, char * argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGHUP, signal_handler);

    std::stop_token stop_token = global_stop_source.get_token();

    rclcpp::init(argc, argv);

    // Default Node Options:
    rclcpp::NodeOptions options;
    
    absl::Status result;
    auto ControllerDriver = std::make_shared<WirelessControllerDriver>();
    auto RobotDriver = std::make_shared<Go2Driver>();

    // Initialize the thread:
    result.Update(ControllerDriver->initialize_thread());
    result.Update(RobotDriver->initialize_thread());
    ABSL_CHECK(result.ok()) << result.message();

    // Sleep for a while to allow the thread to spin up:
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Initialize the driver for control:
    result.Update(RobotDriver->initialize());
    ABSL_CHECK(result.ok()) << result.message();

    // Create a default position command and update command:
    auto command = go2::utilities::default_position_command();
    result.Update(RobotDriver->update_command(command));
    ABSL_CHECK(result.ok()) << result.message();

    // Sleep for a while to allow the thread to spin up:
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Control Loop:
    std::cout << "Starting Control Loop." << std::endl;


    std::array<float, robot::go2::constants::num_joints> collapsed_position = {
        // Front Right:
        -0.7f, 1.5f, -2.5f,
        // Front Left:
        0.0f, 0.9f, -1.8f,
        // Hind Right:
        0.0f, 0.9f, -1.8f,
        // Hind Left:
        0.0f, 0.9f, -1.8f
    };
    
    bool is_running = true;
    float master_gain = 0.0f;
    while (!stop_token.stop_requested() && is_running) {
        auto start_time = std::chrono::steady_clock::now();

        auto state = RobotDriver->get_state();
        if (!state) {
            std::cerr << "Failed to get the latest state message." << std::endl;
            continue;
        }
        else {
            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::A)) {
                std::cout << "Sending Collapsed Position Command" << std::endl;
                
                // Send Position Command:
                auto command = go2::utilities::default_position_command();
                for (size_t i = 0; i < go2::constants::num_joints; ++i) {
                    command.motor_cmd[i].q = collapsed_position[i];
                    command.motor_cmd[i].kp = master_gain * 35.0f;
                    command.motor_cmd[i].kd = 0.5f;
                }
                result.Update(RobotDriver->update_command(command));

                ABSL_CHECK(result.ok()) << result.message();
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::B)) {
                std::cout << "Sending Default Position Command" << std::endl;
                
                // Send Position Command:
                auto command = go2::utilities::default_position_command();
                for (size_t i = 0; i < go2::constants::num_joints; ++i) {
                    command.motor_cmd[i].kp = master_gain * 35.0f;
                    command.motor_cmd[i].kd = 0.5f;
                }
                result.Update(RobotDriver->update_command(command));

                ABSL_CHECK(result.ok()) << result.message();
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::SELECT)) {
                std::cout << "Select button pressed, stopping control loop." << std::endl;
                is_running = false;
                master_gain = 0.0f;
                auto command = go2::utilities::damping_command();
                result.Update(RobotDriver->update_command(command));
                ABSL_CHECK(result.ok()) << result.message();
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::R1)) {
                master_gain += 0.01f;
                master_gain = std::clamp(master_gain, 0.0f, 1.0f);
                std::cout << "Increasing master gain to: " << master_gain << std::endl;
                ABSL_CHECK(result.ok()) << result.message();
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::L1)) {
                master_gain -= 0.01f;
                master_gain = std::clamp(master_gain, 0.0f, 1.0f);
                std::cout << "Decreasing master gain to: " << master_gain << std::endl;
                ABSL_CHECK(result.ok()) << result.message();
            }

        }

        auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        if (elapsed_time < std::chrono::milliseconds(control_rate_ms))
            std::this_thread::sleep_for(std::chrono::milliseconds(control_rate_ms) - elapsed_time);
        else
            std::cout << "Warning: Loop took longer than expected, skipping sleep." << std::endl;
    }

    std::cout << "Control loop exited. Putting robot into a safe state..." << std::endl;

    // Send Damping Command On Exit:
    auto damping_command = go2::utilities::damping_command();
    result.Update(RobotDriver->update_command(damping_command));

    if (result.ok()) {
        std::cout << "Damping command sent successfully. Robot is shutting down." << std::endl;
    } else {
        std::cerr << "Error sending damping command: " << result.message() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    rclcpp::shutdown();
    return 0;
}
