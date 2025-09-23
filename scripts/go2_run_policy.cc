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
#include "rules_cc/cc/runfiles/runfiles.h"

#include "src/go2/lib/driver/wireless_controller_driver.h"
#include "src/go2/lib/driver/go2_driver.h"
#include "src/go2/lib/driver/onnx_driver.h"

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

    std::string error;
    std::unique_ptr<Runfiles> runfiles(
        Runfiles::Create(argv[0], BAZEL_CURRENT_REPOSITORY, &error)
    );

    std::filesystem::path onnx_model_path = 
        runfiles->Rlocation("robot-drivers-bazel/onnx_models/wild-tree-10.onnx");
    
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

    // Initialize ONNX Driver setting Default HighLevelCommandMode and Command:
    constexpr size_t control_rate_ms = 20;
    auto PolicyDriver = std::make_shared<ONNXDriver>(onnx_model_path, RobotDriver);
    result.Update(PolicyDriver->initialize_thread());

    // Sleep for a while to allow the thread to spin up:
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Set Inital Master Gain, Default HighLevelCommandMode, Command, and Initialize:
    result.Update(PolicyDriver->set_master_gain(0.0f));
    result.Update(PolicyDriver->set_control_mode(go2::constants::HighLevelControlMode::DEFAULT));
    result.Update(PolicyDriver->set_command(constants::Vector3<float>(0.0, 0.0, 0.0)));
    result.Update(PolicyDriver->initialize());
    ABSL_CHECK(result.ok()) << result.message();

    // Control Loop:
    std::cout << "Starting Control Loop." << std::endl;
    
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
                std::cout << "Setting control mode to POLICY." << std::endl;
                result.Update(PolicyDriver->set_control_mode(go2::constants::HighLevelControlMode::POLICY));
                ABSL_CHECK(result.ok()) << result.message();
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::B)) {
                std::cout << "Setting control mode to DAMPING." << std::endl;
                result.Update(PolicyDriver->set_control_mode(go2::constants::HighLevelControlMode::DAMPING));
                master_gain = 0.0f;
                ABSL_CHECK(result.ok()) << result.message();
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::SELECT)) {
                std::cout << "Select button pressed, stopping control loop." << std::endl;
                is_running = false;
                result.Update(PolicyDriver->set_control_mode(go2::constants::HighLevelControlMode::DAMPING));
                master_gain = 0.0f;
                ABSL_CHECK(result.ok()) << result.message();
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::R1)) {
                master_gain += 0.01f;
                master_gain = std::clamp(master_gain, 0.0f, 1.0f);
                std::cout << "Increasing master gain to: " << master_gain << std::endl;
                result.Update(PolicyDriver->set_master_gain(master_gain));
                ABSL_CHECK(result.ok()) << result.message();
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::L1)) {
                master_gain -= 0.01f;
                master_gain = std::clamp(master_gain, 0.0f, 1.0f);
                std::cout << "Decreasing master gain to: " << master_gain << std::endl;
                result.Update(PolicyDriver->set_master_gain(master_gain));
                ABSL_CHECK(result.ok()) << result.message();
            }
            
            float x_scale = 1.5f;
            float y_scale = 1.0f;
            float z_scale = 3.0f;
            float x = x_scale * ControllerDriver->get_left_stick_y();
            float y = -1.0f * y_scale * ControllerDriver->get_left_stick_x();
            float z = -1.0f * z_scale * ControllerDriver->get_right_stick_x();

            if (PolicyDriver->get_control_mode() == go2::constants::HighLevelControlMode::POLICY) {
                result.Update(PolicyDriver->set_command(constants::Vector3<float>(x, y, z)));
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
