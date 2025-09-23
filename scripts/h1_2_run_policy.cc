#include <iostream>
#include <filesystem>
#include <algorithm>
#include <thread>
#include <chrono>
#include <bit>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"

#include "absl/status/status.h"
#include "absl/log/absl_check.h"
#include "rules_cc/cc/runfiles/runfiles.h"

#include "src/h1_2/lib/driver/h1_2_driver.h"
#include "src/h1_2/lib/driver/onnx_driver.h"

#include "src/h1_2/lib/utils/constants.h"
#include "src/h1_2/lib/utils/containers.h"
#include "src/h1_2/lib/utils/utilities.h"
#include "src/h1_2/msgs/unitree_hg_msgs.h"

using namespace robot;
using rules_cc::cc::runfiles::Runfiles;


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    std::string error;
    std::unique_ptr<Runfiles> runfiles(
        Runfiles::Create(argv[0], BAZEL_CURRENT_REPOSITORY, &error)
    );

    std::filesystem::path onnx_model_path = 
        runfiles->Rlocation("orl-robot-drivers/onnx_models/dainty-cloud-170.onnx");
    
    absl::Status result;
    auto RobotDriver = std::make_shared<H1_2_Driver>();

    // Initialize the thread:
    result.Update(RobotDriver->initialize_thread());
    ABSL_CHECK(result.ok()) << result.message();

    // Sleep for a while to allow the thread to spin up:
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Initialize the driver for control:
    result.Update(RobotDriver->initialize());
    ABSL_CHECK(result.ok()) << result.message();

    // Create a default position command and update command:
    auto command = h1_2::utilities::default_position_command();
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
    result.Update(PolicyDriver->set_control_mode(h1_2::constants::HighLevelControlMode::DEFAULT));
    result.Update(PolicyDriver->set_command(constants::Vector3<float>(0.0, 0.0, 0.0)));
    result.Update(PolicyDriver->initialize());
    ABSL_CHECK(result.ok()) << result.message();

    // Control Loop:
    std::cout << "Starting Control Loop." << std::endl;
    
    bool is_running = true;
    float master_gain = 0.0f;
    while (is_running) {
        auto start_time = std::chrono::steady_clock::now();

        auto state = RobotDriver->get_state();
        if (!state) {
            std::cerr << "Failed to get the latest state message." << std::endl;
            continue;
        }
        else {
            const auto& byte_array = state->wireless_remote;
            const h1_2::containers::WirelessRemote controller = 
                std::bit_cast<h1_2::containers::WirelessRemote>(byte_array);

            if (controller.buttons.is_A_pressed()) {
                std::cout << "Setting control mode to POLICY." << std::endl;
                std::ignore = PolicyDriver->set_control_mode(h1_2::constants::HighLevelControlMode::POLICY);
            }

            // TODO(jeh15): Maybe setting master gain to 0.0 should be the default when switching to DAMPING.
            if (controller.buttons.is_B_pressed()) {
                std::cout << "Setting control mode to DAMPING." << std::endl;
                result.Update(PolicyDriver->set_control_mode(h1_2::constants::HighLevelControlMode::DAMPING));
                master_gain = 0.0f;
                ABSL_CHECK(result.ok()) << result.message();
            }

            if (controller.buttons.is_select_pressed()) {
                std::cout << "Select button pressed, stopping control loop." << std::endl;
                is_running = false;
                result.Update(PolicyDriver->set_control_mode(h1_2::constants::HighLevelControlMode::DAMPING));
                master_gain = 0.0f;
                ABSL_CHECK(result.ok()) << result.message();
            }
            
            if (controller.buttons.is_R1_pressed()) {
                master_gain += 0.01f;
                master_gain = std::clamp(master_gain, 0.0f, 1.0f);
                std::cout << "Increasing master gain to: " << master_gain << std::endl;
                result.Update(PolicyDriver->set_master_gain(master_gain));
                ABSL_CHECK(result.ok()) << result.message();
            }

            if (controller.buttons.is_L1_pressed()) {
                master_gain -= 0.01f;
                master_gain = std::clamp(master_gain, 0.0f, 1.0f);
                std::cout << "Decreasing master gain to: " << master_gain << std::endl;
                result.Update(PolicyDriver->set_master_gain(master_gain));
                ABSL_CHECK(result.ok()) << result.message();
            }
            float joystick_scale = 0.75f;
            float x = joystick_scale * controller.get_left_stick_y();
            float y = -1.0f * joystick_scale * controller.get_left_stick_x();
            float z = -1.0f * joystick_scale * controller.get_right_stick_x();

            if (PolicyDriver->get_control_mode() == h1_2::constants::HighLevelControlMode::POLICY) {
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

    rclcpp::shutdown();
    return 0;
}
