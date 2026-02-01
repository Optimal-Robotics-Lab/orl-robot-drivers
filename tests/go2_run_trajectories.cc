#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "absl/status/status.h"
#include "absl/log/absl_check.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

#include "rules_cc/cc/runfiles/runfiles.h"

#include "src/go2/lib/driver/wireless_controller_driver.h"
#include "src/go2/lib/driver/go2_driver.h"

#include "src/go2/lib/utils/constants.h"
#include "src/go2/lib/utils/containers.h"
#include "src/go2/lib/utils/utilities.h"
#include "src/go2/msgs/unitree_go_msgs.h"

using namespace robot;
using rules_cc::cc::runfiles::Runfiles;


ABSL_FLAG(
    std::string, filename, "", "CSV filename under orl-robot-drivers/tests/data/ to process."
);


using PositionSetpoints = std::array<float, go2::constants::num_joints>;
using Trajectory = std::vector<PositionSetpoints>;
using Dataset = std::vector<Trajectory>;

Dataset load_trajectories(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::string line;
    int num_trajectories = 0, num_steps = 0, num_joints = 0;

    // Parse Header
    // Expected format: "SHAPE:{num_trajectories},{num_steps},{num_joints}"
    if (std::getline(file, line)) {
        if (line.find("SHAPE:") == 0) {
            std::string dims = line.substr(6);
            std::replace(dims.begin(), dims.end(), ',', ' ');
            std::stringstream ss(dims);
            ss >> num_trajectories >> num_steps >> num_joints;
        }
        else {
            throw std::runtime_error("Invalid file format: Missing SHAPE header.");
        }
    }

    if (num_joints != robot::go2::constants::num_joints)
        throw std::runtime_error("Mismatch: CSV has " + std::to_string(num_joints) + " joints, but code expects 12.");

    std::cout << "Loading " << num_trajectories << " trajectories with " 
              << num_steps << " steps each..." << std::endl;

    Dataset dataset(num_trajectories); 

    // Read Data
    for (int i = 0; i < num_trajectories; ++i) {
        dataset[i].reserve(num_steps);
        for (int t = 0; t < num_steps; ++t) {
            if (!std::getline(file, line))
                throw std::runtime_error("Unexpected end of file at Trajectory " + std::to_string(i) + " Step " + std::to_string(t));

            std::stringstream lineStream(line);
            std::string cell;
            PositionSetpoints q;

            for (int j = 0; j < robot::go2::constants::num_joints; ++j) {
                if (std::getline(lineStream, cell, ','))
                    q[j] = std::stof(cell);
                else
                    throw std::runtime_error("Row missing column " + std::to_string(j));
            }
            
            dataset[i].push_back(q);
        }
    }

    file.close();
    return dataset;
}


PositionSetpoints interpolate(const PositionSetpoints& start, const PositionSetpoints& end, float alpha) {
    PositionSetpoints result;
    for (std::size_t i = 0; i < result.size(); ++i) {
        result[i] = start[i] * (1.0f - alpha) + end[i] * alpha;
    }
    return result;
}


int main(int argc, char * argv[]) {
    absl::ParseCommandLine(argc, argv);

    rclcpp::init(argc, argv);

    // Default Node Options:
    rclcpp::NodeOptions options;

    std::string error;
    std::unique_ptr<Runfiles> runfiles(
        Runfiles::Create(argv[0], BAZEL_CURRENT_REPOSITORY, &error)
    );

    std::string filename = absl::GetFlag(FLAGS_filename);
    if (filename.empty()) {
        std::cerr << "Error: The --filename flag is required." << std::endl;
        std::cerr << "Usage: " << argv[0] << " --filename=<your-filename>" << std::endl;
        return 1;
    }

    std::string runfile_path = "orl-robot-drivers/tests/data/" + filename;
    std::filesystem::path trajectory_data_path = 
        runfiles->Rlocation(runfile_path);
    
    Dataset trajectories = load_trajectories(trajectory_data_path.string());
    
    absl::Status result;
    auto ControllerDriver = std::make_shared<WirelessControllerDriver>();
    auto RobotDriver = std::make_shared<Go2Driver>();

    // Initialize the thread:
    constexpr std::size_t control_rate_ms = 20;
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

    // Control Loop:
    std::cout << "Starting Control Loop." << std::endl;
    
    bool is_running = true;
    const float ramp_time_s = 3.0f;

    // Control Parameters:
    constexpr std::array<float, go2::constants::num_joints> kp = go2::constants::default_kp;
    constexpr std::array<float, go2::constants::num_joints> kd = go2::constants::default_kd;

    // Setup ROS2 Bag Writer:
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "bags/experiment_data";
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    if (std::filesystem::exists(storage_options.uri)) {
        std::filesystem::remove_all(storage_options.uri);
    }
    
    writer->open(storage_options, converter_options);

    const std::string state_topic = "lowstate";
    const std::string command_topic = "lowcmd";

    writer->create_topic({state_topic, "unitree_go/msg/LowState", "cdr", ""});
    writer->create_topic({command_topic, "unitree_go/msg/LowCmd", "cdr", ""});
    
    // Wait for user to start the test:
    std::cout << "Press 'A' on the controller to start the test." << std::endl;
    while(rclcpp::ok()) {
        if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::A)) {
            std::cout << "Starting Test..." << std::endl;
            break;
        }
        if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::B)) {
            std::cout << "Exit requested by user." << std::endl;
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Iterate over trajectories:
    for (const auto& trajectory : trajectories) {
        if (!is_running) break;

        // Bring Robot to start position of trajectory:
        auto ramp_start_time = std::chrono::steady_clock::now();
        const auto setpoint = trajectory.front();
        auto state = RobotDriver->get_state();
        if (!state) {
            auto damping_command = go2::utilities::damping_command();
            result.Update(RobotDriver->update_command(damping_command));
            std::cerr << "Failed to get the latest state message." << std::endl;
            is_running = false; break;
        }
        std::array<float, go2::constants::num_joints> start_position;
        for (std::size_t i = 0; i < robot::go2::constants::num_joints; ++i)
            start_position[i] = state->motor_state[i].q;
        

        // Ramp to start position:
        while(is_running) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<float> elapsed_time = now - ramp_start_time;
            if (elapsed_time.count() >= ramp_time_s) break;

            auto state = RobotDriver->get_state();
            if (!state) {
                auto damping_command = go2::utilities::damping_command();
                result.Update(RobotDriver->update_command(damping_command));
                std::cerr << "Failed to get the latest state message." << std::endl;
                is_running = false; break;
            }
    
            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::B)) {
                std::cout << "Setting control mode to DAMPING and Terminating." << std::endl;
                auto damping_command = go2::utilities::damping_command();
                result.Update(RobotDriver->update_command(damping_command));
                ABSL_CHECK(result.ok()) << result.message();
                is_running = false; break;
            }

            // Interpolate towards setpoint:
            const float alpha = std::clamp(
                elapsed_time.count() / ramp_time_s, 0.0f, 1.0f
            );
            const auto interpolated_setpoint = interpolate(
                start_position, setpoint, alpha
            );

            // Create and send position command:
            auto position_command = go2::utilities::default_position_command();
            for (std::size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
                auto& motor_command = position_command.motor_cmd[i];
                motor_command.q = interpolated_setpoint[i];
                motor_command.kp = kp[i];
                motor_command.kd = kd[i];
            }
            result.Update(RobotDriver->update_command(position_command));
            ABSL_CHECK(result.ok()) << result.message();

            std::this_thread::sleep_for(std::chrono::milliseconds(control_rate_ms));
        }

        // Iterate over position setpoints in the trajectory:
        for (const auto& setpoint : trajectory) {
            if (!is_running) break;

            auto bag_timestamp = rclcpp::Clock().now();
            auto loop_start_time = std::chrono::steady_clock::now();

            auto state = RobotDriver->get_state();
            if (state) {
                writer->write(*state, state_topic, bag_timestamp);
            }
            else {
                auto damping_command = go2::utilities::damping_command();
                result.Update(RobotDriver->update_command(damping_command));
                std::cerr << "Failed to get the latest state message." << std::endl;
                is_running = false; break;
            }

            if (ControllerDriver->is_pressed(WirelessControllerDriver::Button::B)) {
                std::cout << "Setting control mode to DAMPING and Terminating." << std::endl;
                auto damping_command = go2::utilities::damping_command();
                result.Update(RobotDriver->update_command(damping_command));
                ABSL_CHECK(result.ok()) << result.message();
                is_running = false; break;
            }
            
            auto position_command = go2::utilities::default_position_command();
            for (std::size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
                auto& motor_command = position_command.motor_cmd[i];
                motor_command.q = setpoint[i];
                motor_command.kp = kp[i];
                motor_command.kd = kd[i];
            }

            writer->write(position_command, command_topic, bag_timestamp);
            result.Update(RobotDriver->update_command(position_command));
            ABSL_CHECK(result.ok()) << result.message();

            auto elapsed_time = std::chrono::steady_clock::now() - loop_start_time;
            if (elapsed_time < std::chrono::milliseconds(control_rate_ms))
                std::this_thread::sleep_for(std::chrono::milliseconds(control_rate_ms) - elapsed_time);
            else
                std::cout << "Warning: Loop took longer than expected, skipping sleep." << std::endl;
        }

        // Small pause between trajectories to let the system stabilize:
        std::this_thread::sleep_for(std::chrono::seconds(1));
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
