#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

#include "rules_cc/cc/runfiles/runfiles.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/time.h"

#include "src/go2/lib/utils/constants.h"
#include "src/go2/msgs/unitree_go_msgs.h"
#include "src/vicon/msgs/vicon_receiver_msgs.h"
#include "geometry_msgs/msg/vector3.hpp"

using rules_cc::cc::runfiles::Runfiles;

ABSL_FLAG(
    std::string, input_directory, "", "Base directory containing ROS2 bag sessions to search and process."
);

ABSL_FLAG(
    std::string, output_directory, "parsed_csvs", "Directory where the resulting CSV folders will be saved."
);

// --- Data Structures ---

struct MotorMessageData {
    rcutils_time_point_value_t time_stamp_ns{ 0 };
    std::array<float, robot::go2::constants::num_joints> positions{};
    std::array<float, robot::go2::constants::num_joints> velocities{};
    std::array<float, robot::go2::constants::num_joints> torques{};
    std::array<float, robot::go2::constants::num_joints> accelerations{};
};

struct ContactMessageData {
    rcutils_time_point_value_t time_stamp_ns{ 0 };
    std::array<std::int16_t, 4> foot_force{};
    std::array<std::int16_t, 4> foot_force_estimate{};
};

struct IMUMessageData {
    rcutils_time_point_value_t time_stamp_ns{ 0 };
    std::array<float, 4> quaternion{};
    std::array<float, 3> gyroscope{};
    std::array<float, 3> accelerometer{};
};

struct ViconMessageData {
    rcutils_time_point_value_t time_stamp_ns{ 0 };
    std::array<float, 3> position{};
    std::array<float, 4> orientation{};
};

struct PolicyCommandData {
    rcutils_time_point_value_t time_stamp_ns{ 0 };
    std::array<float, 3> command{};
};

// Container for all parsed data in a single session
struct ParsedData {
    std::vector<MotorMessageData> state_history;
    std::vector<MotorMessageData> command_history;
    std::vector<ContactMessageData> contact_history;
    std::vector<IMUMessageData> imu_history;
    std::vector<ViconMessageData> vicon_history;
    std::vector<PolicyCommandData> policy_command_history;
};


// --- Helper Functions ---

void parseBag(const std::filesystem::path& bag_directory, ParsedData& data) {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_directory.string();
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::readers::SequentialReader reader;

    try {
        reader.open(storage_options, converter_options);
    }
    catch (const std::exception& e) {
        std::cerr << "  [!] Error opening bag file " << bag_directory << ": " << e.what() << std::endl;
        return;
    }

    rclcpp::Serialization<unitree_go::msg::LowState> lowstate_serializer;
    rclcpp::Serialization<unitree_go::msg::LowCmd> lowcmd_serializer;
    rclcpp::Serialization<vicon_receiver::msg::Position> vicon_position_serializer;
    rclcpp::Serialization<geometry_msgs::msg::Vector3> policy_command_serializer;

    std::map<std::string, std::string> topic_to_type;
    for (const auto& topic_meta : reader.get_all_topics_and_types()) {
        topic_to_type[topic_meta.name] = topic_meta.type;
    }

    while (reader.has_next()) {
        auto serialized_message = reader.read_next();
        std::string topic_name = serialized_message->topic_name;
        rclcpp::SerializedMessage extracted_message(*serialized_message->serialized_data);
        const std::string& type_name = topic_to_type[topic_name];

        if (type_name == "unitree_go/msg/LowState") {
            unitree_go::msg::LowState msg;
            lowstate_serializer.deserialize_message(&extracted_message, &msg);
            
            MotorMessageData state_data;
            state_data.time_stamp_ns = serialized_message->time_stamp;
            for (size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
                state_data.positions[i] = msg.motor_state[i].q;
                state_data.velocities[i] = msg.motor_state[i].dq;
                state_data.torques[i] = msg.motor_state[i].tau_est;
                state_data.accelerations[i] = msg.motor_state[i].ddq;
            }

            ContactMessageData contact_data;
            contact_data.time_stamp_ns = serialized_message->time_stamp;
            for (size_t i = 0; i < 4; ++i) {
                contact_data.foot_force[i] = msg.foot_force[i];
                contact_data.foot_force_estimate[i] = msg.foot_force_est[i];
            }

            IMUMessageData imu_data;
            imu_data.time_stamp_ns = serialized_message->time_stamp;
            for (size_t i = 0; i < 4; ++i) imu_data.quaternion[i] = msg.imu_state.quaternion[i];
            for (size_t i = 0; i < 3; ++i) {
                imu_data.gyroscope[i] = msg.imu_state.gyroscope[i];
                imu_data.accelerometer[i] = msg.imu_state.accelerometer[i];
            }

            data.state_history.push_back(state_data);
            data.contact_history.push_back(contact_data);
            data.imu_history.push_back(imu_data);
        } 
        else if (type_name == "unitree_go/msg/LowCmd") {
            unitree_go::msg::LowCmd msg;
            lowcmd_serializer.deserialize_message(&extracted_message, &msg);
            
            MotorMessageData command_data;
            command_data.time_stamp_ns = serialized_message->time_stamp;
            for (size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
                command_data.positions[i] = msg.motor_cmd[i].q;
                command_data.velocities[i] = msg.motor_cmd[i].dq;
                command_data.torques[i] = msg.motor_cmd[i].tau;
            }
            data.command_history.push_back(command_data);
        }
        else if (type_name == "vicon_receiver/msg/Position") {
            vicon_receiver::msg::Position msg;
            vicon_position_serializer.deserialize_message(&extracted_message, &msg);
            
            ViconMessageData vicon_data;
            vicon_data.time_stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
            vicon_data.position = {msg.x_trans, msg.y_trans, msg.z_trans};
            vicon_data.orientation = {msg.w, msg.x_rot, msg.y_rot, msg.z_rot};
            data.vicon_history.push_back(vicon_data);
        }
        else if (type_name == "geometry_msgs/msg/Vector3") {
            geometry_msgs::msg::Vector3 msg;
            policy_command_serializer.deserialize_message(&extracted_message, &msg);
            
            PolicyCommandData policy_data;
            policy_data.time_stamp_ns = serialized_message->time_stamp;
            policy_data.command = {static_cast<float>(msg.x), static_cast<float>(msg.y), static_cast<float>(msg.z)};
            data.policy_command_history.push_back(policy_data);
        }
    }
}

void exportToCSV(const std::filesystem::path& output_dir, const ParsedData& data) {
    std::filesystem::create_directories(output_dir);

    auto open_file = [&](const std::string& filename) {
        return std::ofstream((output_dir / filename).string());
    };

    if (!data.state_history.empty()) {
        auto f = open_file("state_history.csv");
        for (const auto& entry : data.state_history) {
            f << entry.time_stamp_ns;
            for (auto v : entry.positions) f << "," << v;
            for (auto v : entry.velocities) f << "," << v;
            for (auto v : entry.torques) f << "," << v;
            for (auto v : entry.accelerations) f << "," << v;
            f << "\n";
        }
    }

    if (!data.contact_history.empty()) {
        auto f = open_file("contact_history.csv");
        for (const auto& entry : data.contact_history) {
            f << entry.time_stamp_ns;
            for (auto v : entry.foot_force) f << "," << v;
            for (auto v : entry.foot_force_estimate) f << "," << v;
            f << "\n";
        }
    }

    if (!data.imu_history.empty()) {
        auto f = open_file("imu_history.csv");
        for (const auto& entry : data.imu_history) {
            f << entry.time_stamp_ns;
            for (auto v : entry.quaternion) f << "," << v;
            for (auto v : entry.gyroscope) f << "," << v;
            for (auto v : entry.accelerometer) f << "," << v;
            f << "\n";
        }
    }

    if (!data.command_history.empty()) {
        auto f = open_file("command_history.csv");
        for (const auto& entry : data.command_history) {
            f << entry.time_stamp_ns;
            for (auto v : entry.positions) f << "," << v;
            for (auto v : entry.velocities) f << "," << v;
            for (auto v : entry.torques) f << "," << v;
            f << "\n";
        }
    }

    if (!data.vicon_history.empty()) {
        auto f = open_file("vicon_history.csv");
        for (const auto& entry : data.vicon_history) {
            f << entry.time_stamp_ns;
            for (auto v : entry.position) f << "," << v;
            for (auto v : entry.orientation) f << "," << v;
            f << "\n";
        }
    }

    if (!data.policy_command_history.empty()) {
        auto f = open_file("policy_command_history.csv");
        for (const auto& entry : data.policy_command_history) {
            f << entry.time_stamp_ns;
            for (auto v : entry.command) f << "," << v;
            f << "\n";
        }
    }
}


int main(int argc, char** argv) {
    absl::ParseCommandLine(argc, argv);

    std::string input_dir_str = absl::GetFlag(FLAGS_input_directory);
    std::string output_dir_str = absl::GetFlag(FLAGS_output_directory);

    if (input_dir_str.empty()) {
        std::cerr << "Error: The --input_directory flag is required." << std::endl;
        std::cerr << "Usage: " << argv[0] << " --input_directory=<path_to_search_for_bags>" << std::endl;
        return 1;
    }

    // Resolve directory through Runfiles (or directly if standard path is given)
    std::string error;
    std::unique_ptr<Runfiles> runfiles(Runfiles::Create(argv[0], &error));
    std::filesystem::path search_path = input_dir_str;
    
    // Attempt to resolve via runfiles first, fall back to literal path if running outside bazel sandbox
    if (runfiles) {
        std::string resolved = runfiles->Rlocation(input_dir_str);
        if (!resolved.empty() && std::filesystem::exists(resolved)) {
            search_path = resolved;
        }
    }

    if (!std::filesystem::exists(search_path) || !std::filesystem::is_directory(search_path)) {
        std::cerr << "Error: Search directory does not exist or is not a directory: " << search_path << std::endl;
        return 1;
    }

    std::cout << "Searching for bags in: " << search_path << std::endl;

    // Store paths to all individual bag directories found
    std::vector<std::filesystem::path> bag_paths;

    for (const auto& entry : std::filesystem::recursive_directory_iterator(search_path)) {
        if (entry.is_regular_file() && entry.path().filename() == "metadata.yaml") {
            // The directory containing metadata.yaml is the bag directory
            bag_paths.push_back(entry.path().parent_path());
        }
    }

    if (bag_paths.empty()) {
        std::cout << "No ROS2 bags (metadata.yaml) found in the provided directory." << std::endl;
        return 0;
    }

    std::filesystem::path base_output(output_dir_str);

    // Process each bag individually
    for (const auto& bag_path : bag_paths) {
        std::cout << "Processing bag: " << bag_path.filename() << std::endl;
        
        ParsedData bag_data;

        // Parse just this specific bag
        parseBag(bag_path, bag_data);

        // Create output directory mirroring the bag directory's name
        // e.g., output_directory/regressed-position-robot-forward-1/
        std::filesystem::path bag_output_dir = base_output / bag_path.filename();
        std::cout << "  -> Saving CSVs to: " << bag_output_dir << "\n" << std::endl;
        
        exportToCSV(bag_output_dir, bag_data);
    }

    std::cout << "All bags processed successfully." << std::endl;
    return 0;
}
