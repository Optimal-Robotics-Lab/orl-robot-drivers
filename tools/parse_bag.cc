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
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/time.h"

#include "src/go2/lib/utils/constants.h"
#include "src/go2/msgs/unitree_go_msgs.h"

using rules_cc::cc::runfiles::Runfiles;

ABSL_FLAG(
    std::string, directory_name, "", "ROS2 bag directory name under orl-robot-drivers/tools/bags/ to process."
);


int main(int argc, char** argv) {
    absl::ParseCommandLine(argc, argv);

    std::string error;
    std::unique_ptr<Runfiles> runfiles(Runfiles::Create(argv[0], &error));
    if (runfiles == nullptr) {
        std::cerr << "Error creating runfiles: " << error << std::endl;
        return 1;
    }

    std::string bag_directory = absl::GetFlag(FLAGS_directory_name);
    if (bag_directory.empty()) {
        std::cerr << "Error: The --directory_name flag is required." << std::endl;
        std::cerr << "Usage: " << argv[0] << " --directory_name=<your-directory-name>" << std::endl;
        return 1;
    }

    std::string manifest_runfile_path = "orl-robot-drivers/tools/bags/" + bag_directory + "/metadata.yaml";
    std::filesystem::path manifest_path = runfiles->Rlocation(manifest_runfile_path);
    if (!std::filesystem::exists(manifest_path)) {
        std::cerr << "Error: Could not find bag manifest file in runfiles." << std::endl;
        std::cerr << "Looked for: " << manifest_runfile_path << std::endl;
        return 1;
    }

    std::filesystem::path directory = manifest_path.parent_path();
    std::cout << "Successfully found manifest at: " << manifest_path << std::endl;
    std::cout << "Successfully derived bag directory uri: " << directory << std::endl;


    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = directory;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::readers::SequentialReader reader;

    try {
        reader.open(storage_options, converter_options);
    }
    catch (const std::exception& e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::Serialization<unitree_go::msg::LowState> lowstate_serializer;
    rclcpp::Serialization<unitree_go::msg::LowCmd> lowcmd_serializer;

    // Bag Metadata:
    const auto& metadata = reader.get_metadata();
    const auto starting_time = metadata.starting_time;
    const auto duration = metadata.duration;
    auto message_count = metadata.message_count;

    // Topic Metadata:
    std::map<std::string, std::string> topic_to_type;
    for (const auto& topic_meta : reader.get_all_topics_and_types()) {
        topic_to_type[topic_meta.name] = topic_meta.type;
    }

    struct MessageData {
        rcutils_time_point_value_t time_stamp_ns{ 0 };
        std::array<float, robot::go2::constants::num_joints> positions{};
        std::array<float, robot::go2::constants::num_joints> velocities{};
        std::array<float, robot::go2::constants::num_joints> torques{};
    };

    std::vector<MessageData> state_history;
    std::vector<MessageData> command_history;

    while (reader.has_next()) {
        auto serialized_message = reader.read_next();
        
        std::string topic_name = serialized_message->topic_name;

        rclcpp::SerializedMessage extracted_message(*serialized_message->serialized_data);

        const std::string& type_name = topic_to_type[topic_name];

        if (type_name == "unitree_go/msg/LowState") {
            auto time_stamp_ns = serialized_message->time_stamp;

            unitree_go::msg::LowState msg;
            lowstate_serializer.deserialize_message(&extracted_message, &msg);
            std::array<float, robot::go2::constants::num_joints> positions{}, velocities{}, torques{};
            for (size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
                positions[i] = msg.motor_state[i].q;
                velocities[i] = msg.motor_state[i].dq;
                torques[i] = msg.motor_state[i].tau_est;
            }

            auto data = MessageData{
                .time_stamp_ns = time_stamp_ns,
                .positions = positions,
                .velocities = velocities,
                .torques = torques
            };

            state_history.push_back(data);

        } 
        else if (type_name == "unitree_go/msg/LowCmd") {
            auto time_stamp_ns = serialized_message->time_stamp;

            unitree_go::msg::LowCmd msg;
            lowcmd_serializer.deserialize_message(&extracted_message, &msg);
            std::array<float, robot::go2::constants::num_joints> positions{}, velocities{}, torques{};
            for (size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
                positions[i] = msg.motor_cmd[i].q;
                velocities[i] = msg.motor_cmd[i].dq;
                torques[i] = msg.motor_cmd[i].tau;
            }

            auto data = MessageData{
                .time_stamp_ns = time_stamp_ns,
                .positions = positions,
                .velocities = velocities,
                .torques = torques
            };
            
            command_history.push_back(data);
        }
    }
    
    std::ofstream state_file("go2_state_history.csv");
    for (const auto& entry : state_history) {
        state_file << entry.time_stamp_ns;
        for (const auto& position : entry.positions)
            state_file << "," << position;

        for (const auto& velocity : entry.velocities)
            state_file << "," << velocity;

        for (const auto& torque : entry.torques)
            state_file << "," << torque;

        state_file << "\n";
    }
    state_file.close();

    std::ofstream command_file("go2_command_history.csv");
    for (const auto& entry : command_history) {
        command_file << entry.time_stamp_ns;
        for (const auto& position : entry.positions)
            command_file << "," << position;

        for (const auto& velocity : entry.velocities)
            command_file << "," << velocity;

        for (const auto& torque : entry.torques)
            command_file << "," << torque;

        command_file << ",\n";
    }
    command_file.close();

    return 0;
}