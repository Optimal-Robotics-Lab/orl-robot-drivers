#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <map>

// 1. Bazel Runfiles library
#include "rules_cc/cc/runfiles/runfiles.h"

// 2. ROS2 bag C++ libraries
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

// 3. The C++ message headers
#include "src/go2/msgs/unitree_go_msgs.h"

// Use a unique_ptr for the Runfiles object
using rules_cc::cc::runfiles::Runfiles;

int main(int argc, char** argv) {
    // --- 1. Find the bag directory using Runfiles ---
    std::string error;
    std::unique_ptr<Runfiles> runfiles(Runfiles::Create(argv[0], &error));
    if (runfiles == nullptr) {
        std::cerr << "Error creating runfiles: " << error << std::endl;
        return 1;
    }

    // We use the same pattern as Python: find a key file, then get its directory
    //
    // --- THIS IS THE FIX ---
    // The path should NOT include the workspace name "orl-robot-drivers"
    std::string manifest_runfile_path = "orl-robot-drivers/tools/bags/BUILD.bazel";
    std::string manifest_path = runfiles->Rlocation(manifest_runfile_path);
    if (manifest_path.empty()) {
        std::cerr << "Error: Could not find bag manifest file in runfiles." << std::endl;
        std::cerr << "Looked for: " << manifest_runfile_path << std::endl;
        return 1;
    }

    // Get the parent directory (this is C++17's filesystem library)
    // A simple string-based alternative is used for broader compatibility.
    std::string bag_path = manifest_path.substr(0, manifest_path.find_last_of("/\\"));
    std::cout << "Successfully found manifest at: " << manifest_path << std::endl;
    std::cout << "Successfully derived bag directory uri: " << bag_path << std::endl;

    // --- 2. Set up the bag reader ---
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::readers::SequentialReader reader;

    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return 1;
    }

    // --- 3. Set up message deserializers ---
    rclcpp::Serialization<unitree_go::msg::LowState> lowstate_serializer;
    rclcpp::Serialization<unitree_go::msg::LowCmd> lowcmd_serializer;

    // Get a map of topic names to their types
    std::map<std::string, std::string> topic_to_type;
    for (const auto& topic_meta : reader.get_all_topics_and_types()) {
        topic_to_type[topic_meta.name] = topic_meta.type;
    }

    std::cout << "\n--- Bag Contents ---" << std::endl;
    std::map<std::string, int> topic_counts;

    // --- 4. Read and deserialize messages ---
    while (reader.has_next()) {
        auto serialized_message = reader.read_next();
        
        std::string topic_name = serialized_message->topic_name;
        topic_counts[topic_name]++;

        if (topic_counts[topic_name] > 1) {
             // Only print the first message to avoid spam
            continue;
        }

        std::cout << "\n[First message from " << topic_name << " at "
                  << serialized_message->time_stamp << " ns]" << std::endl;

        // Create a ROS2 serialized message object to pass to the deserializer
        rclcpp::SerializedMessage extracted_message(*serialized_message->serialized_data);

        const std::string& type_name = topic_to_type[topic_name];

        if (type_name == "unitree_go/msg/LowState") {
            unitree_go::msg::LowState msg;
            lowstate_serializer.deserialize_message(&extracted_message, &msg);
            std::cout << "  Motor 0 Temp: " << static_cast<int>(msg.motor_state[0].temperature) << std::endl;
            std::cout << "  Foot Force 0: " << msg.foot_force[0] << std::endl;

        } else if (type_name == "unitree_go/msg/LowCmd") {
            unitree_go::msg::LowCmd msg;
            lowcmd_serializer.deserialize_message(&extracted_message, &msg);
            std::cout << "  Motor 0 q: " << msg.motor_cmd[0].q << std::endl;
            std::cout << "  Motor 0 tau: " << msg.motor_cmd[0].tau << std::endl;
        }
    }

    std::cout << "\n--- Summary ---" << std::endl;
    for (const auto& pair : topic_counts) {
        std::cout << "Topic: " << pair.first << " | Total Messages: " << pair.second << std::endl;
    }

    return 0;
}