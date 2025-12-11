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
    rclcpp::Serialization<vicon_receiver::msg::Position> vicon_position_serializer;
    rclcpp::Serialization<geometry_msgs::msg::Vector3> policy_command_serializer;

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

    std::vector<MotorMessageData> state_history;
    std::vector<MotorMessageData> command_history;
    std::vector<ContactMessageData> contact_history;
    std::vector<IMUMessageData> imu_history;
    std::vector<ViconMessageData> vicon_history;
    std::vector<PolicyCommandData> policy_command_history;

    while (reader.has_next()) {
        auto serialized_message = reader.read_next();
        
        std::string topic_name = serialized_message->topic_name;

        rclcpp::SerializedMessage extracted_message(*serialized_message->serialized_data);

        const std::string& type_name = topic_to_type[topic_name];

        if (type_name == "unitree_go/msg/LowState") {
            auto time_stamp_ns = serialized_message->time_stamp;

            unitree_go::msg::LowState msg;
            lowstate_serializer.deserialize_message(&extracted_message, &msg);
            std::array<float, robot::go2::constants::num_joints> positions{}, velocities{}, torques{}, accelerations{};
            for (size_t i = 0; i < robot::go2::constants::num_joints; ++i) {
                positions[i] = msg.motor_state[i].q;
                velocities[i] = msg.motor_state[i].dq;
                torques[i] = msg.motor_state[i].tau_est;
                accelerations[i] = msg.motor_state[i].ddq;
            }

            // Foot Contacts:
            std::array<std::int16_t, 4> foot_force, foot_force_estimate;
            for (size_t i = 0; i < 4; ++i) {
                foot_force[i] = msg.foot_force[i];
                foot_force_estimate[i] = msg.foot_force_est[i];
            }

            // IMU State:
            std::array<float, 4> quaternion{};
            std::array<float, 3> gyroscope{};
            std::array<float, 3> accelerometer{};

            for (size_t i = 0; i < 4; ++i)
                quaternion[i] = msg.imu_state.quaternion[i];

            for (size_t i = 0; i < 3; ++i) {
                gyroscope[i] = msg.imu_state.gyroscope[i];
                accelerometer[i] = msg.imu_state.accelerometer[i];
            }

            auto state_data = MotorMessageData{
                .time_stamp_ns = time_stamp_ns,
                .positions = positions,
                .velocities = velocities,
                .torques = torques,
                .accelerations = accelerations
            };

            auto contact_data = ContactMessageData{
                .time_stamp_ns = time_stamp_ns,
                .foot_force = foot_force,
                .foot_force_estimate = foot_force_estimate
            };

            auto imu_data = IMUMessageData{
                .time_stamp_ns = time_stamp_ns,
                .quaternion = quaternion,
                .gyroscope = gyroscope,
                .accelerometer = accelerometer
            };

            state_history.push_back(state_data);
            contact_history.push_back(contact_data);
            imu_history.push_back(imu_data);

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

            auto command_data = MotorMessageData{
                .time_stamp_ns = time_stamp_ns,
                .positions = positions,
                .velocities = velocities,
                .torques = torques
            };

            command_history.push_back(command_data);
        }
        else if (type_name == "vicon_receiver/msg/Position") {
            auto recieved_time_stamp_ns = serialized_message->time_stamp;
            vicon_receiver::msg::Position msg;
            vicon_position_serializer.deserialize_message(&extracted_message, &msg);
            std::array<float, 3> position{};
            std::array<float, 4> orientation{};
            auto time_stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
            position[0] = msg.x_trans;
            position[1] = msg.y_trans;
            position[2] = msg.z_trans;
            orientation[0] = msg.w;
            orientation[1] = msg.x_rot;
            orientation[2] = msg.y_rot;
            orientation[3] = msg.z_rot;
            
            auto vicon_data = ViconMessageData{
                .time_stamp_ns = time_stamp_ns,
                .position = position,
                .orientation = orientation
            };

            vicon_history.push_back(vicon_data);
        }
        else if (type_name == "geometry_msgs/msg/Vector3") {
            auto time_stamp_ns = serialized_message->time_stamp;

            geometry_msgs::msg::Vector3 msg;
            policy_command_serializer.deserialize_message(&extracted_message, &msg);

            std::array<float, 3> command{};
            command[0] = msg.x;
            command[1] = msg.y;
            command[2] = msg.z;

            auto policy_command_data = PolicyCommandData{
                .time_stamp_ns = time_stamp_ns,
                .command = command
            };

            policy_command_history.push_back(policy_command_data);
        }
    }
    
    std::ofstream state_file("state_history.csv");
    for (const auto& entry : state_history) {
        state_file << entry.time_stamp_ns;
        for (const auto& position : entry.positions)
            state_file << "," << position;

        for (const auto& velocity : entry.velocities)
            state_file << "," << velocity;

        for (const auto& torque : entry.torques)
            state_file << "," << torque;

        for (const auto& acceleration : entry.accelerations)
            state_file << "," << acceleration;

        state_file << "\n";
    }
    state_file.close();

    std::ofstream contact_file("contact_history.csv");
    for (const auto& entry : contact_history) {
        contact_file << entry.time_stamp_ns;
        for (const auto& force : entry.foot_force)
            contact_file << "," << force;
        for (const auto& force_est : entry.foot_force_estimate)
            contact_file << "," << force_est;
        contact_file << "\n";
    }
    contact_file.close();

    std::ofstream imu_file("imu_history.csv");
    for (const auto& entry : imu_history) {
        imu_file << entry.time_stamp_ns;
        for (const auto& q : entry.quaternion)
            imu_file << "," << q;

        for (const auto& g : entry.gyroscope)
            imu_file << "," << g;

        for (const auto& a : entry.accelerometer)
            imu_file << "," << a;

        imu_file << "\n";
    }
    imu_file.close();

    std::ofstream command_file("command_history.csv");
    for (const auto& entry : command_history) {
        command_file << entry.time_stamp_ns;
        for (const auto& position : entry.positions)
            command_file << "," << position;

        for (const auto& velocity : entry.velocities)
            command_file << "," << velocity;

        for (const auto& torque : entry.torques)
            command_file << "," << torque;

        command_file << "\n";
    }
    command_file.close();

    std::ofstream vicon_file("vicon_history.csv");
    for (const auto& entry : vicon_history) {
        vicon_file << entry.time_stamp_ns;
        for (const auto& pos : entry.position)
            vicon_file << "," << pos;

        for (const auto& ori : entry.orientation)
            vicon_file << "," << ori;

        vicon_file << "\n";
    }
    vicon_file.close();

    std::ofstream policy_command_file("policy_command_history.csv");
    for (const auto& entry : policy_command_history) {
        policy_command_file << entry.time_stamp_ns;
        for (const auto& cmd : entry.command)
            policy_command_file << "," << cmd;

        policy_command_file << "\n";
    }
    policy_command_file.close();

    return 0;
}