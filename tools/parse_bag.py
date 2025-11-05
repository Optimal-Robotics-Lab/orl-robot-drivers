import sys
import os

from python.runfiles import Runfiles

from rclpy.serialization import deserialize_message
from rosbag2_py._storage import StorageOptions, ConverterOptions
from rosbag2_py._reader import SequentialReader

from unitree_go.msg import LowState, LowCmd

TOPIC_TYPE_MAP = {
    'unitree_go/msg/LowState': LowState,
    'unitree_go/msg/LowCmd': LowCmd,
}

def main():
    try:
        r = Runfiles.Create()
    except Exception as e:
        print(f"Error: Could not initialize Bazel runfiles. Are you running with 'bazel run'?", file=sys.stderr)
        print(f"Details: {e}", file=sys.stderr)
        sys.exit(1)

    runfile_bazel_build = "orl-robot-drivers/tools/bags/BUILD.bazel"
    bazel_build_path = r.Rlocation(runfile_bazel_build)

    # Use Bazel Build file location to determine the directory:
    path = os.path.dirname(bazel_build_path)

    # Check if the file was found
    if not path:
        print(f"Error: Could not find bag file in runfiles.", file=sys.stderr)
        print(f"Looked for: {bag_path_runfile}", file=sys.stderr)
        print("\nDid you remember to run 'record' AND copy the 'robot_bag' directory into //tools?", file=sys.stderr)
        sys.exit(1)

    print(f"Successfully found path at: {path}")

    storage_options = StorageOptions(uri=path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file: {e}", file=sys.stderr)
        sys.exit(1)

    # Get a map of topic names to their string type
    topic_to_type_str = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }

    print("--- Bag Contents ---")
    topic_counts = {}
    
    # 5. Read messages from the bag
    while reader.has_next():
        (topic, data, timestamp_ns) = reader.read_next()

        # Increment count for this topic
        topic_counts[topic] = topic_counts.get(topic, 0) + 1

        # --- Deserialization ---
        msg_type_str = topic_to_type_str.get(topic)
        if not msg_type_str:
            continue

        # Look up the Python class for that type string
        msg_type_class = TOPIC_TYPE_MAP.get(msg_type_str)
        if not msg_type_class:
            continue

        # Finally, deserialize the raw 'data' into a message object
        msg = deserialize_message(data, msg_type_class)
        # --- End Deserialization ---
        
        # 6. Do something with the data
        # As an example, we'll print the first message from each topic
        
        if topic_counts[topic] == 1:
            print(f"\n[First message from {topic} at {timestamp_ns} ns]")
            if topic == "/lowstate":
                print(f"  Motor 0 Temp: {msg.motor_state[0].temperature}")
                print(f"  Foot Force 0: {msg.foot_force[0]}")
            elif topic == "/lowcmd":
                print(f"  Motor 0 q: {msg.motor_cmd[0].q}")
                print(f"  Motor 0 tau: {msg.motor_cmd[0].tau}")
            else:
                print(f"  {msg}") # Print the whole message for other types

    print("\n--- Summary ---")
    for topic, count in topic_counts.items():
        print(f"Topic: {topic:<20} | Total Messages: {count}")
    
    # Good practice to explicitly delete the reader to close file handles
    del reader

if __name__ == "__main__":
    main()