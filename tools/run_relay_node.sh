#!/bin/bash

# Remember to enable the relay before running this script:

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI=file:///home/unitree/repository/orl-robot-drivers/.user/cyclonedds.xml

bazel run \
  --action_env=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --action_env=ROS_DOMAIN_ID=0 \
  --action_env=CYCLONEDDS_URI=file:///home/unitree/repository/orl-robot-drivers/.user/cyclonedds.xml \
  //tools:relay_node
