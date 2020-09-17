#!/bin/bash

# roscore &>/dev/null &
# ROS_ID=$!

TEST_NAME="test0"
source ~/catkin_ws/devel/setup.bash

roslaunch test_sim write_bag_to_file.launch filename:=$TEST_NAME filename_map:=${TEST_NAME}_map filename_poses:=${TEST_NAME}_poses
# REPLAY_ID=$!

# while pgrep -x "play" > /dev/null
# do
#     sleep 1
# done
# sleep 4
# kill $REPLAY_ID

# kill $ROS_ID