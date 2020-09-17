#!/usr/bin/env bash

# for ((i=0; i<2; i++)); do
  # Test params
  TEST_NAME="test${i}"

  # Launch the simulator
  source ~/subt_gz9_ws/install/setup.bash
  # roslaunch subt_gazebo competition.launch scenario:=urban_underground &>/dev/null &
  # GAZEBO_ID=$!
  sleep 3
  # X4_SENSOR_CONFIG_2=1 roslaunch subt_example x4.launch x0:=-4.0 y0:=0.0 z0:=0.1 &>/dev/null &
  # ROBOT_ID=$!
  sleep 3

  # Launch mapping and state estimation
  source ~/catkin_ws/devel/setup.bash
  roslaunch msfm3d mapping_test.launch &>/dev/null &
  MAPPING_ID=$!
  roscd msfm3d/src
  ./subt_true_odom_X4.py &>/dev/null &
  ODOM_ID=$!
  cd
  sleep 2

  # Open rviz
  # rviz -d ~/.rviz/X4.rviz &>/dev/null &
  # RVIZ_ID=$!
  # sleep 2

  # Fly up and into the map volume
  rostopic pub -r 3.0 /X4/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
  FLY_ID=$!
  sleep 3
  kill $FLY_ID
  rostopic pub -r 3.0 /X4/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.25, y: 0.0, z: 0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
  FLY_ID=$!
  sleep 3
  kill $FLY_ID
  rostopic pub -r 3.0 /X4/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: -0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
  FLY_ID=$!
  sleep 1
  kill $FLY_ID

  # # Launch rosbag record
  roslaunch test_sim record_bag.launch filename:=$TEST_NAME &
  ROSBAG_ID=$!

  # # Launch planning and control
  roslaunch msfm3d planning_and_control_test.launch &>/dev/null &
  PLAN_ID=$!

  # # Launch node to monitor map size and vehicle progress for stopping criteria
  # # export STOP_TEST=0
  # # roslaunch test_sim monitor.launch &>/dev/null &
  # # MONITOR_ID=$!

  sleep 30
  kill $PLAN_ID
  kill $ROSBAG_ID
  # kill $RVIZ_ID
  kill $ODOM_ID
  # kill $MAPPING_ID
  # kill $ROBOT_ID
  # kill $GAZEBO_ID
  pkill roscore
  pkill rosmaster

  # sleep 10
  # Launch bag and save data to .csv using save_test_data node
  # roslaunch test_sim write_bag_to_file.launch filename:=$TEST_NAME filename_map:=${TEST_NAME}_map filename_poses:=${TEST_NAME}_poses

  sleep 10 # Let's everything close appropriately.
# done

