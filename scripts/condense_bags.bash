#!/usr/bin/env bash
# This script converts bag data into compact .csv metric files for Matlab plotting

for ((i=0; i<5; i++)); do
  # Test params
  TEST_NAME="test${i}"

  pkill roscore
  pkill rosmaster

  # Launch bag and save data to .csv using save_test_data node
  roslaunch test_sim write_bag_to_file.launch filename:=$TEST_NAME filename_map:=${TEST_NAME}_map filename_poses:=${TEST_NAME}_poses

  sleep 1 # Let's everything close appropriately.
done

