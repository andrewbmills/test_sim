#!/usr/bin/env bash
# This script converts bag data into compact .csv metric files for Matlab plotting

ENV_MAZE="unreal_maze"
ENV_SUBWAY="subway"
ENV_POWERPLANT="powerplant"
MODEL_X4="X4"
MODEL_FIREFLY="firefly"
ALGO_COMPLEX_FRONTIER="complex_frontier"
ALGO_SIMPLE_FRONTIER="simple_frontier"
ALGO_IFVE="ifve"

# Read in all the tests from some file
# Each test needs a test environment, vehicle model, an algorithm name, and a number of runs

# TEST_ENV="subway"
TEST_ENV="unreal_maze"
# TEST_ENV="powerplant"
# TEST_ENV="ignition_cave"
# TEST_ENV="ignition_urban"
# TEST_MODEL="X4"
TEST_MODEL="firefly"
TEST_ALGO="complex_frontier"
# TEST_ALGO="ifve"
# TEST_ALGO="simple_frontier"
TEST_RUNS=6

TEST_PATH="/home/andrew/tests/data/${TEST_ENV}"

for ((i=5; i<$TEST_RUNS; i++)); do
  # Test params
  TEST_NAME="test${i}"
  TEST_FILENAME="${TEST_ENV}_${TEST_ALGO}_${TEST_NAME}"

  pkill roscore
  pkill rosmaster

  # Launch bag and save data to .csv using save_test_data node
  # roslaunch test_sim write_bag_to_file.launch path:=$TEST_PATH filename:=$TEST_FILENAME
  roslaunch test_sim write_bag_to_file_unreal_maze.launch path:=$TEST_PATH filename:=$TEST_FILENAME

  sleep 3 # Let's everything close appropriately.
done

