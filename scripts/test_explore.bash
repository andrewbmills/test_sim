#!/usr/bin/env bash

ENV_MAZE="unreal_maze"
ENV_SUBWAY="subway"
ENV_POWERPLANT="powerplant"
ENV_BRIDGE="truss_bridge"
MODEL_X4="X4"
MODEL_FIREFLY="firefly"
ALGO_COMPLEX_FRONTIER="complex_frontier"
ALGO_SIMPLE_FRONTIER="simple_frontier"
ALGO_IFVE="ifve"
ALGO_IFVE_IROS="ifve_iros"

# Read in all the tests from some file
# Each test needs a test environment, vehicle model, an algorithm name, and a number of runs

# TEST_ENV="subway"
TEST_ENV="truss_bridge"
TEST_MODEL="X4"
TEST_ALGO="complex_frontier"
TEST_RUNS=1
UTILITY_FUNCTION="efficiency"
UTILITY_PARAM=1.0
GAIN="unseen"

TEST_PATH="/home/andrew/tests/data/${TEST_ENV}"

# UTILITY_FUNCTIONS=("exponential" "linear")
# UTILITY_PARAMS=(10.0 50.0 100.0 200.0 500.0 1000.0)
# UTILITY_PARAMS=(50.0 2.0 100.0 0.1)
# UTILITY_FUNCTIONS=("linear" "exponential" "linear" "exponential")
# IDS=(1 1 2 0)
# for ((k=0; k<2; k++)); do
#   UTILITY_FUNCTION=UTILITY_FUNCTIONS[k]
# for ((j=0; j<6; j++)); do
  # UTILITY_PARAM=${UTILITY_PARAMS[j]}
  for ((i=0; i<$TEST_RUNS; i++)); do
    roscore &>/dev/null &
    sleep 2
    # UTILITY_FUNCTION=${UTILITY_FUNCTIONS[i]}
    # UTILITY_PARAM=${UTILITY_PARAMS[i]}
    # Test params
    # TEST_NAME="test${i}"
    TEST_NAME="test${IDS[i]}"
    TEST_FILENAME="${TEST_ENV}_${TEST_ALGO}_${TEST_NAME}_${UTILITY_FUNCTION}_${UTILITY_PARAM}_${GAIN}"

    # Launch the simulator
    if [[ "$TEST_ENV" == "$ENV_MAZE" ]]; then
      cd ~/Downloads/Binary_Maze/LinuxNoEditor/
      ./experiment4.sh &>/dev/null &
      UNREAL_ID=$!
      cd
      sleep 3
      source ~/active_3d_ws/devel/setup.bash
      roslaunch active_3d_planning_app_reconstruction example_ifve.launch experiment_config:=Maze.yaml &>/dev/null &
      GAZEBO_ID=$!
      sleep 5
      source ~/subt_gz9_ws/install/setup.bash
      roslaunch subt_example x4_lee_controller_only.launch x4_name:=firefly &>/dev/null &
      ROBOT_ID=$!
      sleep 3
    else
      source ~/subt_gz9_ws/install/setup.bash
      roslaunch subt_gazebo competition.launch scenario:=$TEST_ENV &>/dev/null &
      GAZEBO_ID=$!
      sleep 3
      if [[ "$TEST_ENV" == "$ENV_SUBWAY" ]]; then
        X4_SENSOR_CONFIG_2=1 roslaunch subt_example x4.launch x0:=-4.0 y0:=0.0 z0:=0.1 &>/dev/null &
      elif [[ "$TEST_ENV" == "$ENV_POWERPLANT" ]]; then
        X4_SENSOR_CONFIG_2=1 roslaunch subt_example x4.launch x0:=-40.0 y0:=30.0 z0:=0.1 &>/dev/null &
      else
        X4_SENSOR_CONFIG_2=1 roslaunch subt_example x4.launch x0:=0.0 y0:=0.0 z0:=0.6&>/dev/null &
      fi
      ROBOT_ID=$!
      sleep 3
    # elif [[$TEST_ENV == ]]
    fi

    if [[ "$TEST_MODEL" == "$MODEL_X4" ]]; then
      # Open rviz
      rviz -d ~/.rviz/x4.rviz &>/dev/null &
      RVIZ_ID=$!
      sleep 2

      # Launch mapping and state estimation
      source ~/catkin_ws/devel/setup.bash
      roslaunch msfm3d mapping_test.launch &>/dev/null &
      MAPPING_ID=$!
      sleep 2

      # Fly up and into the map volume
      roscd test_sim/scripts
      ./takeoff_and_spin.sh
      cd
    fi

    # Launch rosbag record
    source ~/catkin_ws/devel/setup.bash
    roslaunch test_sim record_bag.launch filename:=$TEST_FILENAME path:=$TEST_PATH model:=$TEST_MODEL &>/dev/null &
    ROSBAG_ID=$!

    # Launch planning and control
    if [[ "$TEST_ALGO" == "$ALGO_COMPLEX_FRONTIER" ]]; then
      if [[ "$TEST_ENV" == "$ENV_MAZE" ]]; then
        roslaunch msfm3d frontier_planner_unreal_maze.launch &>/dev/null &
      elif [[ "$TEST_ENV" == "$ENV_SUBWAY" ]]; then
        roslaunch msfm3d frontier_planner_subway_evaluation.launch &
      elif [[ "$TEST_ENV" == "$ENV_POWERPLANT" ]]; then
        roslaunch msfm3d frontier_planner_powerplant.launch model_name:=$TEST_MODEL &
      elif [[ "$TEST_ENV" == "$ENV_BRIDGE" ]]; then
        roslaunch msfm3d frontier_planner_truss_bridge.launch &
      fi
    elif [[ "$TEST_ALGO" == "$ALGO_IFVE" ]]; then
      if [[ "$TEST_ENV" == "$ENV_MAZE" ]]; then
        roslaunch msfm3d goal_pose_planner_unreal_maze.launch &
      elif [[ "$TEST_ENV" == "$ENV_SUBWAY" ]]; then
        roslaunch msfm3d goal_pose_planner_subway_evaluation.launch utility_function:=$UTILITY_FUNCTION utility_param:=$UTILITY_PARAM gain_function:=$GAIN &
      elif [[ "$TEST_ENV" == "$ENV_POWERPLANT" ]]; then
        roslaunch msfm3d goal_pose_planner_powerplant.launch &
      elif [[ "$TEST_ENV" == "$ENV_BRIDGE" ]]; then
        roslaunch msfm3d goal_pose_planner_truss_bridge.launch utility_function:=$UTILITY_FUNCTION utility_param:=$UTILITY_PARAM gain_function:=$GAIN &
      fi
    elif [[ "$TEST_ALGO" == "$ALGO_IFVE_IROS" ]]; then
      if [[ "$TEST_ENV" == "$ENV_MAZE" ]]; then
        roslaunch msfm3d goal_pose_planner_iros_unreal_maze.launch &
      fi
    else
      roslaunch msfm3d goal_pose_planner_unreal_maze.launch &
    fi
    PLAN_ID=$!

    sleep 210
    kill $PLAN_ID
    kill $ROSBAG_ID
    kill $RVIZ_ID
    kill $MAPPING_ID
    kill $ROBOT_ID
    kill $GAZEBO_ID
    if [[ "$TEST_ENV" == "$ENV_MAZE" ]]; then
      pkill experiment4
    fi
    pkill roscore
    pkill rosmaster

    sleep 10

    # Launch bag and save data to .csv using save_test_data node
    if [[ "$TEST_ENV" == "$ENV_SUBWAY" ]]; then
      roslaunch test_sim write_bag_to_file_subway.launch path:=$TEST_PATH filename:=$TEST_FILENAME model_name:=$TEST_MODEL
    elif [[ "$TEST_ENV" == "$ENV_MAZE" ]]; then
      roslaunch test_sim write_bag_to_file_unreal_maze.launch path:=$TEST_PATH filename:=$TEST_FILENAME
    fi

    sleep 10 # Let's everything close appropriately.
  done
done