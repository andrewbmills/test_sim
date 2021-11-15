#!/usr/bin/env bash

# Takeoff and do a 360 degree spin to fill in the map.
rostopic pub -r 3.0 /X4/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
FLY_ID_X4=$!
rostopic pub -r 3.0 /X5/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
FLY_ID_X5=$!
sleep 4
kill $FLY_ID_X4
kill $FLY_ID_X5
rostopic pub -r 3.0 /X4/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: -0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
FLY_ID_X4=$!
rostopic pub -r 3.0 /X5/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: -0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
FLY_ID_X5=$!
sleep 2
kill $FLY_ID_X4
kill $FLY_ID_X5
