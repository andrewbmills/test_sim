#!/usr/bin/env bash
MODEL_NAME="X4"

# Takeoff and do a 360 degree spin to fill in the map.
rostopic pub -r 3.0 /$MODEL_NAME/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
FLY_ID=$!
sleep 5
kill $FLY_ID
rostopic pub -r 3.0 /$MODEL_NAME/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' &>/dev/null &
FLY_ID=$!
sleep 8
kill $FLY_ID
rostopic pub -r 3.0 /$MODEL_NAME/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: -0.25}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &>/dev/null &
FLY_ID=$!
sleep 2
kill $FLY_ID