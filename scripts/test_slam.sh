#!/bin/sh
echo "Start SLAM ..."
echo " 1/4 Launch Gazebo world"
# launch turtle_world.launch file to deploy the turtlebot in the environment
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/myhouse" &
sleep 5

echo " 2/4 Launch gmapping_demo"
# gmapping_demo.launch to perform SLAM
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5

echo " 3/4 Launch naviagation Rviz"
# view_navigation.launch to observe the map in rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

echo " 4/4 Launch keyboard teleop"
# keyboard_teleop.launch to manually control the robot with keyboard commands
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch"
echo "All Nodes Done."
