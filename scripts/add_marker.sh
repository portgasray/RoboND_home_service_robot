#!/bin/sh

echo "Start ADD MARKERS ..."
echo " 1/4 Launch Turtlebot Gazebo World"
# launch turtle_world.launch file to deploy the turtlebot in the environment
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/myhouse" &
sleep 5

echo " 2/4 Launch AMCL "
# amcl_demo.launch to perform AMCL to localize the robot itself
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/myhouse.yaml" & 
sleep 5

echo " 3/4 Launch Naviagation Rviz"
# view_navigation.launch to observe the map in rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

echo " 4/4 Launch Add Markers "
xterm  -e  " rosrun add_markers add_markers"
echo "All Nodes Done."
