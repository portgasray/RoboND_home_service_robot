# RoboND_home_service_robot
Laste project of Robotics Nano program for path planning

* CATKIN_IGNORE put in `../turtlebot_apps/turtlebot_teleop/.` and `../turtlebot_interactions/turtlebot_simulator/.`

update `turtlebot_simulator\turtlebot_gazebo/launch/includes/kobuki.launch.xml`

```
<node name="spawn_turtlebot_model" pkg="gazebo_ros" type="spawn_model"
      args="$(optenv ROBOT_INITIAL_POSE) -unpause -urdf -param robot_description -model mobile_base"/>
```
to 
```
  <!-- Robot pose -->
  <arg name="x" default="0"/>
  <arg name="y" default="0"/>
  <arg name="z" default="0"/>
  <arg name="roll" default="0"/>
  <arg name="pitch" default="0"/>
  <arg name="yaw" default="1.5707"/>

 <node name="spawn_turtlebot_model" pkg="gazebo_ros" type="spawn_model"
        args="-unpause -urdf -param robot_description -model mobile_base
              -x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg roll) -P $(arg pitch) -Y $(arg yaw)"/>
```
