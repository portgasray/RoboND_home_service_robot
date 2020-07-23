#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  // parameters to  track robot states
  
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;
  move_base_msgs::MoveBaseGoal dropoff_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map"; //base_link  //map
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  dropoff_goal.target_pose.header.frame_id = "map";
  dropoff_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup_goal.target_pose.pose.position.x = 4.1609;
  pickup_goal.target_pose.pose.position.y = 4.2602;
  pickup_goal.target_pose.pose.position.z = 0.0;
  pickup_goal.target_pose.pose.orientation.w = 0.9997;
  // pickup_goal.target_pose.pose.orientation.y = 4.17;

  dropoff_goal.target_pose.pose.position.x = -3.5854;
  dropoff_goal.target_pose.pose.position.y = 3.8062;
  dropoff_goal.target_pose.pose.position.z = 0.0;
  dropoff_goal.target_pose.pose.orientation.w = 0.7240;
  // dropoff_goal.target_pose.pose.orientation.y = 3.94;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick up goal");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  bool pick_up_success = false;
  bool drop_off_success = false;

  // Check if the robot reached its goal
  if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
    ROS_INFO("Hooray, the robot moved to pick up place");
    pick_up_success = true;
  }
  else {
    ROS_INFO("The robot failed to move to pick up place for some reason");
  }

  if (pick_up_success) {
    //wait for 5 sec
    ros::Duration(5.0).sleep();

    ROS_INFO("Sending drop off goal");
    ac.sendGoal(dropoff_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the robot moved to drop off zone");
      drop_off_success = true;
    }
    else {
      ROS_INFO("The robot failed to move to drop off zone for some reason");
    } 
  }

  if ( pick_up_success && drop_off_success )
    ROS_INFO("Hooray, the robot moved to both place success!");

  return 0;
}
