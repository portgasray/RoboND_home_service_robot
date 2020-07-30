#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <complex>

ros::Publisher marker_pub;
visualization_msgs::Marker marker;
// Define pick up and drop off position
std::vector<double> pickup_marker{ 5.35, 4.25, 1.0 };
std::vector<double> dropoff_marker{ -3.65, 4.72, 1.0 };


bool marker_appear = true;

void add_marker_callback (const nav_msgs::Odometry::ConstPtr& msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  // ROS_INFO_THROTTLE(3, "Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,
  //                   msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO_THROTTLE(3, "Position-> x: [%f], y: [%f], z: [%f]", x, y, z);
  double tolerance = 0.2;
  double dis_pick_up = 1.15;
  double dis_drop_off = 0.93;
  double cal_dis_pick = sqrt(pow(x - pickup_marker[0] , 2) + pow(y - pickup_marker[1], 2));
  double cal_dis_drop = sqrt(pow(x - dropoff_marker[0] , 2) + pow(y - dropoff_marker[1], 2));
  
  // Check if the arm is moving by comparing its current joints position to its latest
  if ( marker_appear && fabs( cal_dis_pick - dis_pick_up ) < tolerance ) {
    //arrived at the pick up zone
    ros::Duration(5).sleep(); // simulate the pick up
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    marker_appear = false;
  }
  
  if( !marker_appear && fabs( cal_dis_drop - dis_drop_off ) < tolerance) {
    
    marker.pose.position.x = dropoff_marker[0];
    marker.pose.position.y = dropoff_marker[1];
    marker.pose.orientation.w = dropoff_marker[2];
    marker.action = visualization_msgs::Marker::ADD;
    ros::Duration(5).sleep();
    marker_pub.publish(marker);

  } 
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Subscribe to /odom topic to track the robot pose
  ros::Subscriber odom_sub = n.subscribe("/odom", 10, add_marker_callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
    
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pickup_marker[0];
  marker.pose.position.y = pickup_marker[1];
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = pickup_marker[2];

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = .4;
  marker.scale.y = .4;
  marker.scale.z = .4;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  marker_pub.publish(marker);
  marker_appear = true;
  ROS_INFO("Marker Appear in Pick Up Zone!");

  // Handle ROS communication events
  ros::spin();
}
