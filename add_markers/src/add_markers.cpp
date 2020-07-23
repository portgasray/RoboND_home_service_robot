#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscribe to /odom topic to track the robot pose
  // ros::Subscriber sub1 = n.subscribe("/odom", 10, );

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker_pick, marker_drop;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker_pick.header.frame_id = "map";
    marker_pick.header.stamp = ros::Time::now();


    marker_drop.header.frame_id = "map";
    marker_drop.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_pick.ns = "pick_up";
    marker_pick.id = 0;

    marker_drop.ns = "drop_off";
    marker_drop.id = 1;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker_pick.type = shape;
    marker_drop.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_pick.action = visualization_msgs::Marker::ADD;
    marker_drop.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_pick.pose.position.x = 5.35;
    marker_pick.pose.position.y = 4.25;
    marker_pick.pose.position.z = 0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    marker_pick.pose.orientation.w = 1.0;

    marker_drop.pose.position.x = -3.65;
    marker_drop.pose.position.y = 4.72;
    marker_drop.pose.position.z = 0;

    marker_drop.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_pick.scale.x = .5;
    marker_pick.scale.y = .5;
    marker_pick.scale.z = .5;

    marker_drop.scale.x = .5;
    marker_drop.scale.y = .5;
    marker_drop.scale.z = .5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_pick.color.r = 0.0f;
    marker_pick.color.g = 0.0f;
    marker_pick.color.b = 1.0f;
    marker_pick.color.a = 1.0;

    marker_pick.lifetime = ros::Duration();

    marker_drop.color.r = 0.0f;
    marker_drop.color.g = 0.0f;
    marker_drop.color.b = 1.0f;
    marker_drop.color.a = 1.0;

    marker_drop.lifetime = ros::Duration();
    // Publish the marker
    // while (marker_pub.getNumSubscribers() < 1)
    // {
    //   if (!ros::ok())
    //   {
    //     return 0;
    //   }
    //   ROS_WARN_ONCE("Please create a subscriber to the marker");
    //   sleep(1);
    // }
    ROS_INFO("Wait for delivering ...");
    marker_pub.publish(marker_pick);
    ros::Duration(5.0).sleep();

    // //Hied the marker in pick up zone.
    // marker_pick.action = visualization_msgs::Marker::DELETE;
    // marker_pub.publish(marker_pick);

    ROS_INFO("Delivering ...");
    ros::Duration(5.0).sleep();

    // ROS_INFO("Pick up the Marker, Delivering ... ");
  
    ROS_INFO(" Marker Arrived at Drop off zone.");
    marker_pub.publish(marker_drop);

    r.sleep();
  }

  // Handle ROS communication events
  ros::spin();
}
