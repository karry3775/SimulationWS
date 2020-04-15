#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // first we will just print the message out
  ROS_INFO_STREAM("odom callback was called successfully" << std::endl);
}

int main(int argc, char **argv){

  // initiate a node
  ros::init(argc, argv, "ground_truth_node");
  // create a node handle
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 1000, odomCallback);
  ros::spin();
}
