#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "project1/resetOdom.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_odom_client");
  ros::NodeHandle n;

  ros::Publisher resetpub = n.advertise<nav_msgs::Odometry>("/newodom", 1);

  ros::ServiceClient client = n.serviceClient<project1::resetOdom>("reset_odom");
  project1::resetOdom srv;

  if (client.call(srv))
  {
      resetpub.publish(srv.response.newOdom);
      ROS_INFO("\n\n ODOMETRY RESET COMPLETED \n\n");
  }

  else
  {
    ROS_ERROR("Failed to call service reset_odom");
    return 1;
  }

  return 0;
}