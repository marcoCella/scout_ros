#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "project1/resetOdom.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_odom_client");

  if (argc != 1 && argc != 4)
  {
    ROS_INFO("\n\n Usage: insert either 3 numbers for x, y, theta, or none to set them to 0,0,0 \n\n");
    return 1;
  }

  ros::NodeHandle n;

  ros::Publisher resetpub = n.advertise<nav_msgs::Odometry>("/newodom", 100);
  ros::Rate r(1);

  ros::ServiceClient client = n.serviceClient<project1::resetOdom>("reset_odom");
  project1::resetOdom srv;

  if (argc == 1)
  {
    srv.request.x.data = 0.0f;
    srv.request.y.data = 0.0f;
    srv.request.theta.data = 0.0f;
  }
  else
  {
    srv.request.x.data = atoll(argv[1]);
    srv.request.y.data = atoll(argv[2]);
    srv.request.theta.data = atoll(argv[3]) * 3.14f / 180.0f; // So that the user can choose theta in degrees
  }

  if (client.call(srv))
  {
    while (ros::ok() && resetpub.getNumSubscribers() == 0)
    {
      ROS_INFO("\nWaiting for subscriber\n");
      ros::spinOnce();
      r.sleep();
    }

    resetpub.publish(srv.response.newOdom);
    ROS_INFO("\n\n ODOMETRY RESET COMPLETED \n\n");
  }

  else
  {
    ROS_ERROR("\n\nFailed to call service reset_odom\n\n");
    return 1;
  }

  return 0;
}