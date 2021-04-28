#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "project1/resetOdom.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>


bool reset(project1::resetOdom::Request& req,
           project1::resetOdom::Response& res)
{
    res.newOdom.header.stamp = ros::Time::now();
    
    float x = req.x.data;
    float y = req.y.data;
    float theta = req.theta.data;

    tf2::Quaternion q;

    res.newOdom.pose.pose.position.x = x;
    res.newOdom.pose.pose.position.y = y;
    res.newOdom.pose.pose.position.z = 0.0f;

    q.setRPY(0.0f, 0.0f, theta);
    res.newOdom.pose.pose.orientation.x = q.x();
    res.newOdom.pose.pose.orientation.y = q.y();
    res.newOdom.pose.pose.orientation.z = q.z();
    res.newOdom.pose.pose.orientation.w = q.w();

    res.newOdom.twist.twist.linear.x = 0.0f;
    res.newOdom.twist.twist.linear.y = 0.0f;
    res.newOdom.twist.twist.linear.z = 0.0f;
    res.newOdom.twist.twist.angular.x = 0.0f;
    res.newOdom.twist.twist.angular.y = 0.0f;
    res.newOdom.twist.twist.angular.z = 0.0f;

    // res.newOdom.header.stamp = ros::Time::now();
    // res.newOdom.point.x = 0.0f;
    // res.newOdom.point.y = 0.0f;
    // res.newOdom.point.z = 0.0f;
    ROS_INFO("moving to %f, %f, with angle %f",x, y, theta);
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_odom_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("reset_odom", reset);
  ros::spin();

  return 0;
}
