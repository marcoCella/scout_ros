#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

static const float RATE = 50.0f; // Sampling frequency given by "rostopic hz /velpub"
static const float T = 1.0f / RATE; // Sampling period

class odom
{
private:
    ros::NodeHandle n;
    ros::Subscriber velsub;
    ros::Subscriber resetsub;
    ros::Publisher  odompub;
    tf2::Quaternion q;
    tf2_ros::TransformBroadcaster tb;
    geometry_msgs::TransformStamped transformStamped;
    nav_msgs::Odometry scout_odom;

    dynamic_reconfigure::Server<project1::parametersConfig> server;
    dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;

    float x_init;
    float y_init;
    float theta_init;
    float theta;
    float vx, wz;
    int integrationMethod;

public:
    odom()
    {
        odompub  = n.advertise<nav_msgs::Odometry>("/odom", 1);
        velsub   = n.subscribe("/velpub", 1, &odom::callback, this);
        resetsub = n.subscribe("/newodom", 1, &odom::newOdomCallback, this);

        f = boost::bind(&odom::dynparamcallback, this, _1, _2);
        server.setCallback(f);

        n.getParam("/x_init", x_init);
        n.getParam("/y_init", y_init);
        n.getParam("/theta_init", theta_init);

        scout_odom.header.frame_id = "odom";
        scout_odom.child_frame_id  = "base_link";
        
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id  = "base_link";

        scout_odom.pose.pose.position.x = x_init;
        scout_odom.pose.pose.position.y = y_init;
        scout_odom.pose.pose.position.z = 0.0f;

        theta = theta_init;
        q.setRPY(0.0f, 0.0f, theta_init);
        scout_odom.pose.pose.orientation.x = q.x();
        scout_odom.pose.pose.orientation.y = q.y();
        scout_odom.pose.pose.orientation.z = q.z();
        scout_odom.pose.pose.orientation.w = q.w();
    }

    void callback(const geometry_msgs::TwistStamped::ConstPtr& vel)
    {
        scout_odom.header.stamp       = ros::Time::now();
        transformStamped.header.stamp = ros::Time::now();

        vx = vel->twist.linear.x; 
        wz = vel->twist.angular.z;
        
        scout_odom.twist.twist.linear.x  = vx;
        scout_odom.twist.twist.linear.y  = vel->twist.linear.y;
        scout_odom.twist.twist.linear.z  = vel->twist.linear.z;
        scout_odom.twist.twist.angular.x = vel->twist.angular.x;
        scout_odom.twist.twist.angular.y = vel->twist.angular.y;
        scout_odom.twist.twist.angular.z = wz;

        computeOdom();

        transformStamped.transform.translation.x = scout_odom.pose.pose.position.x;
        transformStamped.transform.translation.y = scout_odom.pose.pose.position.y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x    = q.x();
        transformStamped.transform.rotation.y    = q.y();
        transformStamped.transform.rotation.z    = q.z();
        transformStamped.transform.rotation.w    = q.w();

        tb.sendTransform(transformStamped);
        odompub.publish(scout_odom);

    }

    void computeOdom()
    {
        if(integrationMethod == 0)
        {
            // EULER INTEGRATION
            scout_odom.pose.pose.position.x = scout_odom.pose.pose.position.x + vx * T * cosf(theta);
            scout_odom.pose.pose.position.y = scout_odom.pose.pose.position.y + vx * T * sinf(theta);
            theta = theta + wz * T;
        }

        if(integrationMethod == 1)
        {
            // RUNGE-KUTTA INTEGRATION
            scout_odom.pose.pose.position.x = scout_odom.pose.pose.position.x + vx * T * cosf(theta + wz * T / 2.0f);
            scout_odom.pose.pose.position.y = scout_odom.pose.pose.position.y + vx * T * sinf(theta + wz * T / 2.0f);
            theta = theta + wz * T;
        }

        q.setRPY(0.0f, 0.0f, theta);
        scout_odom.pose.pose.orientation.x = q.x();
        scout_odom.pose.pose.orientation.y = q.y();
        scout_odom.pose.pose.orientation.z = q.z();
        scout_odom.pose.pose.orientation.w = q.w();
    }

    void dynparamcallback(project1::parametersConfig &config, uint32_t level) 
    { 
        integrationMethod = config.intmethod;

        if (integrationMethod == 0)
            ROS_INFO("\n\n Integration method: Euler\n\n");

        if (integrationMethod == 1)
            ROS_INFO("\n\n Integration method: Runge - Kutta\n\n");
    }

    void newOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        scout_odom.header.stamp         = msg->header.stamp;
        scout_odom.pose.pose.position.x = msg->pose.pose.position.x;
        scout_odom.pose.pose.position.y = msg->pose.pose.position.y;
        scout_odom.pose.pose.position.z = msg->pose.pose.position.z;

        odompub.publish(scout_odom);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odompubsub");
    odom odpub_sub;
    ros::spin();

    return 0;
}