#include "ros/ros.h"
#include "project1/OdomInt.h"
#include "project1/setOdom.h"
#include "nav_msgs/Odometry.h"
#include "project1/resetOdom.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define PI 3.14159265359

class odom
{
private:
    ros::NodeHandle n;

    ros::Subscriber velsub;
    ros::Publisher odompub;
    ros::Publisher custompub;

    ros::ServiceServer set_service;
    ros::ServiceServer reset_service;

    tf2::Quaternion q;
    nav_msgs::Odometry scout_odom;
    tf2_ros::TransformBroadcaster tb;
    geometry_msgs::TransformStamped transformStamped;

    project1::OdomInt odom_intmethod;
    
    // Dynamic reconfigure
    dynamic_reconfigure::Server<project1::parametersConfig> server;
    dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;

    float x_init;
    float y_init;
    float theta_init;
    float theta;
    float vx, wz;
    int integrationMethod;
    ros::Time curr_time, prev_time;
    float DT; // Sampling period

public:
    odom()
    {
        odompub   = n.advertise<nav_msgs::Odometry>("/our_odom", 100);
        custompub = n.advertise<project1::OdomInt>("/custom_odom", 100);
        velsub    = n.subscribe("/velpub", 100, &odom::callback, this);
        set_service   = n.advertiseService("set_odom", &odom::set, this);
        reset_service = n.advertiseService("reset_odom", &odom::reset, this);

        // Dynamic parameters callback. Whenever the integration method is
        // modified at runtime, call dynparamcallback()
        f = boost::bind(&odom::dynparamcallback, this, _1, _2);
        server.setCallback(f);

        // Retrieve the paramters from the paramater server.
        // They can be found in the launch file.
        if (n.hasParam("/odompubsub/x_init"))
            n.getParam("/odompubsub/x_init", x_init);
        else
            x_init = 0.0f;
        if (n.hasParam("/odompubsub/y_init"))
            n.getParam("/odompubsub/y_init", y_init);
        else
            y_init = 0.0f;
        if (n.hasParam("/odompubsub/theta_init"))
            n.getParam("/odompubsub/theta_init", theta_init);
        else
            theta_init = 0.0f;

        scout_odom.header.frame_id = "odom";
        scout_odom.child_frame_id = "base_link";

        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";

        scout_odom.pose.pose.position.x = x_init;
        scout_odom.pose.pose.position.y = y_init;
        scout_odom.pose.pose.position.z = 0.0f;

        theta = theta_init;
        q.setRPY(0.0f, 0.0f, theta);
        scout_odom.pose.pose.orientation.x = q.x();
        scout_odom.pose.pose.orientation.y = q.y();
        scout_odom.pose.pose.orientation.z = q.z();
        scout_odom.pose.pose.orientation.w = q.w();

        odom_intmethod.odo = scout_odom;

        ros::Time zero_time(0.0);
        curr_time = zero_time;
        prev_time = zero_time;
    }

    void callback(const geometry_msgs::TwistStamped::ConstPtr &vel)
    {
        scout_odom.header.stamp = ros::Time::now();
        transformStamped.header.stamp = ros::Time::now();

        vx = vel->twist.linear.x;
        wz = vel->twist.angular.z;

        scout_odom.twist.twist = vel->twist;

        computeOdom();

        transformStamped.transform.translation.x = scout_odom.pose.pose.position.x;
        transformStamped.transform.translation.y = scout_odom.pose.pose.position.y;
        transformStamped.transform.translation.z = scout_odom.pose.pose.position.z;
        transformStamped.transform.rotation = scout_odom.pose.pose.orientation;

        odom_intmethod.odo = scout_odom;

        if (integrationMethod == 0)
            odom_intmethod.int_method.data = "euler";
        if (integrationMethod == 1)
            odom_intmethod.int_method.data = "rk";

        tb.sendTransform(transformStamped);
        odompub.publish(scout_odom);
        custompub.publish(odom_intmethod);
    }

    void computeOdom()
    {
        curr_time = scout_odom.header.stamp;
        DT = curr_time.toSec() - prev_time.toSec();
        if(DT<0 && curr_time.toSec()>0)
            ROS_WARN("Time going backwards when computing odometry");
        prev_time = curr_time;

        if (integrationMethod == 0)
        {
            // EULER INTEGRATION
            scout_odom.pose.pose.position.x = scout_odom.pose.pose.position.x + vx * DT * cosf(theta);
            scout_odom.pose.pose.position.y = scout_odom.pose.pose.position.y + vx * DT * sinf(theta);
            theta = theta + wz * DT;
        }

        if (integrationMethod == 1)
        {
            // RUNGE-KUTTA INTEGRATION
            scout_odom.pose.pose.position.x = scout_odom.pose.pose.position.x + vx * DT * cosf(theta + wz * DT / 2.0f);
            scout_odom.pose.pose.position.y = scout_odom.pose.pose.position.y + vx * DT * sinf(theta + wz * DT / 2.0f);
            theta = theta + wz * DT;
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

    bool reset(project1::resetOdom::Request &req,
               project1::resetOdom::Response &res)
    {
        scout_odom.header.stamp = ros::Time::now();

        scout_odom.pose.pose.position.x = 0.0f;
        scout_odom.pose.pose.position.y = 0.0f;
        scout_odom.pose.pose.position.z = 0.0f;

        theta = 0.0f;
        q.setRPY(0.0f, 0.0f, theta);
        scout_odom.pose.pose.orientation.x = q.x();
        scout_odom.pose.pose.orientation.y = q.y();
        scout_odom.pose.pose.orientation.z = q.z();
        scout_odom.pose.pose.orientation.w = q.w();

        return true;
    }

    bool set(project1::setOdom::Request &req,
             project1::setOdom::Response &res)
    {
        scout_odom.header.stamp = ros::Time::now();

        float theta_new = req.th * 3.14f / 180.0f;    // so that the user can set the angle in degrees
                                                      // when calling the service with rosservice call

        scout_odom.pose.pose.position.x = req.x;
        scout_odom.pose.pose.position.y = req.y;
        scout_odom.pose.pose.position.z = 0.0f;

        theta = theta_new;
        q.setRPY(0.0f, 0.0f, theta);
        scout_odom.pose.pose.orientation.x = q.x();
        scout_odom.pose.pose.orientation.y = q.y();
        scout_odom.pose.pose.orientation.z = q.z();
        scout_odom.pose.pose.orientation.w = q.w();

        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odompubsub");
    odom odpub_sub;
    ros::spin();

    return 0;
}