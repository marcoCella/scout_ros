#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "robotics_hw1/MotorSpeed.h" // type: float64 rpm
#include "nav_msgs/Odometry.h" // for error computation
#include "project1/er_array.h" // same
#include <tf/tf.h> // you get it
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// Just to have a cleaner code
using TWS = geometry_msgs::TwistStamped;
using MS  = robotics_hw1::MotorSpeed;
using OD  = nav_msgs::Odometry;


// Definition of constants. I've put them here and not in the parameter server
// since they're fixed, there's no need to modify them
static const float RAD = 0.1575f; // Wheel radius  [m]
static const float B = 0.583f;    // Real baseline [m]
static const float PI = 3.1416f;  // Hope it's self explanatory
static const float CHI = 1.75f;   // Scaling coefficient for the apparent baseline
static const float RATIO = 1.0f / 38.7f; // Transmisson ratio from the engine to the wheels

class pubv_subrpm
{

private:
    ros::NodeHandle n; 

    // Declare the subscribers for the wheels' rpms
    message_filters::Subscriber<MS> fr; 
    message_filters::Subscriber<MS> fl;
    message_filters::Subscriber<MS> rl;
    message_filters::Subscriber<MS> rr;

    //for error computation
    message_filters::Subscriber<OD> our_odom_sub;
    message_filters::Subscriber<OD> scout_odom_sub;
    project1::er_array residual;
    


    // Message filters delcaration, uncomment only one of the two policies
    //stypedef message_filters::sync_policies::ExactTime<MS, MS, MS, MS> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<MS, MS, MS, MS> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // same but for error computation
    //typedef message_filters::sync_policies::ExactTime<OD, OD> SyncPolicy_er;
    typedef message_filters::sync_policies::ApproximateTime<OD, OD> SyncPolicy_er;
    typedef message_filters::Synchronizer<SyncPolicy_er> Sync_er;
    boost::shared_ptr<Sync_er> sync_er;

public:
    TWS velocity;
    ros::Publisher pubv = n.advertise<geometry_msgs::TwistStamped>("/velpub", 1);

    ros::Publisher pub_er = n.advertise<project1::er_array>("/residual", 1);
    // Constructor
    pubv_subrpm()
    {
        fr.subscribe(n, "/motor_speed_fr", 1);
        fl.subscribe(n, "/motor_speed_fl", 1);
        rr.subscribe(n, "/motor_speed_rr", 1);
        rl.subscribe(n, "/motor_speed_rl", 1);

        sync.reset(new Sync(SyncPolicy(1), fr, fl, rr, rl));
        sync->registerCallback(boost::bind(&pubv_subrpm::callback, this, _1, _2, _3, _4));

        //yep, error computation again
        scout_odom_sub.subscribe(n, "/scout_odom", 1);
        our_odom_sub.subscribe(n, "/our_odom", 1);

        sync_er.reset(new Sync_er(SyncPolicy_er(1), scout_odom_sub, our_odom_sub));
        sync_er->registerCallback(boost::bind(&pubv_subrpm::callback_er, this, _1, _2));
    }

    void callback(const MS::ConstPtr& fr, const MS::ConstPtr& fl,
                  const MS::ConstPtr& rr, const MS::ConstPtr& rl)
    {
        float vl = (-(rl->rpm + fl->rpm) / 2.0f * RATIO) * RAD * 2.0f * PI / 60.0f; // Left wheels linear velocity  [m/s]
        float vr = ((fr->rpm + rr->rpm) / 2.0f * RATIO)  * RAD * 2.0f * PI / 60.0f; // Right wheels linear velocity [m/s]

        velocity.header.stamp = ros::Time::now();
        velocity.twist.linear.x  = (vl + vr) / 2.0f; // Linear velocity of the robot  [m/s]
        velocity.twist.linear.y  = 0.0f;
        velocity.twist.linear.z  = 0.0f;
        velocity.twist.angular.x = 0.0f;
        velocity.twist.angular.y = 0.0f;
        velocity.twist.angular.z = (-vl + vr) / (B * CHI); // Angular velocity of the robot [rad/s]

        pubv.publish(velocity);
    }

    // don't give importance to the fact that this is called velpubsub and not vel&positionerrorpubsub
    void callback_er(const OD::ConstPtr& scout_odom, const OD::ConstPtr& our_odom)
    {
        ROS_INFO("%f",scout_odom->pose.pose.orientation.x);

        tf::Quaternion scout(
            scout_odom->pose.pose.orientation.x,
            scout_odom->pose.pose.orientation.y,
            scout_odom->pose.pose.orientation.z,
            scout_odom->pose.pose.orientation.w);
        tf::Matrix3x3 scout_matrix(scout);
        double roll, pitch, yaw_scout;
        scout_matrix.getRPY(roll, pitch, yaw_scout);
        tf::Quaternion our(
            our_odom->pose.pose.orientation.x,
            our_odom->pose.pose.orientation.y,
            our_odom->pose.pose.orientation.z,
            our_odom->pose.pose.orientation.w);
        tf::Matrix3x3 our_matrix(our);
        double yaw_our;
        our_matrix.getRPY(roll, pitch, yaw_our);

        float dx = our_odom->pose.pose.position.x - scout_odom->pose.pose.position.x;
        float dy = our_odom->pose.pose.position.y - scout_odom->pose.pose.position.y;
        float dtheta = yaw_our - yaw_scout;
        float dtot = sqrt(pow(dx, 2.0f)+pow(dy, 2.0f));

        residual.dx.data = dx;
        residual.dy.data = dy;
        residual.dtheta.data = dtheta;
        residual.dtot.data = dtot;

        ROS_INFO("%f", dx);
        
        pub_er.publish(residual);
        
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velpubsub");
    pubv_subrpm pub_sub;
    ros::spin();

    return 0;
}