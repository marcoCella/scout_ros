#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "robotics_hw1/MotorSpeed.h" // type: float64 rpm
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using TWS = geometry_msgs::TwistStamped;
using MS  = robotics_hw1::MotorSpeed;

static const float RAD = 0.1575f; // Wheel radius  [m]
static const float B = 0.583f;    // Real baseline [m]
static const float PI = 3.1416f;  // Hope it's self explanatory
static const float CHI = 1.75f;    // Scaling coefficient for the apparent baseline
static const float RATIO = 1.0f / 38.7f; // Transmisson ratio from the engine to the wheels

class pubv_subrpm
{

private:
    ros::NodeHandle n;

    message_filters::Subscriber<MS> fr;
    message_filters::Subscriber<MS> fl;
    message_filters::Subscriber<MS> rl;
    message_filters::Subscriber<MS> rr;

    typedef message_filters::sync_policies::ExactTime<MS, MS, MS, MS> SyncPolicy;
    //typedef message_filters::sync_policies::ApproximateTime<MS, MS, MS, MS> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    TWS velocity;
    ros::Publisher pubv = n.advertise<geometry_msgs::TwistStamped>("/velpub", 1);

    pubv_subrpm()
    {
        fr.subscribe(n, "/motor_speed_fr", 1);
        fl.subscribe(n, "/motor_speed_fl", 1);
        rr.subscribe(n, "/motor_speed_rr", 1);
        rl.subscribe(n, "/motor_speed_rl", 1);

        sync.reset(new Sync(SyncPolicy(1), fr, fl, rr, rl));

        sync->registerCallback(boost::bind(&pubv_subrpm::callback, this, _1, _2, _3, _4));
    }

    void callback(const MS::ConstPtr& fr, const MS::ConstPtr& fl,
                  const MS::ConstPtr& rr, const MS::ConstPtr& rl)
    {
        float vl = (-rl->rpm * RATIO) * RAD * 2.0f * PI / 60.0f; // Left wheels linear velocity  [m/s]
        float vr = (fr->rpm * RATIO)  * RAD * 2.0f * PI / 60.0f; // Right wheels linear velocity [m/s]

        velocity.header.stamp = ros::Time::now();
        velocity.twist.linear.x  = (vl + vr) / 2.0f; // Linear velocity of the robot  [m/s]
        velocity.twist.linear.y  = 0.0f;
        velocity.twist.linear.z  = 0.0f;
        velocity.twist.angular.x = 0.0f;
        velocity.twist.angular.y = 0.0f;
        velocity.twist.angular.z = (-vl + vr) / (B * CHI); // Angular velocity of the robot [rad/s]

        pubv.publish(velocity);
    }

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "velpubsub");
    pubv_subrpm pub_sub;
    ros::spin();

    return 0;
}