#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "robotics_hw1/MotorSpeed.h" // type: float64 rpm
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// Just to have a cleaner code
using TWS = geometry_msgs::TwistStamped;
using MS  = robotics_hw1::MotorSpeed;


// Definition of constants. I've put them here and not in the parameter server
// since they're fixed, there's no need to modify them
const float RAD = 0.1575f; // Wheel radius  [m]
const float B = 0.583f;    // Real baseline [m]
const float PI = 3.1416f;  // Hope it's self explanatory

class pubv_subrpm
{

private:
    ros::NodeHandle n; 

    // Declare the subscribers for the wheels' rpms
    message_filters::Subscriber<MS> fr; 
    message_filters::Subscriber<MS> fl;
    message_filters::Subscriber<MS> rl;
    message_filters::Subscriber<MS> rr;

    // Message filters delcaration, uncomment only one of the two policies
    typedef message_filters::sync_policies::ExactTime<MS, MS, MS, MS> SyncPolicy;
    //typedef message_filters::sync_policies::ApproximateTime<MS, MS, MS, MS> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    float RATIO; // Transmisson ratio from the engine to the wheels
    float CHI; // Scaling coefficient for the apparent baseline

public:
    TWS velocity;
    ros::Publisher pubv = n.advertise<geometry_msgs::TwistStamped>("/velpub", 1);

    // Constructor
    pubv_subrpm()
    {
        fr.subscribe(n, "/motor_speed_fr", 100);
        fl.subscribe(n, "/motor_speed_fl", 100);
        rr.subscribe(n, "/motor_speed_rr", 100);
        rl.subscribe(n, "/motor_speed_rl", 100);
        

        sync.reset(new Sync(SyncPolicy(100), fr, fl, rr, rl));
        sync->registerCallback(boost::bind(&pubv_subrpm::callback, this, _1, _2, _3, _4));

        float temp;
        if (n.hasParam("/CHI"))
            n.getParam("/CHI", CHI);
        else
            CHI = 1.75f;
        if (n.hasParam("/Inv_RATIO"))
        {
            n.getParam("/Inv_RATIO", temp);
            RATIO = 1.0f /temp;
        }
        else
            RATIO = 1.0f / 38.7f;
    }

    void callback(const MS::ConstPtr& fr, const MS::ConstPtr& fl,
                  const MS::ConstPtr& rr, const MS::ConstPtr& rl)
    {
        // we average front and rear wheels
        float vl = ((-rl->rpm -fl->rpm)/2* RATIO) * RAD * 2.0f * PI / 60.0f; // Left wheels linear velocity  [m/s]
        float vr = ((fr->rpm+rr->rpm)/2 * RATIO)  * RAD * 2.0f * PI / 60.0f; // Right wheels linear velocity [m/s]

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