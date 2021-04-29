#include "ros/ros.h"
#include "nav_msgs/Odometry.h" 
#include "project1/er_array.h" 
#include <tf/tf.h> 
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using OD  = nav_msgs::Odometry;

class residual

{
private:
    ros::NodeHandle n;

    
    project1::er_array error_array;
    message_filters::Subscriber<OD> our_odom_sub;
    message_filters::Subscriber<OD> scout_odom_sub;
    

    typedef message_filters::sync_policies::ApproximateTime<OD, OD> SyncPolicy_er;
    typedef message_filters::Synchronizer<SyncPolicy_er> Sync_er;
    boost::shared_ptr<Sync_er> sync_er;


public:

    float dx, dy, dtheta, dtot;
    ros::Publisher pub_er = n.advertise<project1::er_array>("/residual", 1000);
    ros::Subscriber re_timer;      //listen to scout odom, republih the message with a new timestamp
    ros::Publisher mirror; // publish the restamped message for the filter to synchronize

    residual()
    {
        re_timer = n.subscribe("/scout_odom",  1000, &residual::time_callback, this);
        
        scout_odom_sub.subscribe(n, "/real_time_odom", 1000);
        our_odom_sub.subscribe(n, "/our_odom", 1000);

        sync_er.reset(new Sync_er(SyncPolicy_er(10), scout_odom_sub, our_odom_sub));
        sync_er->registerCallback(boost::bind(&residual::callback_er, this, _1, _2));
    }

    void time_callback(const OD::ConstPtr& scout_odom)
    {
        mirror = n.advertise<OD>("/real_time_odom", 1000);
        OD real_time_odom;
        real_time_odom = *scout_odom;
        real_time_odom.header.stamp = ros::Time::now();  //real time is the same as scout but with header "now"

        mirror.publish(real_time_odom);
    }

    void callback_er(const OD::ConstPtr& scout_odom, const OD::ConstPtr& our_odom)
    {
        ROS_INFO("I'm residuating");
        
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

        error_array.dx.data = dx;
        error_array.dy.data = dy;
        error_array.dtheta.data = dtheta;
        error_array.dtot.data = dtot;

        pub_er.publish(error_array);
        
    }
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "Residuals");

    residual err;
    ros::spin();

    return 0;  
    
}

