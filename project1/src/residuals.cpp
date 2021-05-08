#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "project1/er_array.h"
#include <project1/parametersConfig.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#define PI 3.14159265359

using OD = nav_msgs::Odometry;
using GT = geometry_msgs::PoseStamped;

class residual
{
private:
    ros::NodeHandle n;

    project1::er_array od_error_array;
    project1::er_array gt_error_array;

    message_filters::Subscriber<OD> our_odom_sub;
    message_filters::Subscriber<OD> scout_odom_resampled_sub;
    message_filters::Subscriber<GT> gt_pose_resampled_sub;

    typedef message_filters::sync_policies::ApproximateTime<OD, OD> SyncPolicy_od;
    typedef message_filters::Synchronizer<SyncPolicy_od> Sync_od;
    boost::shared_ptr<Sync_od> sync_od;
    
    typedef message_filters::sync_policies::ApproximateTime<GT, OD> SyncPolicy_gt;
    typedef message_filters::Synchronizer<SyncPolicy_gt> Sync_gt;
    boost::shared_ptr<Sync_gt> sync_gt;

    OD real_time_od;
    GT real_time_gt;
    double dx_od, dy_od, dtheta_od, cumulateError_od;
    double dx_gt, dy_gt, dtheta_gt, cumulateError_gt;

    double gt_theta_offset, gt_x_offset, gt_y_offset, gt_map_rotation;
    int not_considered_messages = 935;
    int gt_msg_counter = 0;

public:
    ros::Publisher pub_od;      // publishes the errors /scout_odom - /our_odom
    ros::Publisher pub_gt;      // publishes the errors /gt_pose - /our_odom
    ros::Subscriber od_sub;     // listens to /scout_odom, republishes the message with a new timestamp
    ros::Subscriber gt_sub;     // listens to /gt_pose, republishes the message with a new timestamp
    ros::Publisher od_mirror;   // publishes the restamped message for the filter to synchronize
    ros::Publisher gt_mirror;   // publishes the restamped message for the filter to synchronize

    residual()
    {
        od_sub = n.subscribe("/scout_odom", 100, &residual::od_repub_callback, this);
        od_mirror = n.advertise<OD>("/real_time_od", 100);

        gt_sub = n.subscribe("/gt_pose", 100, &residual::gt_repub_callback, this);
        gt_mirror = n.advertise<GT>("/real_time_gt", 100);

        our_odom_sub.subscribe(n, "/our_odom", 100);
        scout_odom_resampled_sub.subscribe(n, "/real_time_od", 100);
        gt_pose_resampled_sub.subscribe(n, "/real_time_gt", 100);

        pub_od = n.advertise<project1::er_array>("/residuals_od", 100);
        pub_gt = n.advertise<project1::er_array>("/residuals_gt", 100);

        sync_od.reset(new Sync_od(SyncPolicy_od(100), scout_odom_resampled_sub, our_odom_sub));
        sync_od->registerCallback(boost::bind(&residual::od_error_callback, this, _1, _2));

        sync_gt.reset(new Sync_gt(SyncPolicy_gt(100), gt_pose_resampled_sub, our_odom_sub));
        sync_gt->registerCallback(boost::bind(&residual::gt_error_callback, this, _1, _2));

        if (n.hasParam("/gt_theta_offset"))
            n.getParam("/gt_theta_offset", gt_theta_offset);
        else
            gt_theta_offset = -281.97f;
        
        if (n.hasParam("/gt_x_offset"))
            n.getParam("/gt_x_offset", gt_x_offset);
        else
            gt_x_offset = -0.6840248f;

        if (n.hasParam("/gt_y_offset"))
            n.getParam("/gt_y_offset", gt_y_offset);
        else
            gt_y_offset =  0.5007557f;

        if (n.hasParam("/gt_map_rotation"))
            n.getParam("/gt_map_rotation", gt_map_rotation);
        else
            gt_y_offset =  -64.484f;

        cumulateError_od = 0;
        cumulateError_gt = 0;
    }

    void od_repub_callback(const OD::ConstPtr &scout_odom)
    {
        real_time_od = *scout_odom;
        real_time_od.header.stamp = ros::Time::now(); //real time is the same as scout but with header "now"

        od_mirror.publish(real_time_od);
    }

    void gt_repub_callback(const GT::ConstPtr &gt_pose)
    {
        real_time_gt = *gt_pose;
        real_time_gt.header.frame_id = "odom";

        // we offset and rotate x and y
        real_time_gt.pose.position.x =  gt_pose->pose.position.x*cos(gt_map_rotation*PI/180) - gt_pose->pose.position.y*sin(gt_map_rotation*PI/180) + gt_x_offset;
        real_time_gt.pose.position.y =  gt_pose->pose.position.x*sin(gt_map_rotation*PI/180) + gt_pose->pose.position.y*cos(gt_map_rotation*PI/180) + gt_y_offset;

        //quaternion to RPY
        tf::Quaternion temp_quaternion(
            gt_pose->pose.orientation.x,
            gt_pose->pose.orientation.y,
            gt_pose->pose.orientation.z,
            gt_pose->pose.orientation.w);
        tf::Matrix3x3 gt_matrix(temp_quaternion);
        double roll, pitch, yaw;
        gt_matrix.getRPY(roll, pitch, yaw);

        // we offset theta
        yaw -= gt_theta_offset*PI/180;
        temp_quaternion.setRPY(roll, pitch, yaw);

        real_time_gt.pose.orientation.x = temp_quaternion.x();
        real_time_gt.pose.orientation.y = temp_quaternion.y();
        real_time_gt.pose.orientation.z = temp_quaternion.z();
        real_time_gt.pose.orientation.w = temp_quaternion.w();

        //real time is the same as scout but with header "now"
        real_time_gt.header.stamp = ros::Time::now();
        
        gt_msg_counter++;

        gt_mirror.publish(real_time_gt);
    }

    //this recieve the odom from the bag, with the corrected time stamp, and the one from our odompubsub
    void od_error_callback(const OD::ConstPtr &scout_odom, const OD::ConstPtr &our_odom)
    {
        //quaternion to RPY
        tf::Quaternion temp_quaternion(
            scout_odom->pose.pose.orientation.x,
            scout_odom->pose.pose.orientation.y,
            scout_odom->pose.pose.orientation.z,
            scout_odom->pose.pose.orientation.w);
        tf::Matrix3x3 scout_matrix(temp_quaternion);
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

        dx_od = our_odom->pose.pose.position.x - scout_odom->pose.pose.position.x;
        dy_od = our_odom->pose.pose.position.y - scout_odom->pose.pose.position.y;
        dtheta_od = remainder(yaw_our, (2*PI)) - remainder(yaw_scout, (2*PI));
        dtheta_od = remainder(dtheta_od, (2*PI));
        if (dtheta_od > PI)
            dtheta_od -= 2*PI;
        cumulateError_od += sqrt(10000.0f*pow(dx_od, 2.0f) + 10000.0f*pow(dy_od, 2.0f) + pow(6/PI, 2.0f)*pow(dtheta_od, 2.0f));

        od_error_array.dx.data = dx_od;
        od_error_array.dy.data = dy_od;
        od_error_array.dtheta.data = dtheta_od;
        od_error_array.cumulateError.data = cumulateError_od;

        pub_od.publish(od_error_array);
    }

    //this recieve the odom from the bag, with the corrected time stamp, and the one from our odompubsub
    void gt_error_callback(const GT::ConstPtr& gt_pose, const OD::ConstPtr& our_odom)
    {
        //quaternion to RPY
        tf::Quaternion temp_quaternion(
            gt_pose->pose.orientation.x,
            gt_pose->pose.orientation.y,
            gt_pose->pose.orientation.z,
            gt_pose->pose.orientation.w);
        tf::Matrix3x3 gt_matrix(temp_quaternion);
        double roll, pitch, gt_yaw;
        gt_matrix.getRPY(roll, pitch, gt_yaw);

        tf::Quaternion our(
            our_odom->pose.pose.orientation.x,
            our_odom->pose.pose.orientation.y,
            our_odom->pose.pose.orientation.z,
            our_odom->pose.pose.orientation.w);
        tf::Matrix3x3 our_matrix(our);
        double our_yaw;
        our_matrix.getRPY(roll, pitch, our_yaw);

        dx_gt = our_odom->pose.pose.position.x - gt_pose->pose.position.x;
        dy_gt = our_odom->pose.pose.position.y - gt_pose->pose.position.y;
        dtheta_gt = remainder(our_yaw, (2*PI)) - remainder(gt_yaw, (2*PI));
        dtheta_gt = remainder(dtheta_od, (2*PI));
        if (dtheta_gt > PI)
            dtheta_gt -= 2*PI;

        
        if(gt_msg_counter < not_considered_messages)
        {
            cumulateError_gt = 0;
            gt_error_array.dx.data = 0;
            gt_error_array.dy.data = 0;
            gt_error_array.dtheta.data = 0;
            gt_error_array.cumulateError.data = 0;
        }
        else
        {
            cumulateError_gt += sqrt(10000.0f*pow(dx_gt, 2.0f) + 10000.0f*pow(dy_gt, 2.0f) + 0.25*pow(6/PI, 2.0f)*pow(dtheta_gt, 2.0f));
            gt_error_array.dx.data = dx_od;
            gt_error_array.dy.data = dy_od;
            gt_error_array.dtheta.data = dtheta_od;
            gt_error_array.cumulateError.data = cumulateError_od;
        }

        pub_gt.publish(gt_error_array);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Residuals");

    residual err;
    ros::spin();

    return 0;
}
