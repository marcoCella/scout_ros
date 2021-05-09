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
// here are the parameter for rotrotranslation from world to odom
#define GT_X_OFFSET -0.6683338288
#define GT_Y_OFFSET -0.0377706864
#define GT_Z_OFFSET  0.3290136176
#define GT_X_Q_ROT  -0.0079870560
#define GT_Y_Q_ROT   0.0181781832
#define GT_Z_Q_ROT  -0.6260676536
#define GT_W_Q_ROT   0.7795158906

// filter 
#define ERROR_FILTER_LENGTH 100

// the initial points will not be considered to compute the errors
#define GT_NOT_CONSIDERED_MSGS 41
#define  OD_NOT_CONSIDERED_MSGS 5
   

using OD = nav_msgs::Odometry;
using GT = geometry_msgs::PoseStamped;


/* This node does the following things:
  - subscribes to /scout_odom and republishes with the current ros time in /od_real_time 
  - subscribes to /gt_pose it translates and rotates the reference system (from world to odom) and republishes
    with the current ros time in /gt_real_time 
  - uses a filter to match messages from /our_odom (the odometry we compute) and from /od_real_time and computes the
    errors (in particular it computes deltaX, deltaY, deltaTheta and a cumulate error)
    cumulateError is a filtered error, which means that it is the average of the last ERROR_FILTER_LENGTH quadratic
    errors. Each quadratic error is the square root of the sum of the square of deltaX, deltaY, deltaTheta (each
    multiplied by a particular quadratic weight)
  - uses a filter to match messages from /our_odom (the odometry we compute) and from /gt_real_time and computes the
    errors
  - publishes the errors in two different topics: /residuals_od for the errors wrt the /scout_odom pose and /residuals_gt
    for the errors wrt the /gt_pose pose
*/

class residual
{
private:
    ros::NodeHandle n;

    project1::er_array od_error_array; // errors for /scout_odom
    project1::er_array gt_error_array; // errors for /gt_pose

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
    GT prev_gt; // used since many messages are equal and so will be discarded
    OD prev_od;

    // errors
    double dx_od, dy_od, dtheta_od, cumulateError_od;
    double dx_gt, dy_gt, dtheta_gt, cumulateError_gt;

    // here are the vectors used to store previous sum of errors (so that we can filter them)
    double lastErrors_od[ERROR_FILTER_LENGTH] = {};
    double lastErrors_gt[ERROR_FILTER_LENGTH] = {};

    // incoming message counters
    int gt_msg_counter = 0; int od_msg_counter = 0;

    // here are the variables used to rototranslate from wolrd ref to odom ref
    double temp_x, temp_y, temp_z;
    // quaternion to rot matrix
    tf::Quaternion gtTOsc_quat;
    tf::Matrix3x3 gtTOsc_rot; 

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

        gtTOsc_quat.setX(GT_X_Q_ROT);
        gtTOsc_quat.setY(GT_Y_Q_ROT);
        gtTOsc_quat.setZ(GT_Z_Q_ROT);
        gtTOsc_quat.setW(GT_W_Q_ROT);
        tf::Matrix3x3 tmp_mat(gtTOsc_quat);
        gtTOsc_rot = tmp_mat;        
    }

    void od_repub_callback(const OD::ConstPtr &scout_odom)
    {
        // if the position hasn't changed since the last "good" message, we will not publish any new message
        if ( od_msg_counter != 0 && pow(prev_od.pose.pose.position.x-scout_odom->pose.pose.position.x, 2.0)+pow(prev_od.pose.pose.position.y-scout_odom->pose.pose.position.x, 2.0) < 0.00001 )
            return;
        
        prev_od = *scout_odom;
        real_time_od = *scout_odom;
        real_time_od.header.stamp = ros::Time::now(); //real time is the same as scout but with header "now"

        od_msg_counter++;

        od_mirror.publish(real_time_od);
    }

    void gt_repub_callback(const GT::ConstPtr& gt_pose)
    {
        // if the position hasn't changed since the last "good" message, we will not publish any new message
        if ( gt_msg_counter != 0 && pow(prev_gt.pose.position.x-gt_pose->pose.position.x, 2.0)+pow(prev_gt.pose.position.y-gt_pose->pose.position.y, 2.0) < 0.00001 )
            return;
        
        prev_gt = *gt_pose;
        real_time_gt = *gt_pose;
        real_time_gt.header.frame_id = "odom";

        // Now we rototranslate /gt_pose from world to odom

        // we offset xyz
        temp_x = gt_pose->pose.position.x - GT_X_OFFSET;
        temp_y = gt_pose->pose.position.y - GT_Y_OFFSET;
        temp_z = gt_pose->pose.position.z - GT_Z_OFFSET;       

        // we rotatate the gt_pose pose
        real_time_gt.pose.position.x = gtTOsc_rot[0][0]*temp_x+gtTOsc_rot[0][1]*temp_y+gtTOsc_rot[0][2]*temp_z;
        real_time_gt.pose.position.y = gtTOsc_rot[1][0]*temp_x+gtTOsc_rot[1][1]*temp_y+gtTOsc_rot[1][2]*temp_z;
        real_time_gt.pose.position.z = gtTOsc_rot[2][0]*temp_x+gtTOsc_rot[2][1]*temp_y+gtTOsc_rot[2][2]*temp_z;

        // we rotate the gt_pose quaternion
        real_time_gt.pose.orientation.x = gtTOsc_quat.x()*gt_pose->pose.orientation.w + gtTOsc_quat.w()*gt_pose->pose.orientation.x + gtTOsc_quat.y()*gt_pose->pose.orientation.z - gtTOsc_quat.z()*gt_pose->pose.orientation.y;   // x component
        real_time_gt.pose.orientation.y = gtTOsc_quat.w()*gt_pose->pose.orientation.y - gtTOsc_quat.x()*gt_pose->pose.orientation.z + gtTOsc_quat.y()*gt_pose->pose.orientation.w + gtTOsc_quat.z()*gt_pose->pose.orientation.x;   // y component
        real_time_gt.pose.orientation.z = gtTOsc_quat.w()*gt_pose->pose.orientation.z + gtTOsc_quat.x()*gt_pose->pose.orientation.y - gtTOsc_quat.y()*gt_pose->pose.orientation.x + gtTOsc_quat.z()*gt_pose->pose.orientation.w;   // z component
        real_time_gt.pose.orientation.w = gtTOsc_quat.w()*gt_pose->pose.orientation.w - gtTOsc_quat.x()*gt_pose->pose.orientation.x - gtTOsc_quat.y()*gt_pose->pose.orientation.y - gtTOsc_quat.z()*gt_pose->pose.orientation.z;   // w component

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

        if (od_msg_counter < OD_NOT_CONSIDERED_MSGS)
        {
            cumulateError_od = 0.0;

            od_error_array.dx.data = 0.0;
            od_error_array.dy.data = 0.0;
            od_error_array.dtheta.data = 0.0;
            od_error_array.cumulateError.data = 0.0;
        }
        else
        {
            for(int i = 1; i < ERROR_FILTER_LENGTH; i++)
                lastErrors_od[i-1] = lastErrors_od[i];
            
            lastErrors_od[ERROR_FILTER_LENGTH-1] = sqrt(10000.0f*pow(dx_od, 2.0f) + 10000.0f*pow(dy_od, 2.0f) + pow(6/PI, 2.0f)*pow(dtheta_od, 2.0f));
            
            cumulateError_od = 0;
            for(int i = 0; i < ERROR_FILTER_LENGTH; i++)
                cumulateError_od += lastErrors_od[i];
            cumulateError_od = cumulateError_od / (float) ERROR_FILTER_LENGTH;

            od_error_array.dx.data = dx_od;
            od_error_array.dy.data = dy_od;
            od_error_array.dtheta.data = dtheta_od;
            od_error_array.cumulateError.data = cumulateError_od;
        }
        

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
        dtheta_gt = remainder(dtheta_gt, (2*PI));
        if (dtheta_gt > PI)
            dtheta_gt -= 2*PI;

        
        if(gt_msg_counter < GT_NOT_CONSIDERED_MSGS)
        {
            cumulateError_gt = 0;
            gt_error_array.dx.data = 0;
            gt_error_array.dy.data = 0;
            gt_error_array.dtheta.data = 0;
            gt_error_array.cumulateError.data = 0;
        }
        else
        {
            for(int i = 1; i < ERROR_FILTER_LENGTH; i++)
                lastErrors_gt[i-1] = lastErrors_gt[i];
        
            lastErrors_gt[ERROR_FILTER_LENGTH-1] = sqrt(10000.0f*pow(dx_gt, 2.0f) + 10000.0f*pow(dy_gt, 2.0f) + 0.25*pow(6/PI, 2.0f)*pow(dtheta_gt, 2.0f));
        
            cumulateError_gt = 0;
            for(int i = 0; i < ERROR_FILTER_LENGTH; i++)
                cumulateError_gt += lastErrors_gt[i];
            cumulateError_gt = cumulateError_gt / (float) ERROR_FILTER_LENGTH;
            
            gt_error_array.dx.data = dx_gt;
            gt_error_array.dy.data = dy_gt;
            gt_error_array.dtheta.data = dtheta_gt;
            gt_error_array.cumulateError.data = cumulateError_gt;
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
