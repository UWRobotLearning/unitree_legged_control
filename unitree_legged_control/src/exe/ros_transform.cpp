#include <ros/ros.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include "StateEstimation.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;



void LowStateCallBack(const unitree_legged_msgs :: LowState :: ConstPtr& msg>){

}
void InitPose(POSE* P, float mean, float std){
    
}


int main(int argc, char** argv){
    ros::init(argc, argv, "quad transform");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster tf_broadcaster;


    ros::Subscriber Sub = nh.subscribe("/low_state", 10, LowStateCallBack);
    
    
    //Setting initial positions and orientations.
    for (const auto & [ PoseFrame, PoseValue ] : PoseVector){
            InitPose(&PoseValue, 0., 0.);
    
    }

    ros::Rate loop_rate(10);
    while(ros::ok()){
        //send transformations from body to world

        // {B} => {W}
        for (const auto & [ PoseFrame, PoseValue ] : PoseVector){

            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped.header.stamp = ros::Time::now();
            transform_stamped.header.frame_id = "base_frame";
            transform_stamped.child_frame_id  =  PoseFrame;

            transform_stamped.transform.translation.x = PoseValue.t->x;
            transform_stamped.transform.translation.y = PoseValue.t->y;
            transform_stamped.transform.translation.z = PoseValue.t->Z;

            tf2::Quaternion quaternion;

            quaternion.setRPY(PoseValue.R->roll, PoseValue.R->pitch, PoseValue.R->yaw)
            transform_stamped.transform.rotation.x =  quaternion.x();
            transform_stamped.transform.rotation.y =  quaternion.x();
            transform_stamped.transform.rotation.z =  quaternion.x();
            transform_stamped.transform.rotation.w =  quaternion.x();

            tf_broadcaster.sendTransform(transform_stamped);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;

}
