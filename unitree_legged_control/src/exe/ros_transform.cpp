#include <ros/ros.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "PoseParse.h"
#include "control.h"

using namespace UNITREE_LEGGED_SDK;


PoseParse::void Transform(POSE* p, RPY* R, XYZ* t);{


}


int main(int argc, char** argv){
    ros::init(argc, argv, "quad transform");
    ros::NodeHandle nh;

    tf::TransformListener tf_ls;
    tf::TransformBroadcaster tf_br;


    tf::Transform quad_tf;

    // Hip Joints
    tf::Transform hj1;
    tf::Transform hj2;
    tf::Transform hj3;
    tf::Transform hj4;

    //Shoulder Joints
    tf::Transform sj1;
    tf::Transform sj2;
    tf::Transform sj3;
    tf::Transform sj4;

    //Ankle Joints
    tf::Transform aj1;
    tf::Transform aj2;
    tf::Transform aj3;
    tf::Transform aj4;

    //Foots .. feet lul
    tf::Transform f1;
    tf::Transform f2;
    tf::Transform f3;
    tf::Transform f4;


    //Setting initial positions and orientations.

    ros::Rate loop_rate(10);
    while(ros::ok()){
        //send transformations from body to world

        // {B} => {W}

        try{

            //Compute SE(3)
        }

        catch(tf::TransformException &e){

            //Catch errors eg: runtime error, zero div, sensor issue, etc..
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

}
