#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>

using namespace UNITREE_LEGGED_SDK;

static void OutputState();

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);

    // printf("%f %f %f %f %f\n", state.imu.rpy[1], state.imu.rpy[2], state.position[0], state.position[1], state.velocity[0]);
    // printf("%f %f %f\n", state.motorState[3].q, state.motorState[4].q, state.motorState[5].q);

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;

    /* Init rpy in place of eulers[3] (which is used in 3.3.1)*/
    
    cmd.roll = 0.f;
    cmd.pitch = 0.f;
    cmd.yaw = 0.f;

    /*Init forward and side speed in place of velocity[2](which is used in 3.2)*/
    cmd.forwardSpeed = 0.f;
    cmd.sideSpeed    = 0.f;


    if(motiontime > 0 && motiontime < 1000){
        cmd.mode = 1;
        cmd.roll = -0.3;
    }
    if(motiontime > 1000 && motiontime < 2000){
        cmd.mode = 1;
        cmd.roll = 0.3;
    }
    if(motiontime > 2000 && motiontime < 3000){
        cmd.mode = 1;
        cmd.pitch = -0.2;
    }
    if(motiontime > 3000 && motiontime < 4000){
        cmd.mode = 1;
        cmd.pitch = 0.2;
    }
    if(motiontime > 4000 && motiontime < 5000){
        cmd.mode = 1;
        cmd.yaw = -0.2;
    }
    if(motiontime > 5000 && motiontime < 6000){
        cmd.mode = 1;
        cmd.yaw = 0.2;
    }
    if(motiontime > 6000 && motiontime < 7000){
        cmd.mode = 1;
        cmd.bodyHeight = -0.2;
    }
    if(motiontime > 7000 && motiontime < 8000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.1;
    }
    if(motiontime > 8000 && motiontime < 9000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.0;
    }
    if(motiontime > 9000 && motiontime < 11000){
        cmd.mode = 5;
    }
    if(motiontime > 11000 && motiontime < 13000){
        cmd.mode = 6;
    }
    if(motiontime > 13000 && motiontime < 14000){
        cmd.mode = 0;
    }
    if(motiontime > 14000 && motiontime < 18000){
        cmd.mode = 2;
        cmd.forwardSpeed = 0.4f; // -1  ~ +1
        cmd.footRaiseHeight = 0.1;
        // printf("walk\n");
    }
    if(motiontime > 18000 && motiontime < 20000){
        cmd.mode = 0;
        cmd.forwardSpeed = 0;
    }
    if(motiontime > 20000 && motiontime < 24000){
        cmd.mode = 2;
        cmd.forwardSpeed = 0.2f; // -1  ~ +1
        cmd.bodyHeight = 0.1;
        // printf("walk\n");
    }

    if(motiontime>24000 ){
        cmd.mode = 1;
    }

    udp.SetSend(cmd);

}

int main(int argc, char **argv) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "ros_walk_udp");
    

    ros::NodeHandle nh;

    printf("Initiating ros_walk_udp..\n"); 

    ros::Publisher pub_high;
    unitree_legged_msgs::HighState high_state_ros;

    Custom custom(HIGHLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        high_state_ros = state2rosMsg(custom.state);
        pub_high.publish(high_state_ros);
        sleep(10);
    };

    return 0; 
}
