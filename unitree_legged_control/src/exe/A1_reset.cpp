#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "mavros_msgs/RCIn.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>


using namespace UNITREE_LEGGED_SDK;


unitree_legged_msgs::HighState high_state_ros;

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

void Custom::RobotReset() 
{
    udp.GetRecv(state);
    high_state_ros = state2rosMsg(state);
    pub_high.publish(high_state_ros);
    cmd.mode            = 8;
    udp.SetSend(cmd);
}

/*
So when the policy fails, it should send a signal that I will subscribe to all
the time. 
After receiving the signal, I will run the reset policy.
*/
int main(int argc, char **argv) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    

    ros::init(argc, argv, "node_A1_reset");
    

    ros::NodeHandle nh;


    // ros::Subscriber sub_h12_cb;
    // ros::Subscriber sub_mppi_cb;
    
    Custom custom(HIGHLEVEL);
    // InitEnvironment();

    // sub_h12_cb = nh.subscribe("/mavros/rc/in", 1, h12_cb);
    // sub_mppi_cb = nh.subscribe("/low_level_controller/dawg/control", 1, mppi_cb);

    // pub_high       = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotReset, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    ros::spin();

    return 0; 
}