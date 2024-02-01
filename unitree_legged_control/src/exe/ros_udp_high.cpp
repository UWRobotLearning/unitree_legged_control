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


    //UDP low_udp;
    UDP high_udp;
    Safety safe;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:

    // Custom(uint8_t level): safe(LeggedType::Aliengo), low_udp(level) {
    //     low_udp.InitCmdData(low_cmd);
    // }

    Custom()
        : 
        safe(LeggedType::Aliengo),
        high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
        //low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        //high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);

    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        //high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        //low_udp.SetSend(low_cmd);
        //low_udp.Send();
    }

    void lowUdpRecv()
    {
        // low_udp.Recv();
        // low_udp.GetRecv(low_state);

        // OutputState();
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
        OutputState();
    }
};


unitree_legged_msgs::HighState high_state_ros;

ros::Publisher pub_high;

Custom custom;

static inline void OutputState(){

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high.publish(high_state_ros);

}

long high_count = 0;
long low_count = 0;

// void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
// {
//     printf("highCmdCallback is running !\t%ld\n", ::high_count);

//     custom.high_cmd = rosMsg2Cmd(msg);

//     unitree_legged_msgs::HighState high_state_ros;

//     high_state_ros = state2rosMsg(custom.high_state);

//     pub_high.publish(high_state_ros);

//     printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
// }

// void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
// {

//     printf("lowCmdCallback is running !\t%ld\n", low_count);

//     //custom.low_cmd = rosMsg2Cmd(msg);

//     unitree_legged_msgs::LowState low_state_ros;

//     low_state_ros = state2rosMsg(custom.low_state);

//     pub_low.publish(low_state_ros);

//     printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
// }

// LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
//     LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
//     LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

//     loop_udpSend.start();
//     loop_udpRecv.start();
//     loop_control.start();


int main(int argc, char **argv)
{
    printf("Initiating ros_udp...\n");
    ros::init(argc, argv, "ros_udp");

    ros::NodeHandle nh;

    printf("Initiating ros_udp..\n"); 


    //custom.lowUdpSend();
    custom.high_cmd.mode = 2;
    custom.high_udp.SetSend(custom.high_cmd);

    if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        printf("HIGHLEVEL \n");
        // sub_high = nh.subscribe("high_cmd", 1, highCmdCallback);
        pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

        LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
        LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        ros::spin();

        //printf("low level runing!\n");
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    return 0;
}
