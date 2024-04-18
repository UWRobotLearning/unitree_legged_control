#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "mavros_msgs/RCIn.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>

using namespace UNITREE_LEGGED_SDK;


float footraiseheight = 0.f;
float bodyheight      = 0.f;
float r = 0.f,p = 0.f,y = 0.f;
uint8_t A1mode = 0;
uint8_t gaitType = 0;
uint8_t speedLevel = 0;
float xpos       = 0;
float ypos       = 0;


ros::Publisher pub_high;
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

void Custom::RobotControl() 
{
    udp.GetRecv(state);
    high_state_ros = state2rosMsg(state);
    pub_high.publish(high_state_ros);

    cmd.mode            = A1mode;
    cmd.gaitType        = gaitType;
    cmd.speedLevel      = speedLevel;
    cmd.footRaiseHeight = footraiseheight;
    cmd.bodyHeight      = bodyheight;
    cmd.position[0]     = xpos;
    cmd.position[1]     = ypos;
    cmd.euler[0]        = r;
    cmd.euler[1]        = p;
    cmd.euler[2]        = y;
    udp.SetSend(cmd);

}

void channel_cb(const mavros_msgs::RCIn::ConstPtr rc){
    if(rc->channels.empty())
    {   
        return;
    }
    /*
    Right joystick
    channel 0 for (left, right). channel space (2000, 1000) => sideSpeed cmd space (-1, +1)
    channel 1 for (up,   down) . channel space (2000, 1000) => forwardSpeed cmd space (-1, +1)

    Left joystick
    roll pitch yaw?
    channel 3 for (left, right) . channel space (1050, 1950) => sideSpeed cmd space (-1, +1)
    channel 2 for (up,   down)  . channel space (1950, 1050) => sideSpeed smd space (-1, +1)
    */
    A1mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    gaitType = 0;
    footraiseheight = 0;
    bodyheight = 0;
    xpos = 0.f;
    ypos = 0.f;
    

    /* Init rpy in place of eulers[3] (which is used in 3.3.1)*/
    
    r = 0.f;
    p = 0.f;
    y = 0.f;

    /*Init forward and side speed in place of velocity[2](which is used in 3.2)*/

    float xyfactor = rc->channels[11]/1950;
    float rpyFactor   = rc->channels[10]/1950;

    if (rc->channels[0] > 1600)
    {
        A1mode = 3;
        ypos = -xyfactor * (rc->channels[0] - 1501)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[0] < 1400)
    {
        A1mode = 3;
        ypos = -xyfactor * (rc->channels[0] - 1501)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[1] > 1600)
    {
        A1mode = 3;
        xpos = -xyfactor * (rc->channels[1] - 1501)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[1] < 1400)
    {
        A1mode = 3;
        xpos = -xyfactor * (rc->channels[1] - 1501)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if ((rc->channels[0] < 1600 && rc->channels[0] > 1400) && (rc->channels[1] < 1600 && rc->channels[1] > 1400)){
        A1mode = 1;
        xpos = 0.0;
        ypos = 0.0;
    }

    if (rc->channels[2] > 1600)
    {
        A1mode = 1;
        r = -rpyFactor * (rc->channels[2] - 1501)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[2] < 1400)
    {
        A1mode = 1;
        r = -rpyFactor * (rc->channels[2] - 1501)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[3] > 1600)
    {
        A1mode = 1;
        p = -rpyFactor * (rc->channels[3] - 1501)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[3] < 1400)
    {
        A1mode = 1;
        p = -rpyFactor * (rc->channels[3] - 1501)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

}


int main(int argc, char **argv) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    

    ros::init(argc, argv, "node_A1_control");
    

    ros::NodeHandle nh;


    ros::Subscriber sub_channel_cb;
    
    Custom custom(HIGHLEVEL);
    // InitEnvironment();

    sub_channel_cb = nh.subscribe("/mavros/rc/in", 1, channel_cb);
    pub_high       = nh.advertise<unitree_legged_msgs::HighState>("/high_state", 1);
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    ros::spin();

    return 0; 
}