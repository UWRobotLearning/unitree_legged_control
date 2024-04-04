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


float Fspeed = 0.f;
float Sspeed = 0.f;
float Yspeed = 0.f;
float footraiseheight = 0.f;
float bodyheight      = 0.f;
float r = 0.f,p = 0.f,y = 0.f;
uint8_t A1mode = 0;
uint8_t gaitType = 0;
uint8_t speedLevel = 0;


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
    cmd.velocity[0]     = Fspeed;
    cmd.velocity[1]     = Sspeed;
    cmd.yawSpeed        = Yspeed;
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
    
    Fspeed      = 0.f;
    Sspeed      = 0.f;
    Yspeed      = 0.f;

    /* Init rpy in place of eulers[3] (which is used in 3.3.1)*/
    
    r = 0.f;
    p = 0.f;
    y = 0.f;

    /*Init forward and side speed in place of velocity[2](which is used in 3.2)*/

    float speedFactor = (float)(rc->channels[11])/2500.0;
    float rpyFactor   = (float)(rc->channels[10])/2500.0;
    cout<<"speedfactor is "<<speedFactor<<endl;
    if (rc->channels[0] > 1500)
    {
        A1mode = 2;
        Sspeed = -speedFactor * (rc->channels[0] - 1500)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[0] < 1500)
    {
        A1mode = 2;
        Sspeed = -speedFactor * (rc->channels[0] - 1500)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[1] > 1500)
    {
        A1mode = 2;
        Fspeed = -speedFactor * (rc->channels[1] - 1500)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[1] < 1500)
    {
        A1mode = 2;
        Fspeed = -speedFactor * (rc->channels[1] - 1500)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if ((rc->channels[0] < 1500 && rc->channels[0] > 1500) && (rc->channels[1] < 1500 && rc->channels[1] > 1500)){
        A1mode = 1;
        Fspeed = 0.0;
        Sspeed = 0.0;
    }

    if (rc->channels[2] > 1500)
    {
        A1mode = 1;
        r = -rpyFactor * (rc->channels[2] - 1500)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[2] < 1500)
    {
        A1mode = 1;
        r = -rpyFactor * (rc->channels[2] - 1500)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[3] > 1500)
    {
        A1mode = 1;
        p = -rpyFactor * (rc->channels[3] - 1500)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

    if (rc->channels[3] < 1500)
    {
        A1mode = 1;
        p = -rpyFactor * (rc->channels[3] - 1500)/500;
        footraiseheight = 0.1;
        bodyheight      = 0.1;
    }

}


int main(int argc, char **argv) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    

    ros::init(argc, argv, "node_control_via_H12");
    

    ros::NodeHandle nh;


    ros::Subscriber sub_channel_cb;
    
    Custom custom(HIGHLEVEL);
    // InitEnvironment();

    sub_channel_cb = nh.subscribe("/mavros/rc/in", 1, channel_cb);
    pub_high       = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    ros::spin();

    return 0; 
}
