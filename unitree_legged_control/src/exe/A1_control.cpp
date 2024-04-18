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

float Fspeed          = 0.f;
float Sspeed          = 0.f;
float Yspeed          = 0.f;
float footraiseheight = 0.f;
float bodyheight      = 0.f;
float r = 0.f, p = 0.f, y = 0.f;
uint8_t A1mode = 0;
uint8_t gaitType = 0;
uint8_t speedLevel = 0;

bool MPPI_flag = false;
bool ODT_flag  = false;


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

void h12_cb(const mavros_msgs::RCIn::ConstPtr rc){
    if(rc->channels.empty())
    {   
        return;
    }
    /*
    TBD: 
    if mppi button clicked, set the mppi flag as high

    if ODT button clicked, set the ODT flag as high
    
    */


    /*
    Right joystick
    forward/backward left/right speed
    channel 0 for (left, right). channel space (2000, 1000) => sideSpeed cmd space (-1, +1)
    channel 1 for (up,   down) . channel space (2000, 1000) => forwardSpeed cmd space (-1, +1)

    Left joystick
    pitch yawspeed
    channel 3 for yawspeed . channel space (1050, 1950) => yawspeed cmd space (-2.8, +2.8)
    channel 2 for pitch (up,   down)  . channel space (1950, 1050) => sideSpeed smd space (-1, +1)
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

    // Changing gains using RC
    float speedFactor = (float)(rc->channels[11])/2000.0;
    float rpyFactor   = (float)(rc->channels[10])/2000.0;
    
    //Check if the Robot has halted
    if ((rc->channels[0] < 1550  && rc->channels[0] > 1450) && (rc->channels[1] < 1550 && rc->channels[1] > 1450))
    {
        A1mode = 0;
        Fspeed = 0.0;
        Sspeed = 0.0;
        Yspeed = 0.0;
    }
    //yawspeed
       if (rc->channels[3] > 1550 || rc->channels[3] < 1450)
    {
        A1mode = 2;
        //-2.8 to +2.8 rad/s
        Yspeed = -rpyFactor * (rc->channels[3] - 1500)/100;
    }
    // Side speed
    if (rc->channels[0] > 1550 || rc->channels[0] < 1450)
    {
        A1mode = 2;
        Sspeed = speedFactor * (rc->channels[0] - 1500)/500;
    }
    //Front speed
    if (rc->channels[1] > 1550 || rc->channels[1] < 1450)
    {
        A1mode = 2;
        Fspeed = speedFactor * (rc->channels[1] - 1500)/500;
    }

    // pitch
    if (rc->channels[2] > 1550 || rc->channels[2] < 1450)
    {
        A1mode = 1;
        p = -rpyFactor * (rc->channels[2] - 1500)/500;
    }
    //roll not used
}

/*

Subscribe to low_level_controller/dawg/control
for getting control outputs from MPPI

msg => steering_angle, speed
    where 
    steering_angle (theta)  = ctrl[0] * self.steering_max
    speed (S)          = ctrl[1] * self.throttle_to_wheelspeed

    steering_max =  0.488 from Dynamics_config from dawg_mppi.yaml file
    throttle_to_wheelspeed = 17.0 from Dynamics_config from dawg_mppi.yaml file


    Wheeled to dawg control conversion 

    (theta, S) => [wheeled to dawg] => [YawSpeed, Fspeed, Sspeed]

       desired theta -> x -> PD -> YawSpeed -> Dawg
                        ^                       |
                        |                       |
                         - - - -  - - - - - - - 
                            initial theta

    wheelspeed = ([0, 17] - 8.5)/8.5=>  =>[-1, 1]
    S (wheelspeed) =>  [Scaling and Normalize] => Fspeed


    TBD:
        1. Check steering angles and yaw orientation using pj
        2. Ch
*/

void mppi_cb(const ackermann_msgs::AckermannDriveStamped::ConstPtr commands ){
    //Check if user has permitted MPPI
    if (!MPPI_flag){
        return;
    }

}

/*

Subscibe to /camera/depth/image_rect_raw
and /camera/color/image_rect_raw to ∂]=\
get depth and rgb data. 

ODT
*/
void odt_cb(){
    //Check if user has permitted object detection and tracking
    if (!ODT_flag){
        return ;
    }

}

/*

Go directly to a waypoint or follow a bunch of waypoints
Subscribe to /mavros/mission/waypoints

Or 

subscribe to /path since it gets published after
    - Calculating gps origin.
    - Generating a path along all waypoints. Each waypoint will be a X,Y coordinate that is
      calculated from lat and long in NED frame.
*/
void traverse_waypoints(){
    /*
    Check if there is a live path
    */

}


int main(int argc, char **argv) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    

    ros::init(argc, argv, "node_A1_control");
    

    ros::NodeHandle nh;


    ros::Subscriber sub_h12_cb;
    ros::Subscriber sub_mppi_cb;
    
    Custom custom(HIGHLEVEL);
    // InitEnvironment();

    sub_h12_cb = nh.subscribe("/mavros/rc/in", 1, h12_cb);
    sub_mppi_cb = nh.subscribe("/low_level_controller/dawg/control", 1, mppi_cb);

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
