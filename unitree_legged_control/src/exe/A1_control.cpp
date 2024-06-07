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
bool steer_flag = false;
bool reset_flag = false;


ros::Publisher pub_high, pub_low;
unitree_legged_msgs::HighState high_state_ros;
unitree_legged_msgs::LowState low_state_ros;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), low_udp(LOWLEVEL), high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
        high_udp.InitCmdData(high_cmd);
        cout << " high level" << endl;
    };

    Custom()
        : 
        safe(LeggedType::A1),
        low_udp(LOWLEVEL),
        high_udp(8090 , "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
        cout << "low mode" << endl;
    }

    void HighUDPRecv();
    void HighUDPSend();
    void HighRobotControl();

    void LowUDPRecv();
    void LowUDPSend();
    void LowRobotControl();

    Safety safe;
    UDP high_udp;
    UDP low_udp;
    HighCmd high_cmd = {0};
    HighState high_state = {0};
    LowCmd low_cmd = {0};
    LowState low_state = {0};

    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01

    ~Custom(){
        cout << "deleted level";
    };
};



void Custom::HighUDPRecv()
{
    high_udp.Recv();
}

void Custom::HighUDPSend()
{  
    high_udp.Send();
}

void Custom::HighRobotControl() 
{
    high_udp.GetRecv(high_state);
    high_state_ros = state2rosMsg(high_state);
    pub_high.publish(high_state_ros);

    high_cmd.mode            = A1mode;
    high_cmd.gaitType        = gaitType;
    high_cmd.speedLevel      = speedLevel;
    high_cmd.footRaiseHeight = footraiseheight;
    high_cmd.bodyHeight      = bodyheight;
    high_cmd.velocity[0]     = Fspeed;
    high_cmd.velocity[1]     = Sspeed;
    high_cmd.yawSpeed        = Yspeed;
    high_cmd.euler[0]        = r;
    high_cmd.euler[1]        = p;
    high_cmd.euler[2]        = y;
    high_udp.SetSend(high_cmd);

}

void Custom::LowUDPRecv()
{
    low_udp.Recv();
}

void Custom::LowUDPSend()
{  
    low_udp.Send();
}

void Custom::LowRobotControl() 
{
    low_udp.GetRecv(low_state);
    low_state_ros = state2rosMsg(low_state);
    pub_high.publish(low_state_ros);
    /*TODO send low level commands*/

    low_udp.SetSend(low_cmd);

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

    /* Init rpy in place of eulers[3] (which is used in 3.3.1)*/
    r = 0.f;
    p = 0.f;
    y = 0.f;

    /*Init forward and side speed in place of veloci
    LowCmd low_cmd = {0};
    LowState low_state = {0};ty[2](which is used in 3.2)*/

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

    //recovery
    if (rc->channels[8] > 1500)
    {
        if (!reset_flag){
            A1mode = 8;
            cout<<"recovery stand!"<<endl;
            reset_flag = true;
        }
    }
    else
    {
        if (reset_flag){
            A1mode = 7;
            cout<<"damping mode!"<<endl;
            reset_flag = false;
        }
    }

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
*/

void mppi_cb(const ackermann_msgs::AckermannDriveStamped::ConstPtr commands ){
    //Check if user has permitted MPPI
    if (!MPPI_flag){
        return;
    }
    float auto_steering   = commands->drive.steering_angle;
    float auto_wheelspeed = commands->drive.speed;

    Fspeed =  (auto_wheelspeed - 8.5)/8.5;

    /*
    Ideally we are targetting the steering angle,
    if we target steering angle, might as well as use a PID.
    for now only using a P with kp = 10 since 10 * steeringmax = 10*0.47 = 4.7 that
    can be assumed as 4.7rad/s
    */
    Yspeed =  auto_steering * 10;
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


    // InitEnvironment();

    sub_h12_cb = nh.subscribe("/mavros/rc/in", 1, h12_cb);
    sub_mppi_cb = nh.subscribe("/low_level_controller/dawg/control", 1, mppi_cb);
    pub_high       = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    pub_low       = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);

    if (strcasecmp(argv[1], "HIGHLEVEL") == 0){
        cout<<"highlevel"<<endl;        

        Custom custom(HIGHLEVEL);
        LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::HighRobotControl, &custom));
        LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::HighUDPSend,      &custom));
        LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::HighUDPRecv,      &custom));

        loop_udpSend.start();
        loop_udpRecv.start();
        loop_control.start(); 
        ros::spin();
    }

    else if (strcasecmp(argv[1], "LOWLEVEL") == 0){
        cout<<"lowlevel"<<endl;        
        Custom custom;
        // send low cmd for init
        custom.low_udp.SetSend(custom.low_cmd);
        // LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::LowRobotControl, &custom));
        LoopFunc loop_udpSend("udp_send",     custom.dt*10, 3, boost::bind(&Custom::LowUDPSend,      &custom));
        LoopFunc loop_udpRecv("udp_recv",     custom.dt*10, 3, boost::bind(&Custom::LowUDPRecv,      &custom));

        loop_udpSend.start();
        loop_udpRecv.start();
        // loop_control.start();
        ros::spin();
    }

    return 0; 
}
