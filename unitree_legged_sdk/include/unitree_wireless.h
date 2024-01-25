#include "unitree_legged_sdk/unitree_joystick.h"
#include "unitree_legged_sdk/udp.h"
#include "unitree_legged_sdk/comm.h"
#include "unitree_legged_sdk/loop.h"

using namespace UNITREE_LEGGED_SDK;

class WIRE_LESS_CONTROL{

    private:

        static WIRE_LESS_CONTROL* wirelesscontrol = nullptr;
            WIRE_LESS_CONTROL* UdpSingleton(uint8_t level);

    public:

    WIRE_LESS_CONTROL(uint8_t level) : safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
    }

    ~WIRE_LESS_CONTROL();

    Safety safe;
    UDP udp;
    
    LowCmd cmd = {0};
    LowState state = {0};
    xRockerBtnDataStruct _keyData;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01

    void UDPSend();
    void UDPRecv();
    void UDPCont();
    void UDPLoop();
    void UdpDeleteInstance();
    WIRE_LESS_CONTROL* GetUdpInstance(uint8_t level);
    
};
