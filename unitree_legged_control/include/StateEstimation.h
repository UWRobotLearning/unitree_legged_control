#ifndef _POSEPARSE_H_
#define _POSEPARSE_H_
#include <unitree_legged_msgs/LowState.h>
#include <unordered_map>
using namespace std;

typedef struct xyz{
    float x;
    float y;
    float z;
}XYZ;

typedef struct rpy{
    float roll;
    float pitch;
    float yaw;
}RPY;

typedef struct pose{
    XYZ* t;
    RPY* R;
}POSE;

POSE Q = {0};

POSE FR0 = {0};
POSE FL0 = {0};
POSE RR0 = {0};
POSE RL0 = {0};

POSE FR1 = {0};
POSE FL1 = {0};
POSE RR1 = {0};
POSE RL1 = {0};

POSE FR2 = {0};
POSE FL2 = {0};
POSE RR2 = {0};
POSE RL2 = {0};

POSE FR3 = {0};
POSE FL3 = {0};
POSE RR3 = {0};
POSE RL3 = {0};

unordered_map<string, POSE>PoseVector= 

{ 
{"Q", Q},

{"FR0", FR0},
{"FL0", FL0},
{"RR0", RR0},
{"RL0", RL0},

{"FR1", FR1},
{"FL1", FL1},
{"RR1", RR1},
{"RL1", RL1},

{"FR2", FR2},
{"FL2", FL2},
{"RR2", RR2},
{"RL2", RL2},

{"FR3", FR3},
{"FL3", FL3},
{"RR3", RR3},
{"RL3", RL3},

}

void LowStateCallBack(const unitree_legged_msgs :: LowState :: ConstPtr& msg>)
void InitPose(POSE* P, float mean, float std);

#endif