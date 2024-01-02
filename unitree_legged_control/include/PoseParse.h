#ifndef _POSEPARSE_H_
#define _POSEPARSE_H_

typedef struct xyz{
    float points[3];
}XYZ;

typedef struct rpy{
    float points[3];
}RPY;

typedef struct pose{
    XYZ* t;
    RPY* R;
}POSE;


class Parser{

    public:

    static void Transform(POSE* p, RPY* R, XYZ* t);

};

#endif