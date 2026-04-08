#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H


typedef struct
{
    float q0;
    float qf;

    float v0;
    float vf;
    
    float t0;
    float tf;

    float a0;
    float a1;
    float a2;
    float a3;

    float num_joints;  //关节数量
    float T;           //总时间
} Cubic_Polynomial;


#endif
