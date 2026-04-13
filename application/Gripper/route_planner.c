#include "main.h"
#include "route_planner.h"


 void route_planner_init(Cubic_Polynomial *polynomial, float num_joints, float T)
{
  // 初始化路径规划器
}

void route_planner_calculate_coefficients(Cubic_Polynomial *polynomial)
{
  // 计算三次多项式系数
    polynomial->a0 = polynomial->q0;
    polynomial->a1 = polynomial->v0;
    polynomial->a2 = 3.0f * (polynomial->qf - polynomial->q0) / (polynomial->tf - polynomial->t0) - 2.0f * polynomial->v0 - polynomial->vf;
    polynomial->a3 = 2.0f * (polynomial->q0 - polynomial->qf) / (polynomial->tf - polynomial->t0) + polynomial->v0 + polynomial->vf;

}
