#ifndef  __PID_H
#define  __PID_H
#include "stm32f10x.h"

// PID结构体定义
typedef struct {
    double kp;  // 比例增益
    double ki;  // 积分增益
    double kd;  // 微分增益
    double prev_error;  // 上一次误差
    double integral;    // 积分值
} PID;
// 串级PID控制器
typedef struct {
    PID primary;  // 主PID控制器
    PID secondary;  // 次PID控制器
} CascadePID;

void PID_Init(PID *pid, double kp, double ki, double kd);
double PID_Compute(PID *pid, double setpoint, double measured_value);
void CascadePID_Init(CascadePID *cascade_pid, double primary_kp, double primary_ki, double primary_kd, double secondary_kp, double secondary_ki, double secondary_kd);
double CascadePID_Compute(CascadePID *cascade_pid, double primary_setpoint, double secondary_setpoint, double measured_value);
 
#endif
