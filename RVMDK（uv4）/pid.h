#ifndef  __PID_H
#define  __PID_H
#include "stm32f10x.h"

// PID�ṹ�嶨��
typedef struct {
    double kp;  // ��������
    double ki;  // ��������
    double kd;  // ΢������
    double prev_error;  // ��һ�����
    double integral;    // ����ֵ
} PID;
// ����PID������
typedef struct {
    PID primary;  // ��PID������
    PID secondary;  // ��PID������
} CascadePID;

void PID_Init(PID *pid, double kp, double ki, double kd);
double PID_Compute(PID *pid, double setpoint, double measured_value);
void CascadePID_Init(CascadePID *cascade_pid, double primary_kp, double primary_ki, double primary_kd, double secondary_kp, double secondary_ki, double secondary_kd);
double CascadePID_Compute(CascadePID *cascade_pid, double primary_setpoint, double secondary_setpoint, double measured_value);
 
#endif
