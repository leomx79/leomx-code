#include "pid.h"



// ��ʼ��PID������
void PID_Init(PID *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
}

// ����PID���
double PID_Compute(PID *pid, double setpoint, double measured_value) {
    double error = setpoint - measured_value;
    pid->integral += error;
    double derivative = error - pid->prev_error;
    pid->prev_error = error;
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}


// ��ʼ������PID������
void CascadePID_Init(CascadePID *cascade_pid, double primary_kp, double primary_ki, double primary_kd, double secondary_kp, double secondary_ki, double secondary_kd) {
    PID_Init(&cascade_pid->primary, primary_kp, primary_ki, primary_kd);
    PID_Init(&cascade_pid->secondary, secondary_kp, secondary_ki, secondary_kd);
}

// ���㴮��PID���
double CascadePID_Compute(CascadePID *cascade_pid, double primary_setpoint, double secondary_setpoint, double measured_value) {
    double primary_output = PID_Compute(&cascade_pid->primary, primary_setpoint, measured_value);
    double secondary_output = PID_Compute(&cascade_pid->secondary, secondary_setpoint, primary_output);
    return secondary_output;
}

//int main() {
//    CascadePID cascade_pid;
//    CascadePID_Init(&cascade_pid, 1.0, 0.1, 0.01, 1.5, 0.2, 0.02);

//    double primary_setpoint = 100.0;
//    double secondary_setpoint = 50.0;
//    double measured_value = 90.0;

//    double output = CascadePID_Compute(&cascade_pid, primary_setpoint, secondary_setpoint, measured_value);
//    printf("�����������%f\n", output);

//    return 0;
//}
