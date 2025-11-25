#include "pid.h"

void PID_init(PID_t *pid)
{
    pid->KP = 8.0;
    pid->KI = 0.9;
    pid->KD = 8.0;
    pid->fdb = 0;
    pid->ref = 0;
    pid->iout = 0;
    pid->cur_error = 0;
    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->output = 0;
    pid->outputMax = 800;
    pid->outputMin = 5;
 
}

//位置式pid
void PID_Calc_p(PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    pid->iout += pid->cur_error;
    pid->output = pid->KP * pid->cur_error + pid->KI * pid->iout + pid->KD * (pid->cur_error - pid->error[1]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->ref - pid->fdb;
    if(pid->output > pid->outputMax)
    {
        pid->output = pid->outputMax;
    }
    if(pid->output < -pid->outputMax)
    {
        pid->output = -pid->outputMax;
    }
    if(fabs(pid->output) < pid->outputMin)
    {
        pid->output = 0;
    }
}

//增量式pid
void PID_Calc(PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->ref - pid->fdb;
    if(pid->output > pid->outputMax)
    {
        pid->output = pid->outputMax;
    }
    if(pid->output < -pid->outputMax)
    {
        pid->output = -pid->outputMax;
    }
}