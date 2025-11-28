#include "pid.h"

//PID初始化
void PID_init(PID_t *pid)
{
    pid->KP = 0;
    pid->KI = 0;
    pid->KD = 0;
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

//PID参数设定
void PID_Set(PID_t *pid, float kp, float ki, float kd)
{
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
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
// void PID_Calc(PID_t *pid)
// {
//     pid->cur_error = pid->ref - pid->fdb;
//     pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
//     pid->error[0] = pid->error[1];
//     pid->error[1] = pid->ref - pid->fdb;
//     if(pid->output > pid->outputMax)
//     {
//         pid->output = pid->outputMax;
//     }
//     if(pid->output < -pid->outputMax)
//     {
//         pid->output = -pid->outputMax;
//     }
// }


//增量式PID
void PID_Calc(PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    
    // 计算增量
    float increment = pid->KP * (pid->cur_error - pid->error[1]) 
                    + pid->KI * pid->cur_error 
                    + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    
    // 抗饱和处理：如果已经饱和，且增量方向相同，则抑制积分
    if(pid->output >= pid->outputMax && increment > 0) {
        // 只保留PD部分，去掉积分项
        increment = pid->KP * (pid->cur_error - pid->error[1]) 
                   + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    }
    else if(pid->output <= -pid->outputMax && increment < 0) {
        // 只保留PD部分，去掉积分项
        increment = pid->KP * (pid->cur_error - pid->error[1]) 
                   + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    }
    
    pid->output += increment;
    
    // 更新误差队列
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->cur_error;  // 这里可以直接用pid->cur_error
    
    // 输出限幅
    if(pid->output > pid->outputMax) {
        pid->output = pid->outputMax;
    }
    if(pid->output < -pid->outputMax) {
        pid->output = -pid->outputMax;
    }
}