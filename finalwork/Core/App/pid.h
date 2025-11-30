#include "main.h"
#include "gpio.h"
#include "math.h"
#include "tim.h"
#include "stdio.h"
typedef struct{
    float KP;
    float KI;
    float KD;
    float iout;
    float dout;
    float fdb;
    float ref;
    float cur_error;
    float error[2];
    float output;
    float outputMax;
    float outputMin;
    float ioutMax; //»ý·ÖÏÞ·ù
}PID_t;

void PID_init(PID_t *pid);
void PID_Calc_p(PID_t *pid);
void PID_Calc(PID_t *pid);
void PID_Set(PID_t *pid, float kp, float ki, float kd);
void PIDMax_Set(PID_t *pid, float pMax);


