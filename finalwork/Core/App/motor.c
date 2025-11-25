#include "motor.h"

void Motor_Init(Motor_t *motor)
{
       motor->motor_dir = CLOCKWISE;
       motor->pid_flag = 0;
       motor->speed_flag = 0;
       motor->dutyfactor = 0;
       motor->is_enable = 0;
       motor->actual_speed = 0;
       motor->actual_angle = 0;
       motor->overflowNum = 0;
       motor->total_count = 0;
       motor->last_count = 0;
       PID_init(&motor->motor_pospid);
       PID_init(&motor->motor_speedpid);
       motor->TIM_PWMHandle = htim1;
       motor->TIM_EncoderHandle = htim2;
       motor->TIM_PWM_CH = TIM_CHANNEL_1;
       motor->TIM_Encoder_CH1 = TIM_CHANNEL_1;
       motor->TIM_Encoder_CH2 = TIM_CHANNEL_2;
}

//使能电机
void Motor_Enable(Motor_t *motor)
{
    motor->is_enable = 1;
}

//失能电机
void Motor_Disable(Motor_t *motor)
{
    motor->is_enable = 0;
}
//控制电机启动
void Motor_control(Motor_t *motor)
{
    if(motor->is_enable == 1)
    {
        HAL_GPIO_WritePin(GPIOA, STBY_Pin, 1);
    }
    else if(motor->is_enable == 0)
    {
        HAL_GPIO_WritePin(GPIOA, STBY_Pin, 0);
    }
}

//设定电机方向
void dir_set(Motor_t *motor)
{
    if(motor->motor_dir == CLOCKWISE)
    {
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, 0);
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, 1);
    }
    else if(motor->motor_dir == ANTI_CLOCKWISE)
    {
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, 1);
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, 0);
    }
}


//位置伺服函数，双环PID驱动
void positionServo(float ref, Motor_t *motor)
{
    motor->motor_pospid.ref = ref;
    motor->motor_pospid.fdb = motor->actual_angle;
    PID_Calc(&motor->motor_pospid);

    motor->motor_speedpid.ref = motor->motor_pospid.output; //位置pid的输出结果作为速度pid期望
    motor->motor_speedpid.fdb = motor->motor_speedpid.output; //速度pid的反馈值取上次输出（仅控制角度无需精确）
    PID_Calc(&motor->motor_speedpid);
   
    //打印参数
    printf("%.3f,%.3f,%.3f\r\n",ref * 360,motor->motor_pospid.fdb * 180,motor->motor_pospid.output);
   
    if(motor->motor_speedpid.output > 0)
    {
        // if(motor->motor_dir != CLOCKWISE)
        // {
        //     motor->motor_dir = CLOCKWISE;
        //     dir_set(motor);
        // }
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 ,motor->motor_speedpid.output);
    }
    //如果转过了则反向转动
    // if(motor->motor_speedpid.output < 0)
    // {
    //     if(motor->motor_dir != ANTI_CLOCKWISE)
    //     {
    //         motor->motor_dir = ANTI_CLOCKWISE;
    //         dir_set(motor);
    //     }
    //     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 ,-motor->motor_speedpid.output);
    // }
}

//速度伺服函数
void speedServo(float ref, Motor_t *motor)
{
    motor->motor_speedpid.ref = ref; 
    motor->motor_speedpid.fdb = motor->actual_speed;
    PID_Calc(&motor->motor_speedpid);
    //打印参数
    //printf("%.2f,%.2f\r\n",motor->motor_speedpid.fdb,motor->motor_speedpid.output);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor->motor_speedpid.output);
}

//获取电机实时角度
float get_angle(float pre_angle, uint8_t dir)
{
    float raw_angle = (float)__HAL_TIM_GET_COUNTER(&htim2);
    if(raw_angle > 60000)
    {
        raw_angle = 65535 - raw_angle;
    }
    raw_angle /= (30 * 13 * 2);
    // uint32_t d_angle = raw_angle - (uint32_t)pre_angle;
    // if(dir == CLOCKWISE)
    // {
    //     pre_angle += d_angle;
    // }
    // else if(dir == ANTI_CLOCKWISE)
    // {
    //     pre_angle -= d_angle;
    // }
    return raw_angle;
}

//获取电机实时速度
float get_speed(Motor_t *motor)
{
    float raw_angle = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2,0);
    float speed = 0;
    speed = raw_angle/(30 * 13 * 2) / 0.01;
    return speed;
}

void angle_ctrl(Motor_t *motor, float angle) //角度控制
{
    while(1)
    {
        if(motor->pid_flag)
        {
            motor->actual_angle = get_angle(motor->actual_angle,motor->motor_dir); //获取实时角度
            positionServo(angle ,motor);
            motor->pid_flag = 0;
            //终止PID
            if(motor->motor_pospid.output <= 0)
            {
                return;
            }
        }
    }
}

void speed_ctrl(Motor_t *motor,float speed) //速度控制
{
    while(1)
    {
        if(motor->pid_flag)
        {
            motor->actual_speed = get_speed(motor);
            speedServo(speed, motor);
            motor->pid_flag = 0;
            //终止pid
            if(motor->motor_speedpid.output == 0)
            {
                return;
            }
        }
    }
}