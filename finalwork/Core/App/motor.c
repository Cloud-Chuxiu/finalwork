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
       //设置两环pid的参数
       PID_Set(&motor->motor_pospid, 700, 5, 200);  // KP  KI  KD
       PID_Set(&motor->motor_speedpid, 7, 0.1, 1);  // KP  KI  KD
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
    HAL_GPIO_WritePin(GPIOA, STBY_Pin, 1);
}

//失能电机
void Motor_Disable(Motor_t *motor)
{
    motor->is_enable = 0;
    HAL_GPIO_WritePin(GPIOA, STBY_Pin, 0);

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
    PID_Calc_p(&motor->motor_pospid);
    //PID_Calc_p(&motor->motor_pospid);

    motor->motor_speedpid.ref = motor->motor_pospid.output; //位置pid的输出结果作为速度pid期望
    motor->motor_speedpid.fdb = motor->actual_speed; //速度pid的反馈值取上次输出（仅控制角度无需精确）
    PID_Calc(&motor->motor_speedpid);
    
   
    //打印参数
    printf("%.3f,%.3f,%.3f\r\n",ref* 360,motor->motor_pospid.fdb * 360,motor->motor_pospid.output);
   
    //死区控制


    //顺时针为正，逆时针为负
    if(motor->motor_speedpid.output > 0)
    {
        motor->motor_dir = CLOCKWISE;
        dir_set(motor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 ,motor->motor_speedpid.output);
    }
    else if(motor->motor_speedpid.output < 0)
    {
        motor->motor_dir = ANTI_CLOCKWISE;
        dir_set(motor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 ,-motor->motor_speedpid.output);
    }
    
}

//速度伺服函数
void speedServo(float ref, Motor_t *motor)
{
    motor->motor_speedpid.ref = ref; 
    motor->motor_speedpid.fdb = motor->actual_speed;
    PID_Calc(&motor->motor_speedpid);
    //打印参数
    printf("%.3f,%.3f,%.3f\r\n",motor->motor_speedpid.fdb,motor->motor_speedpid.output,ref);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor->motor_speedpid.output);
}

//获取电机实时角度 & 速度
float get_angle(Motor_t *motor, uint8_t dir)
{
    //获取原始计数
    float raw_angle = (short)__HAL_TIM_GET_COUNTER(&htim2); //(short) ?
    __HAL_TIM_SET_COUNTER(&htim2,0);
    //防止溢出计算
    if(raw_angle > 32768)
    {
        raw_angle = 65535 - raw_angle;
    }
    //将计数改为角度
    raw_angle = raw_angle  /  (30 * 13 *4) ;
    motor->actual_angle += raw_angle;
    motor->actual_speed =  60 * raw_angle / 0.01;
    return raw_angle;
}


//获取电机实时速度
float get_speed(Motor_t *motor)
{
    float raw_angle = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2,0);
    motor->actual_speed = 60 * raw_angle / (30 * 13 *4) / 0.01;
    return 0;
}

//角度控制
void angle_ctrl(Motor_t *motor, float angle) 
{
    while(1)
    {
        if(motor->pid_flag)
        {
            get_angle(motor,motor->motor_dir); //获取实时角度
            positionServo(angle ,motor);
            motor->pid_flag = 0;
            //终止PID
            if(motor->motor_pospid.output == 0)
            {
                return;
            }
        }
    }
}

//速度控制
void speed_ctrl(Motor_t *motor,float speed) 
{
    while(1)
    {
        if(motor->pid_flag)
        {
            get_speed(motor);
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