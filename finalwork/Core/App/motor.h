#include "pid.h"

#define CLOCKWISE 1
#define ANTI_CLOCKWISE 0
#define RELOADVALUE 65535
#define MOTOR_SPEED_RE 30
#define PULSE_PRE_ROUND 13
#define DEATH_ZONE 3


typedef struct {
    uint8_t motor_dir; //电机方向
    uint8_t pid_flag;  
    uint8_t speed_flag;
    uint8_t is_enable;
    uint16_t dutyfactor;
    int overflowNum;
    long total_count;
    long last_count;
    float actual_speed;
    float actual_angle;
    uint16_t TIM_PWM_CH;
    uint16_t TIM_Encoder_CH1;
    uint16_t TIM_Encoder_CH2;
    TIM_HandleTypeDef TIM_EncoderHandle;
    TIM_HandleTypeDef TIM_PWMHandle;
    PID_t motor_pospid;
    PID_t motor_speedpid;
}Motor_t;   

void Motor_Init(Motor_t *motor);
void Motor_Enable(Motor_t *motor);
void positionServo(float ref, Motor_t *motor);
float get_angle(Motor_t *motor, uint8_t dir);
void Motor_control(Motor_t *motor);
void Motor_Disable(Motor_t *motor);

void dir_set(Motor_t *motor);
float get_speed(Motor_t *motor);
void angle_ctrl(Motor_t *motor, float angle); //角度控制
void speed_ctrl(Motor_t *motor,float speed); //速度控制


