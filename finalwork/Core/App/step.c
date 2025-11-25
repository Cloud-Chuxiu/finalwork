#include "step.h"

//步进电机初始化
void step_init()
{
    HAL_GPIO_WritePin(GPIOB, ENA_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, DIR_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, PUL_Pin, 0);
}

//步进电机方向设定
void step_dir_set(uint8_t dir)
{
    HAL_GPIO_WritePin(GPIOB, ENA_Pin, dir);
}

void step_set(uint32_t round)
{
    for(uint32_t i = 0; i < round * 400 * 2; i++)
    {
        HAL_GPIO_TogglePin(GPIOB, PUL_Pin);
        delay_us(100);
    }
}

void delay_us(uint16_t us)
{
    uint16_t counter = 0;
    __HAL_TIM_SetAutoreload(&htim3, us);   //设定 ARR值
    __HAL_TIM_SetCounter(&htim3, counter);  //初始化CNT
    HAL_TIM_Base_Start(&htim3);             //启动计时器
    while(counter != us) 
    {
        counter = __HAL_TIM_GetCounter(&htim3); // 获取定时器当前计数
    }   
    HAL_TIM_Base_Stop(&htim3); //停止定时器
}