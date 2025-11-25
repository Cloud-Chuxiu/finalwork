#include "gpio.h"
#include "tim.h"

void step_init();
void step_dir_set(uint8_t dir);
void step_set(float round);
void delay_us(uint16_t us);
