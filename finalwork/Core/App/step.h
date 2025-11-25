#include "gpio.h"
#include "tim.h"

void step_init();
void step_dir_set(uint8_t dir);
void step_set(uint32_t round);
void delay_us(uint16_t us);
