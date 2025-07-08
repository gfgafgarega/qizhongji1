#ifndef __HC_SR04_H
#define __HC_SR04_H
#include "main.h"
void HC_SR04_Compute(void);
extern uint32_t Average_Distance_1;
extern uint32_t Distance_1;
extern uint32_t Average_Distance_2;
extern uint32_t Distance_2;
//void Delay_us(uint32_t xus);
//void Delay_ms(uint32_t Delay);
void HC_SR04_START(void);
void HC_SR04_Compute_IT(void);
#endif
