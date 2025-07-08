#ifndef __USART_H
#define __USART_H

#include "main.h"
#include "my_value.h"

void uart_SendByte(UART_HandleTypeDef *huart, uint8_t data);
void uart_SendCmd(UART_HandleTypeDef *huart, uint8_t *cmd, uint8_t len);
void Start_Detect(openmvTypeDef *openmv, uint8_t process_flags, uint8_t counter_val, uint8_t *counter_ptr);
void End_Detect(openmvTypeDef *openmv);
void init_openmv(openmvTypeDef *openmv, uint8_t deviceType);
void init_all_openmv(openmvTypeDef *A_openmv, openmvTypeDef *B_openmv);
void process_openmv(openmvTypeDef *A_openmv, openmvTypeDef *B_openmv);

#endif
