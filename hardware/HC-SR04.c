//#include "main.h"
#include "tim.h"
#include "gpio.h"
//#include "OLED.h"

uint32_t num_1;
uint32_t num_2;
uint32_t Distance_1;
uint32_t Distance_2;
uint32_t Average_Distance_1=0;
uint32_t Average_Distance_Temp_1=0;
uint32_t Average_Distance_2=0;
uint32_t Average_Distance_Temp_2=0;
uint8_t Average_count = 0;
uint8_t trigger_flag = 0;

//void Read_Pulse_Value(uint8_t addr);
//void Delay_us(uint32_t xus)
//{
//    // 获取当前的计时值
//    uint32_t tickstart = HAL_GetTick(); // 获取当前的 systick 值

//    // 计算应结束的 tick 值
//    uint32_t delay = xus / 1000; // 把微秒转为毫秒
//    if (delay == 0) {
//        delay = 1; // 最小延迟为1毫秒
//    }

//    // 进行延迟
//    while ((HAL_GetTick() - tickstart) < delay) {
//        // 可以在这里做其他事情，比如检查某个条件
//    }
//}

//void Delay_ms(uint32_t Delay)
//{
//    // 获取当前的计时值
//   uint32_t tickstart = HAL_GetTick();
//	uint32_t wait = Delay;

//  /* Add a freq to guarantee minimum wait */
//  if (wait < HAL_MAX_DELAY)
//  {
//    wait += (uint32_t)(uwTickFreq);
//  }

//  while((HAL_GetTick() - tickstart) < wait)
//  {
//  }
//}


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{	
//	if (GPIO_Pin == echo1_Pin) 
//	{
//		if (HAL_GPIO_ReadPin(GPIOB, echo1_Pin) == GPIO_PIN_SET)       //Ɛ׏ʏʽҘ
//		{
//			num_1=0;
//			HAL_TIM_Base_Start_IT(&htim6);                                  //Ǵ֯֨ʱǷ݆ʱ
//		}
//		else if(HAL_GPIO_ReadPin(GPIOB, echo1_Pin) == GPIO_PIN_RESET)  //Ɛ׏ЂݵҘ
//		{
//			HAL_TIM_Base_Stop_IT(&htim6);                               //ژҕ֨ʱǷ݆ʱ
//			Distance_1=(num_1*1000+__HAL_TIM_GET_COUNTER(&htim6))*0.34/2;        //݆̣ߠkեλmm  340m/s=0.34mm/us   ޸ɫא׏քՎ˽ݓʏ݆˽Ƿʏӑޭ݆քՎ˽=؜ʱݤ
//		}
//  }
//	if (GPIO_Pin == echo2_Pin) 
//	{
//		if (HAL_GPIO_ReadPin(GPIOD, echo2_Pin) == GPIO_PIN_SET)       //Ɛ׏ʏʽҘ
//		{
//			num_2=0;
//			HAL_TIM_Base_Start_IT(&htim10);                                  //Ǵ֯֨ʱǷ݆ʱ
//		}
//		else if(HAL_GPIO_ReadPin(GPIOD, echo2_Pin) == GPIO_PIN_RESET)  //Ɛ׏ЂݵҘ
//		{
//			HAL_TIM_Base_Stop_IT(&htim10);                               //ژҕ֨ʱǷ݆ʱ
//			Distance_2=(num_2*1000+__HAL_TIM_GET_COUNTER(&htim10))*0.34/2;        //݆̣ߠkեλmm  340m/s=0.34mm/us   ޸ɫא׏քՎ˽ݓʏ݆˽Ƿʏӑޭ݆քՎ˽=؜ʱݤ
//		}
//  }
//}

//void HC_SR04_START(void)
//{
//	HAL_TIM_Base_Start_IT(&htim7);
//}





//void HC_SR04_Compute_IT(void)
//{
//	if(trigger_flag==1)
//	{
//			HAL_GPIO_WritePin(GPIOB, trigger1_Pin,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOD, trigger2_Pin,GPIO_PIN_SET);
//			Delay_us(10);
//			HAL_GPIO_WritePin(GPIOB, trigger1_Pin,GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOD, trigger2_Pin,GPIO_PIN_RESET);
//			Average_count++;
//			Average_Distance_Temp_1=Average_Distance_Temp_1+Distance_1;
//			Average_Distance_Temp_2=Average_Distance_Temp_2+Distance_2;
//			
//			if (Average_count == 8)
//				{
//						Average_Distance_Temp_1=Average_Distance_Temp_1/8;
//						Average_Distance_1=Average_Distance_Temp_1;
//						Average_Distance_Temp_2=Average_Distance_Temp_2/8;
//						Average_Distance_2=Average_Distance_Temp_2;
//						Average_count=0;
//						
//				}
//		
//			trigger_flag=0;
//	}
//}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM6) 
//		{
//			num_1++;
//		}
//		if (htim->Instance == TIM10) 
//		{
//			num_2++;
//		}
//    if (htim->Instance == TIM7) 
//		{
//				trigger_flag=1;
////				HC_SR04_Compute_IT();
//				Read_Pulse_Value(2);
//			Delay_ms(1);
//				Read_Pulse_Value(1);
//		}   
//}
