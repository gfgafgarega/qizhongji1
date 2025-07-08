/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "uart.h"
#include "Emm_V5.h"
#include "my_value.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern carTypeDef Underframe;
extern carTypeDef Turntable;
extern carTypeDef Elevate;
extern openmvTypeDef A_openmv;
extern openmvTypeDef B_openmv;
extern const DownFunction down_functions[6];
extern GoToFunction goToFunctions[12];  // 鍒濆鍖栦负绌?

uint8_t counter=0;//杩欐槸鎺ユ敹澶氬皯缁刼penmv鏁版嵁鐨勫??
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


// 定义全局变量来存储Degree_Value的值
volatile int32_t first_value = 0;    // 存储第一次中断时的角度值
volatile int32_t degree_value = 0;   // 存储最新的角度值
volatile uint8_t first_interrupt = 1; // 初始为1表示第一次中断
volatile int32_t e = 0;//用于修正的值


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	
	
	init_all_openmv(&A_openmv, &B_openmv);
	motor_init();
	OLED_Init();
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);	
	
//	Delay_ms(1);
	
	HAL_UART_Receive_IT(&huart4,Underframe.data,1);
	HAL_UART_Receive_IT(&huart5,Turntable.data,1);
	HAL_UART_Receive_IT(&huart6,Elevate.data,1);
	
	/*-----------------------------璋冭瘯鍖?------------------------------------*/
//	Elevate_control(Elevate_debug_position_1, 0);
//	Turntable_control(Turntable_debug_position, 0);
//	Underframe_controXl(Underframe_debug_position, 0);

//	Rotary_servo_motor_control(36, 100, 100);
//	Gripper_servo_motor_control(24,100,400);

//	Turntable_control(-1,0);
//	Underframe_control(1,0);
//	Underframe_control(1,200);
//	Turntable_control(-1000,200);
//	Underframe_control(16600,0);

////	Underframe_control(-1,200);
//	Elevate_control(-15000,0);  //-6000 ,-15000

//	Turntable_control(-5650, 0);
//	Turntable_control(-30100,0);
//	Underframe_control(16600,0);
//	Underframe_control(21350,0);
//	
//	Elevate_control(-23700,1000);
//	Gripper_servo_motor_control(Gripper_open,400,400);
//	Elevate_control(-27000,0);

//  Go_to_B_d();
//	Go_to_B_a();
//	Go_to_B_b();
//	Go_to_B_e();
//	Go_to_B_c();
//	Go_to_B_f();

//	Go_to_A_a_Down();
//	Go_to_A_b_Down();
//	Go_to_A_c_Down();
//	Go_to_A_d_Down();
//	Go_to_A_e_Down();
//	Go_to_A_f_Down();
/*-----------------------------------------------------------------*/
	

/*-----------------------A鍖鸿瘑鍒?--------------------------------------*/
//		Go_to_A_abc_detect();
//		Start_Detect(&A_openmv,0x07,30,&counter);
//		OLED_ShowNum(2,1,A_openmv.a.num,1);
//		OLED_ShowNum(2,3,A_openmv.b.num,1);
//		OLED_ShowNum(2,5,A_openmv.c.num,1);

//		Go_to_A_def_detect();
//		Start_Detect(&A_openmv,0x38,30,&counter);

//		OLED_ShowNum(2,7,A_openmv.d.num,1);
//		OLED_ShowNum(2,9,A_openmv.e.num,1);
//		OLED_ShowNum(2,11,A_openmv.f.num,1);
///*-----------------------------------------------------------------*/
///*-----------------------B鍖鸿瘑鍒?--------------------------------------*/
//		Go_to_B_detect();
//		Start_Detect(&B_openmv,0xFF,20,&counter);
//		OLED_ShowNum(1,1,B_openmv.a.num,1);
//		OLED_ShowNum(1,3,B_openmv.b.num,1);
//		OLED_ShowNum(1,5,B_openmv.c.num,1);
//		OLED_ShowNum(1,7,B_openmv.d.num,1);
//		OLED_ShowNum(1,9,B_openmv.e.num,1);
//		OLED_ShowNum(1,11,B_openmv.f.num,1);			
/*-----------------------------------------------------------------*/

//		A_openmv.a.num=5;
//		A_openmv.b.num=4;
//		A_openmv.c.num=3;
//		A_openmv.d.num=0;
//		A_openmv.e.num=2;
//		A_openmv.f.num=1;

//		B_openmv.a.num=5;
//		B_openmv.b.num=2;
//		B_openmv.c.num=1;
//		B_openmv.d.num=6;
//		B_openmv.e.num=4;
//		B_openmv.f.num=3;
//		
//	Go_to_A_abc_detect();	
//	Go_to_A_def_detect();
//	Go_to_B_detect();	
//	Go_to_B_d();
//	Go_to_A_b_Down();
//	process_openmv(&A_openmv,&B_openmv);
//	execute_functions();


	// 鍏抽棴TIM7鐨勫畾鏃跺櫒涓柇
//	HAL_TIM_Base_Stop_IT(&htim7);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		Delay_ms(1);
//		
//		Start_Detect(&A_openmv,0x01,30,&counter);//绗竴涓舰鍙傛槸閫塷penmv,1鏄疉_openmv,2鏄疊_openmv.
//		OLED_ShowNum(3,1,s,3);																											 //绗簩涓舰鍙傛槸璁剧疆瑕佹娴嬬殑瀛楁瘝锛屾瘮濡?0B00000001灏辨槸a锛?0xFF灏辨槸鍏ㄩ儴
																													 //绗笁涓舰鍙傛槸璁¤澶勭悊鏁版嵁鐨勮疆鏁帮紝绗洓涓笉鐢ㄧ
//		OLED_ShowNum(2,13,Turntable.motor1.arrived_flag,2);																											 //杩欒鍑芥暟鎬讳綋鎰忔?濆氨鏄紑濮嬭幏鍙朅_openmv鍦╝鍖烘娴嬫椂鍙戞潵鐨?30涓暟鎹寘
//			OLED_ShowNum(3,1,A_openmv.a.num,3);
//			OLED_ShowNum(3,5,A_openmv.b.num,3);
//			OLED_ShowNum(3,9,A_openmv.c.num,3);
//			OLED_ShowNum(3,13,A_openmv.d.num,3);
//			OLED_ShowNum(4,1,A_openmv.e.num,3);
//			OLED_ShowNum(4,5,A_openmv.f.num,3);
//		
//		OLED_ShowHexNum(1, 1, Underframe.motor2.received_data[0], 2);
//    OLED_ShowHexNum(1, 4, Underframe.motor2.received_data[1], 2);
//    OLED_ShowHexNum(1, 7, Underframe.motor2.received_data[2], 2);
//    OLED_ShowHexNum(1, 10, Underframe.motor2.received_data[3], 2);
//    OLED_ShowHexNum(1, 13, Underframe.motor2.received_data[4], 2);
//    OLED_ShowHexNum(2, 1, Underframe.motor2.received_data[5], 2);
//    OLED_ShowHexNum(2, 4, Underframe.motor2.received_data[6], 2);
//    OLED_ShowHexNum(2, 7, Underframe.motor2.received_data[7], 2);

//		OLED_ShowHexNum(1, 1, Underframe.motor2.position_data[0], 2);
//		OLED_ShowHexNum(1, 4, Underframe.motor2.position_data[1], 2);
//		OLED_ShowHexNum(1, 7, Underframe.motor2.position_data[2], 2);
//		OLED_ShowHexNum(1, 10, Underframe.motor2.position_data[3], 2);
//		OLED_ShowHexNum(1, 13, Underframe.motor2.position_data[4], 2);
//		OLED_ShowHexNum(2, 1, Underframe.motor2.position_data[5], 2);
//		OLED_ShowHexNum(2, 4, Underframe.motor2.position_data[6], 2);
//		OLED_ShowHexNum(2, 7, Underframe.motor2.position_data[7], 2);
			
//		OLED_ShowHexNum(1, 1, Underframe.motor2.Pulse_Value_data[0], 2);
//		OLED_ShowHexNum(1, 4, Underframe.motor2.Pulse_Value_data[1], 2);
//		OLED_ShowHexNum(1, 7, Underframe.motor2.Pulse_Value_data[2], 2);
//		OLED_ShowHexNum(1, 10, Underframe.motor2.Pulse_Value_data[3], 2);
//		OLED_ShowHexNum(1, 13, Underframe.motor2.Pulse_Value_data[4], 2);
//		OLED_ShowHexNum(2, 1, Underframe.motor2.Pulse_Value_data[5], 2);
//		OLED_ShowHexNum(2, 4, Underframe.motor2.Pulse_Value_data[6], 2);
//		OLED_ShowHexNum(2, 7, Underframe.motor2.Pulse_Value_data[7], 2);
//			OLED_ShowSignedNum(1,7,Underframe.motor2.Degree_Value,8);
//		OLED_ShowSignedNum(3,7,Underframe.motor2.Pulse_Value-Underframe.motor2.Degree_Value,8);
//    OLED_ShowHexNum(3, 1, Underframe.motor1.received_data[0], 2);
//    OLED_ShowHexNum(3, 4, Underframe.motor1.received_data[1], 2);
//    OLED_ShowHexNum(3, 7, Underframe.motor1.received_data[2], 2);
//    OLED_ShowHexNum(3, 10, Underframe.motor1.received_data[3], 2);
//    OLED_ShowHexNum(3, 13, Underframe.motor1.received_data[4], 2);
//    OLED_ShowHexNum(4, 1, Underframe.motor1.received_data[5], 2);
//    OLED_ShowHexNum(4, 4, Underframe.motor1.received_data[6], 2);
//    OLED_ShowHexNum(4, 7, Underframe.motor1.received_data[7], 2);
//		OLED_ShowSignedNum(2,7,Underframe.motor1.Degree_Value,8);    
		// 鏄剧ず A_openmv.received_data
//			OLED_ShowHexNum(1,1,A_openmv.data[0],3);
//			OLED_ShowNum(1,5,A_openmv.data[1],3);
//			OLED_ShowNum(1,9,A_openmv.data[2],3);
//			OLED_ShowNum(1,13,A_openmv.data[3],3);
//			OLED_ShowNum(2,1,A_openmv.data[4],3);
//			OLED_ShowHexNum(2,5,A_openmv.data[5],3);
//			
//			OLED_ShowHexNum(3,9,A_openmv.received_data[6],3);
//			OLED_ShowHexNum(3,13,A_openmv.received_data[7],3);
//			OLED_ShowHexNum(4,1,A_openmv.received_data[8],3);
//			OLED_ShowHexNum(4,5,A_openmv.received_data[9],3);

//			// 鏄剧ず B_openmv.received_data
//			OLED_ShowHexNum(2,1,B_openmv.data[0],3);
//			OLED_ShowNum(2,5,B_openmv.data[1],3);
//			OLED_ShowNum(2,9,B_openmv.data[2],3);
//			OLED_ShowNum(2,13,B_openmv.data[3],3);
//			OLED_ShowNum(3,1,B_openmv.data[4],3);
//			OLED_ShowHexNum(3,5,B_openmv.data[5],3);
			// 鎵撳嵃鎵?鏈夋爣蹇椾綅
    

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//中断函数
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
	
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
		
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_9))
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
	
        if (first_interrupt) 
				{
            first_value = Underframe.motor1.Degree_Value;  // 第一次中断时保存到first_value
            first_interrupt = 0;                
        } 
				else 
				{
            degree_value = Underframe.motor1.Degree_Value; // 后续中断更新degree_value
					  e= degree_value-first_value; //更新e的值
        }
			
    
	}
  /* USER CODE END EXTI9_5_IRQn 1 */

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
