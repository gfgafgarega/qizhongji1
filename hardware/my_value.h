#ifndef __MY_VALUE_H
#define __MY_VALUE_H

#include "main.h"
#include "tim.h"
#define relative_movement 0
#define absolute_movement 1
#define cw 0
#define ccw 1
#define multi_machine_synchonization_false 0
#define multi_machine_synchonization_true 1

#define Position_mode 0xFD //位置模式
#define Read_pulse_count 0x32 //读脉冲数
#define Read_Degree_count 0x36 //读角度
#define Correct_return 0x02 //正确返回
#define In_place 0x9F //电机到位
// 定义最大等待时间（单位：ms）
#define MAX_WAIT_TIME 3000
// 定义错误码
#define SUCCESS 0
#define TIMEOUT_ERROR 1
#define Turntable_vel 2000
#define Turntable_acc 5
//#define Underframe_vel 8000
//#define Underframe_acc 50  //最大加速度在150左右
#define Elevate_vel 24000
#define Elevate_acc 245

/*-----------------------------------------------------------------*/

#define Turntable_debug_position 1000 //往大的是逆时针
#define Underframe_debug_position 16800//往大的是往货架这边走-18000 1
#define Elevate_debug_position_1 -27400//往大是降低
#define Elevate_debug_position_2 25000 
#define Rotary_debug_angle 23 //往大是顺时针,最大40几

#define Gripper_close 18
#define Gripper_open 24 //往大是张开，最大到26
#define Gripper_close_all 14

/*-----------------------------------------------------------------*/
#define Turntable_position_A_detect_a -2500 //
#define Turntable_position_A_detect_b -5000 
#define Turntable_position_A_detect_c -9500
#define Turntable_position_A_detect_d -13000
#define Turntable_position_A_detect_e -17500
#define Turntable_position_A_detect_f -20000

#define Underframe_position_A_detect_a_f -7500
#define Underframe_position_A_detect_b_e -11200
#define Underframe_position_A_detect_c_d -7900

#define add_position 0 //底盘修正参数

#define Elevate_position_A_detect -25000 //往小是升高

#define Rotary_angle_A_detect_a 8
#define Rotary_angle_A_detect_b 16
#define Rotary_angle_A_detect_c 12
#define Rotary_angle_A_detect_d 9
#define Rotary_angle_A_detect_e 4
#define Rotary_angle_A_detect_f 12
/*-----------------------------------------------------------------*/
#define Turntable_position_B_detect_a_d 0 
#define Turntable_position_B_detect_b_e 0 
#define Turntable_position_B_detect_c_f 0 

#define Underframe_position_B_detect_a_d_c_f 0
#define Underframe_position_B_detect_b_e 0

#define Elevate_position_B_a_b_c_up 0 //往小是升高
#define Elevate_position_B_a_b_c_down 0
#define Elevate_position_B_d_e_f_up 0 
#define Elevate_position_B_d_e_f_down 0 

#define Rotary_angle_B_a_d 0
#define Rotary_angle_B_b_e 0
#define Rotary_angle_B_c_f 0
#define Rotary_angle_B_detect 22


/*-----------------------------------------------------------------*/
#define Turntable_position_A_a 1 //往小的是顺时针
#define Turntable_position_A_b -5000
#define Turntable_position_A_c -9500
#define Turntable_position_A_d -13500
#define Turntable_position_A_e -17500
#define Turntable_position_A_f -22000

#define Underframe_position_A_a_f 9000 //往小的是往货架这边走-18000 1
#define Underframe_position_A_b_e 9600
#define Underframe_position_A_c_d 7000

#define Elevate_position_Up 25000 //最高就是25000左右了
#define Elevate_position_Down 1000 //往大是升高
#define Elevate_position_a4 7000

#define Rotary_angle_A_a 8 //往大是顺时针
#define Rotary_angle_A_b 17
#define Rotary_angle_A_c 11
#define Rotary_angle_A_d 9
#define Rotary_angle_A_e 5
#define Rotary_angle_A_f 11
/*-----------------------------------------------------------------*/
// 定义状态的宏
#define END 0
#define START 1
// 定义不同 OpenMV 设备的宏
#define A_OPENMV 1
#define B_OPENMV 2

// 模拟 Go_to_A_a 等函数
#define Up 1
#define Down 2
typedef struct 
{
    uint8_t received_data[36];
    volatile uint8_t arrived_flag;
    int32_t Pulse_Value;
		uint8_t position_data[36];
		uint8_t Pulse_Value_data[36];
    volatile uint8_t motor_received_flag;  // 重命名以解决命名冲突
		volatile int32_t Degree_Value;
	  volatile int32_t Underframe_vel;//底盘速度
	  volatile int32_t Underframe_acc;//底盘加速度
} MotorTypeDef;


typedef struct 
{
    uint8_t data[64];           // 原始数据缓冲区
    volatile uint8_t uart_received_flag; // 重命名以解决命名冲突
    MotorTypeDef motor1;
    MotorTypeDef motor2;
    MotorTypeDef motor3;
    MotorTypeDef motor4;

	// 嵌入MotorTypeDef结构体
} carTypeDef;


typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
	volatile uint8_t num;
	float num_score[7]; // 修改为浮点数数组，长度为 7
	uint8_t up;
	uint8_t down;
	
}SignalTypeDef;

// 定义 openmvTypeDef 结构体，添加处理标志位
typedef struct
{
    uint8_t data[36];
    SignalTypeDef a;
    SignalTypeDef b;
    SignalTypeDef c;
    SignalTypeDef d;
    SignalTypeDef e;
    SignalTypeDef f;
    uint8_t deviceType;
    volatile uint8_t process_flags; // 处理标志位，每一位代表一个 SignalTypeDef 实例
		volatile uint8_t Ending_flags;
} openmvTypeDef;

// 定义函数指针类型
typedef void (*DownFunction)(void);
// 统一函数指针类型（Up/Down函数均为无参void函数）
typedef void (*GoToFunction)(void);
void motor_init(void);

uint8_t Elevate_control(int32_t position, uint32_t error_range);
uint8_t Underframe_control(int32_t position, uint32_t error_range);
uint8_t Turntable_control(int32_t position, uint32_t error_range);

void Rotary_servo_motor_control(uint8_t angle, uint16_t delay_time_before, uint16_t delay_time_after);
void Gripper_servo_motor_control(uint8_t angle, uint16_t delay_time_before, uint16_t delay_time_after);

uint8_t process_b_signal(openmvTypeDef *A_openmv, openmvTypeDef *B_openmv);
uint8_t get_current_b_index(void);
uint8_t get_b_process_complete(void);

void Go_to_B_detect(void);
void Go_to_A_abc_detect(void);
void Go_to_A_def_detect(void);

void Go_to_A_a_Up(void);
void Go_to_A_b_Up(void);
void Go_to_A_c_Up(void);
void Go_to_A_d_Up(void);
void Go_to_A_e_Up(void);
void Go_to_A_f_Up(void);
void Go_to_A_a_Down(void);
void Go_to_A_b_Down(void);
void Go_to_A_c_Down(void);
void Go_to_A_d_Down(void);
void Go_to_A_e_Down(void);
void Go_to_A_f_Down(void);

void Go_to_B_a(void);
void Go_to_B_b(void);
void Go_to_B_c(void);
void Go_to_B_d(void);
void Go_to_B_e(void);
void Go_to_B_f(void);

void execute_functions(void);

#endif
