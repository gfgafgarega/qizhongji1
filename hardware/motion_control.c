#include "main.h"
#include "usart.h"
#include "uart.h"
#include "my_value.h"
#include "Emm_V5.h"
#include "tim.h"
#include <string.h>
extern carTypeDef Underframe;
extern carTypeDef Turntable;
extern carTypeDef Elevate;

extern volatile int32_t e;  // 声明外部变量e


// 封装求 min_val 和 max_val 的函数
void get_min_max(int32_t a, int32_t b, int32_t *min_val, int32_t *max_val)
{
    if (a < b) {
        *min_val = a;
        *max_val = b;
    } else {
        *min_val = b;
        *max_val = a;
    }
}

uint8_t wait_for_motor_arrival(MotorTypeDef *motor, int32_t min_val, int32_t max_val) {
   
    while (!(motor->Degree_Value >= min_val && motor->Degree_Value <= max_val) && !motor->arrived_flag) {
        Delay_ms(100);  // 轮询间隔，避免CPU满载

    }
    return SUCCESS;  // 只要退出循环即视为成功（标志位或位置达标）
}

// 统一使用 error_range 参数替代原 Pulse_Value
uint8_t Underframe_control(int32_t position, uint32_t error_range) 
{
//    #define CMD_SEND_TIMES 3  // 命令发送次数
    
    Delay_ms(2);
    while (!Underframe.motor1.arrived_flag) Delay_ms(100);

    if (position != 0) {
        Underframe.motor1.arrived_flag = 0;
        
        // 重复发送电机控制命令多次
     while (!Underframe.motor1.motor_received_flag) 
     {
            Emm_V5_Pos_Control(&huart4, 1, position+e, Underframe.motor1.Underframe_vel, Underframe.motor1.Underframe_acc, absolute_movement, multi_machine_synchonization_false);
					  Emm_V5_Pos_Control(&huart4, 2, position+e, Underframe.motor2.Underframe_vel, Underframe.motor2.Underframe_acc, absolute_movement, multi_machine_synchonization_false);
					  Emm_V5_Pos_Control(&huart4, 3, position+add_position+e, Underframe.motor3.Underframe_vel, Underframe.motor3.Underframe_acc, absolute_movement, multi_machine_synchonization_false);
					  Emm_V5_Pos_Control(&huart4, 4, position+add_position+e, Underframe.motor4.Underframe_vel, Underframe.motor4.Underframe_acc, absolute_movement, multi_machine_synchonization_false);
            Delay_ms(5);  // 每次发送间隔一小段时间
     }

        if (error_range != 0) {
            int32_t min_val = position - error_range;
            int32_t max_val = position + error_range;
            wait_for_motor_arrival(&Underframe.motor1, min_val, max_val);
        }
    }
    
    Underframe.motor1.arrived_flag = 1;
    Delay_ms(2);
    return SUCCESS;
}

uint8_t Turntable_control(int32_t position, uint32_t error_range) 
{
    #define CMD_SEND_TIMES 3  // 命令发送次数
    
    Delay_ms(2);
    while (!Turntable.motor1.arrived_flag) Delay_ms(100);

    if (position != 0) {
        Turntable.motor1.arrived_flag = 0;
        
        // 重复发送电机控制命令多次
        for (int i = 0; i < CMD_SEND_TIMES; i++) {
            Emm_V5_Pos_Control(&huart5, 0, position, Turntable_vel, Turntable_acc, absolute_movement, multi_machine_synchonization_false);
            Delay_ms(5);  // 每次发送间隔一小段时间
        }

        if (error_range != 0) {
            int32_t min_val = position - error_range;
            int32_t max_val = position + error_range;
            wait_for_motor_arrival(&Turntable.motor1, min_val, max_val);
        }
    }
    
    // 注：原注释称"应由驱动自动置1"，但实际所有控制函数均有此操作，保持一致
    Turntable.motor1.arrived_flag = 1;
    Delay_ms(2);
    return SUCCESS;
}

uint8_t Elevate_control(int32_t position, uint32_t error_range) 
{
    #define CMD_SEND_TIMES 3  // 命令发送次数
    
    Delay_ms(2);
    while (!Elevate.motor1.arrived_flag) Delay_ms(100);

    if (position != 0) {
        Elevate.motor1.arrived_flag = 0;
        
        // 重复发送电机控制命令多次
        for (int i = 0; i < CMD_SEND_TIMES; i++) {
            Emm_V5_Pos_Control(&huart6, 0, position, Elevate_vel, Elevate_acc, absolute_movement, multi_machine_synchonization_false);
            Delay_ms(5);  // 每次发送间隔一小段时间
        }

        if (error_range != 0) {
            int32_t min_val = position - error_range;
            int32_t max_val = position + error_range;
            wait_for_motor_arrival(&Elevate.motor1, min_val, max_val);
        }
    }
    
    Elevate.motor1.arrived_flag = 1;
    Delay_ms(2);
    return SUCCESS;
}
// 旋转舵机控制函数
void Rotary_servo_motor_control(uint8_t angle, uint16_t delay_time_before, uint16_t delay_time_after) {
		Delay_ms(2);
    if (delay_time_before > 0) {
        Delay_ms(delay_time_before);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, angle);
    if (delay_time_after > 0) {
        Delay_ms(delay_time_after);
    }
		Delay_ms(2);
}

// 夹爪舵机控制函数
void Gripper_servo_motor_control(uint8_t angle, uint16_t delay_time_before, uint16_t delay_time_after) {
		Delay_ms(2);
    if (delay_time_before > 0) {
        Delay_ms(delay_time_before);
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle);
    if (delay_time_after > 0) {
        Delay_ms(delay_time_after);
    }
		Delay_ms(2);
}


void Go_to_A_abc_detect(void) 
{
	Turntable_control(-10000,0);
	Elevate_control(-27500,300);
}

void Go_to_A_def_detect(void) 
{
	
	Turntable_control(-21100,300);
}

void Go_to_B_detect(void)
{
	Turntable_control(-9000,1000);
	Underframe_control(13800,500);
}




/*-----------------------------------------------------------------*/
void Go_to_A_a_Down(void) 
{ 
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(30,100,100);
	Underframe_control(-600,300);
	Turntable_control(-2150,300);//-12150
	
	Elevate_control(-3800,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,3000);
}

void Go_to_A_a_Up(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(30,100,100);
	Underframe_control(-600,300);
	Turntable_control(-2150,0);
	
	Elevate_control(-14300,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
}
/*-----------------------------------------------------------------*/

void Go_to_A_b_Down(void) 
{
	
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(42,100,100);
	Turntable_control(-2750,0);
	Underframe_control(-3300,300);
	
	Elevate_control(-3800,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,3000);
}

void Go_to_A_b_Up(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(42,100,100);
	Turntable_control(-2750,0);
	Underframe_control(-3300,300);
	Elevate_control(-14300,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
}
/*-----------------------------------------------------------------*/
void Go_to_A_c_Down(void) 
{ 
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(35,100,100);
	Underframe_control(-300,1000);
	Turntable_control(-3550,1000);
	
	Elevate_control(-3800,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
	Turntable_control(-3000,3000);
}
void Go_to_A_c_Up(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(35,100,100);
	Underframe_control(-300,1000);
	Turntable_control(-3550,1000);
	
	Elevate_control(-14300,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
	Turntable_control(-3000,2000);
}
/*-----------------------------------------------------------------*/
void Go_to_A_d_Down(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(28,100,100);
	Underframe_control(-180,1000);
	Turntable_control(-4150,1000);//23400
	
	Elevate_control(-3800,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
	
	Turntable_control(-3000,3000);
}
void Go_to_A_d_Up(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(28,100,100);
	Underframe_control(-180,1000);
	Turntable_control(-4150,1000);
	
	Elevate_control(-14300,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
	
	Turntable_control(-3000,2000);
}
/*-----------------------------------------------------------------*/
void Go_to_A_e_Down(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(20,100,100);
	Underframe_control(-300,3000);
	Turntable_control(-4900,2000);
	Underframe_control(-3300,300);
	
	Elevate_control(-3800,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1500);
	
	Underframe_control(-300,1000);
	Turntable_control(-3000,2000);
}

void Go_to_A_e_Up(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(20,100,100);
	Underframe_control(-300,1000);
	Turntable_control(-4900,1000);
	Underframe_control(-3300,300);
	
	Elevate_control(-14300,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
	
	Underframe_control(-300,200);
	Turntable_control(-3000,2000);
}
/*-----------------------------------------------------------------*/
void Go_to_A_f_Down(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(36,100,100);
	Underframe_control(-1220,2000);
	Turntable_control(-5650,500);
	
	Elevate_control(-3800,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
	
	Turntable_control(-3000,1000);
	
}

void Go_to_A_f_Up(void) 
{
	Elevate_control(-20000,0);
	Rotary_servo_motor_control(36,100,100);
	Underframe_control(-1220,2000);
	Turntable_control(-5650,1000);
	
	Elevate_control(-3800,1000);
	Gripper_servo_motor_control(Gripper_open,300,300);
	Elevate_control(-20000,1000);
	
	Turntable_control(-3000,1000);
}
/*-----------------------------------------------------------------*/

/*-----------------------------------------------------------------*/
void Go_to_B_a(void) 
{
	Elevate_control(-1000,200);
	Turntable_control(-500,3000);
	Gripper_servo_motor_control(Gripper_close,0,0);
	Rotary_servo_motor_control(33, 0, 0);
	Elevate_control(-28000,0);
	Underframe_control(16600,3000);
	Turntable_control(1000,500);
	Delay_ms(1000);
	Gripper_servo_motor_control(Gripper_open,10,10);
	Underframe_control(21400,400);
	Turntable_control(900,500);
	Rotary_servo_motor_control(39,0,500);
	Elevate_control(-23700,1000);
	Gripper_servo_motor_control(Gripper_close,400,400);
	Elevate_control(-28300,800);
	
	Underframe_control(16600,200);
	Turntable_control(-500,6000);
}
void Go_to_B_d(void) 
{
	Turntable_control(-4000,2000);
	Gripper_servo_motor_control(Gripper_close,0,0);
	Rotary_servo_motor_control(33, 0, 0);
	Elevate_control(-3200,0);
	Underframe_control(16600,3000);
	Turntable_control(5000,500);
	Delay_ms(2000);
	Gripper_servo_motor_control(Gripper_open,10,10);
	Underframe_control(21250,400);
	
	Rotary_servo_motor_control(39,0,500);
	Elevate_control(-1,200);
	Gripper_servo_motor_control(Gripper_close,400,400);
	Elevate_control(-4000,800);
	
	Underframe_control(16600,200);
	Turntable_control(-9100,6000);
}
/*-----------------------------------------------------------------*/
void Go_to_B_b(void) 
{
	Elevate_control(-1000,200);
	Turntable_control(-500,2000);
	Gripper_servo_motor_control(Gripper_close,0,0);
	Rotary_servo_motor_control(33, 0, 0);
	Elevate_control(-30000,0);
	Underframe_control(16600,2000);
	Turntable_control(250,300);
	Delay_ms(500);
	Gripper_servo_motor_control(Gripper_open,0,0);
	Underframe_control(19650,300);
	Turntable_control(150, 0);
	
	Rotary_servo_motor_control(32,50,50);
	Elevate_control(-23700,1000);
	Gripper_servo_motor_control(Gripper_close,400,400);
	Elevate_control(-28300,1000);
	Underframe_control(16600,200);
	Turntable_control(-500,6000);
}

void Go_to_B_e(void) 
{
	Turntable_control(-4000,2000);
	Gripper_servo_motor_control(Gripper_close,0,0);
	Rotary_servo_motor_control(33, 0, 0);
	Elevate_control(-3200,0);
	Underframe_control(16600,2000);
	Turntable_control(1330,300);
	Delay_ms(500);

	Gripper_servo_motor_control(Gripper_open,0,0);
	Underframe_control(19600,300);
	
	Rotary_servo_motor_control(32,50,500);
	Elevate_control(-1,1000);
	Gripper_servo_motor_control(Gripper_close,400,400);
	Elevate_control(-4000,1000);
	
	Underframe_control(16600,200);
	Turntable_control(-9100,6000);
}
/*-----------------------------------------------------------------*/
void Go_to_B_c(void) 
{
//	Turntable_control(-4000,2000);
	Gripper_servo_motor_control(Gripper_open,0,0);
	Rotary_servo_motor_control(23, 0, 0);
	Elevate_control(-27000,1000);
	Turntable_control(-1650,300);
	Delay_ms(2000);
	Underframe_control(21350,400);
	
	Elevate_control(-22700,1000);
	Gripper_servo_motor_control(Gripper_close,400,400);
	Elevate_control(-27400,1000);
	Underframe_control(16600,200);
	Turntable_control(-9100,6000);
}

void Go_to_B_f(void) 
{
//	Turntable_control(-4000,2000);
	Gripper_servo_motor_control(Gripper_open,0,0);
	Rotary_servo_motor_control(23, 0, 0);
	Elevate_control(-3200,2000);
	Turntable_control(-1650,300);
	Delay_ms(2000);
	Underframe_control(21350,400);
	
	Elevate_control(-1,1000);
	Gripper_servo_motor_control(Gripper_close,400,400);
	Elevate_control(-4000,1000);
	Underframe_control(16600,200);
	Turntable_control(-9100,6000);
}

void execute_functions(void);

/*-----------------------------------------------------------------*/
// 电机初始化函数
void motor_init(void) {
    // 初始化 Underframe 的所有电机
    memset(Underframe.data, 0, sizeof(Underframe.data));
    Underframe.uart_received_flag = 0;

    memset(Underframe.motor1.received_data, 0, sizeof(Underframe.motor1.received_data));
    Underframe.motor1.arrived_flag = 1;
    Underframe.motor1.Pulse_Value = 0;
    memset(Underframe.motor1.position_data, 0, sizeof(Underframe.motor1.position_data));
    memset(Underframe.motor1.Pulse_Value_data, 0, sizeof(Underframe.motor1.Pulse_Value_data));
    Underframe.motor1.motor_received_flag = 0;
    Underframe.motor1.Degree_Value = 0;

    memset(Underframe.motor2.received_data, 0, sizeof(Underframe.motor2.received_data));
    Underframe.motor2.arrived_flag = 1;
    Underframe.motor2.Pulse_Value = 0;
    memset(Underframe.motor2.position_data, 0, sizeof(Underframe.motor2.position_data));
    memset(Underframe.motor2.Pulse_Value_data, 0, sizeof(Underframe.motor2.Pulse_Value_data));
    Underframe.motor2.motor_received_flag = 0;
    Underframe.motor2.Degree_Value = 0;

    memset(Underframe.motor3.received_data, 0, sizeof(Underframe.motor3.received_data));
    Underframe.motor3.arrived_flag = 1;
    Underframe.motor3.Pulse_Value = 0;
    memset(Underframe.motor3.position_data, 0, sizeof(Underframe.motor3.position_data));
    memset(Underframe.motor3.Pulse_Value_data, 0, sizeof(Underframe.motor3.Pulse_Value_data));
    Underframe.motor3.motor_received_flag = 0;
    Underframe.motor3.Degree_Value = 0;

    memset(Underframe.motor4.received_data, 0, sizeof(Underframe.motor4.received_data));
    Underframe.motor4.arrived_flag = 1;
    Underframe.motor4.Pulse_Value = 0;
    memset(Underframe.motor4.position_data, 0, sizeof(Underframe.motor4.position_data));
    memset(Underframe.motor4.Pulse_Value_data, 0, sizeof(Underframe.motor4.Pulse_Value_data));
    Underframe.motor4.motor_received_flag = 0;
    Underframe.motor4.Degree_Value = 0;

    // 初始化 Turntable 的 motor1
    memset(Turntable.data, 0, sizeof(Turntable.data));
    Turntable.uart_received_flag = 0;
    memset(Turntable.motor1.received_data, 0, sizeof(Turntable.motor1.received_data));
    Turntable.motor1.arrived_flag = 1;
    Turntable.motor1.Pulse_Value = 0;
    memset(Turntable.motor1.position_data, 0, sizeof(Turntable.motor1.position_data));
    memset(Turntable.motor1.Pulse_Value_data, 0, sizeof(Turntable.motor1.Pulse_Value_data));
    Turntable.motor1.motor_received_flag = 0;
    Turntable.motor1.Degree_Value = 0;

    // 初始化 Elevate 的 motor1
    memset(Elevate.data, 0, sizeof(Elevate.data));
    Elevate.uart_received_flag = 0;
    memset(Elevate.motor1.received_data, 0, sizeof(Elevate.motor1.received_data));
    Elevate.motor1.arrived_flag = 1;
    Elevate.motor1.Pulse_Value = 0;
    memset(Elevate.motor1.position_data, 0, sizeof(Elevate.motor1.position_data));
    memset(Elevate.motor1.Pulse_Value_data, 0, sizeof(Elevate.motor1.Pulse_Value_data));
    Elevate.motor1.motor_received_flag = 0;
    Elevate.motor1.Degree_Value = 0;
}
//假设A_openmv.a.num=2,A_openmv.b.num=4,A_openmv.c.num=0,A_openmv.d.num=1,A_openmv.e.num=3,A_openmv.f.num=6
//B_openmv.a.num=1,B_openmv.b.num=3,B_openmv.c.num=2,B_openmv.d.num=5,B_openmv.e.num=6,B_openmv.f.num=4
//先看A_openmv中谁为0(0代表1到6中缺失的数字)，发现是A_openmv.c.num（b,c,d,e,f不管谁为0都如此）就把A_openmv.a.up=12。如果A_openmv.a.num=0，就把A_openmv.f.up=12
//然后把B_openmv中对应缺失数字的对象的down设为11，比如上面案例为B_openmv.d.num=5，就把B_openmv.d.down=11。
//然后再把A_openmv和B_openmv中num相同的字母设为一组（注意是num相同，不是字母相同），比如上面案例中A_openmv.a.num=2=B_openmv.c.num，就把A_openmv.a和B_openmv.c分为一组。再把1 3 5 7 9按照字母abcdef(除去已经赋值的)顺序赋值给
//B_openmv.a.down,B_openmv.b.down...比如上面案列就是B_openmv.a.down=1，B_openmv.b.down=3，B_openmv.c.down=5，B_openmv.e.down=7，B_openmv.f.down=9（d已经赋值了）
//然后再把同组的A_openmv赋值2 4 6 8 10（为同组B_openmv的奇数数字加1），比如上面案例中A_openmv.d和B_openmv.a为一组，B_openmv.a.down=1，就把A_openmv.d=1+1。
//然后我写了Go_to_A_a(uint8_t floor)//floor为Down或Up,Go_to_A_b()等函数与A_openmv.a和A_openmv.b等一一对应，同时Go_to_A_a(Up)对应A_openmv.a.up。
//我想按照上面排好的1到12的顺序执行Go_to_A_a(Up)，Go_to_A_d(Down)，Go_to_B_d(Down)等函数

//现在改成每次调用一次Start_Detect(&B_openmv,0x20,120,&counter);就得到一个B_openmv.字母.num的值（1 2 3 4 5 6），第一次是B_openmv.a.num，然后调用process_openmv，把B_openmv.a.down赋值1，第二次得到B_openmv.b.num的值
//再把B_openmv.a.down赋值3，一直到f，然后B_openmv.f.down赋值11，然后每一次得到的一个B_openmv.字母.num后就和已知的A_openmv.字母.num（也是有6个值，不过1到6中有一个被替代成为0）进行匹配，把A_openmv.字母.num中值与每次
//测得的B_openmv.字母.num的值相等的结构体的dowm赋值，第一次赋值2，第二次为4，调用6次后把最后一个A_openmv.字母.down赋值12，比如第一次B_openmv.a.num=2=A_openmv.d.num，就把B_openmv.a.down=1，A_openmv.d.down=2，相当于B_openmv.a和A_openmv.d被划分到了一组
//如果某次测得的B_openmv.字母.num在A_openmv.字母.num中找不到（即A_openmv.字母.num中对应数字为0），就把这次的B_openmv.字母和A_openmv.字母划分到一组，然后对他们的结构体down执行上面的操作。
//每次处理完后调用那次与B_openmv.字母匹配的A_openmv.字母对应的函数，比如某次把A_openmv.c.num=0和B_openmv.d.num=5化为一组后，接着调用Go_to_A_c_Down，以此类推。



