#include "main.h"
#include "usart.h"
#include "uart.h"
#include "my_value.h"
#include "Emm_V5.h"
#include "OLED.h"
#include <string.h>
carTypeDef Underframe;
carTypeDef Turntable;
carTypeDef Elevate;
openmvTypeDef A_openmv;
openmvTypeDef B_openmv;
extern uint8_t counter;
//uint8_t HEX_Num[5];

/**
 * @brief   USART发送一个字节
 * @param   无
 * @retval  无
 */
// 发送单个字节数据到指定串口
void uart_SendByte(UART_HandleTypeDef *huart, uint8_t data)
{
    HAL_UART_Transmit(huart, &data, 1, 1000);
}

/**
 * @brief   向指定串口发送多个字节
 * @param   huart: 串口句柄指针
 *          cmd: 要发送的字节数组指针
 *          len: 要发送的字节长度
 * @retval  无
 */
void uart_SendCmd(UART_HandleTypeDef *huart, uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0;
    for (i = 0; i < len; i++)
    {
        uart_SendByte(huart, cmd[i]);
    }
}


// 提示 openmv 开始发送数据
void Start_Detect(openmvTypeDef *openmv, uint8_t process_flags, uint8_t counter_val, uint8_t *counter_ptr)
{
		openmv->Ending_flags = 0;
    // 根据设备类型选择对应的 UART 句柄
    UART_HandleTypeDef *target_huart = NULL;
    switch (openmv->deviceType) {
        case 1:
            target_huart = &huart1;
            break;
        case 2:
            target_huart = &huart2;
            break;
        default:
            return;
    }

    // 启动串口接收中断
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(target_huart, openmv->data, 1);
    if (status != HAL_OK) {
        // 处理接收中断启动失败的情况
        return;
    }
		
    // 设置处理标志位
    openmv->process_flags = process_flags;
    // 更新 counter_ptr 指向的值
    *counter_ptr = counter_val;

    // 等待结束标志位置 1
    while (!openmv->Ending_flags) 
		{
			Delay_ms(100);
        // 可以添加超时机制，避免无限等待
        // 例如使用 HAL_GetTick() 函数进行超时判断
    }

    // 标志位清零
//    openmv->Ending_flags = 0;
}
// 提示 openmv 停止发送数据
void End_Detect(openmvTypeDef *openmv)
{
    // 检查输入指针是否为空
    if (openmv == NULL) {
        return;
    }

    // 根据设备类型选择对应的 UART 句柄
    UART_HandleTypeDef *target_huart = NULL;
    switch (openmv->deviceType) {
        case 1:
            target_huart = &huart1;
            break;
        case 2:
            target_huart = &huart2;
            break;
        default:
            return;
    }

    // 停止串口接收中断
    HAL_UART_AbortReceive_IT(target_huart);

//    // 重置处理标志位
//    openmv->process_flags = 0;

//    // 确保结束标志位为 0
//    openmv->Ending_flags = 0;
}
// 初始化 openmvTypeDef 结构体的函数，移除 state 参数并初始化 SignalTypeDef 成员，可根据设备类型赋值 xywh
void init_openmv(openmvTypeDef *openmv, uint8_t deviceType)
{

    openmv->deviceType = deviceType;
    // 初始化 SignalTypeDef 实例的 x, y, w, h
    if (deviceType == 1) 
		{
        // 假设 deviceType 为 1 是 A_openmv
        openmv->a.x = 50;
        openmv->a.y = 185;
        openmv->a.w = 90;
        openmv->a.h = 90;

        openmv->b.x = 107;
        openmv->b.y = 18;
        openmv->b.w = 90;
        openmv->b.h = 90;

        openmv->c.x = 211;
        openmv->c.y = 23;
        openmv->c.w = 90;
        openmv->c.h = 90;

        openmv->d.x = 33;
        openmv->d.y = 184;
        openmv->d.w = 90;
        openmv->d.h = 90;

        openmv->e.x = 40;
        openmv->e.y = 73;
        openmv->e.w = 90;
        openmv->e.h = 90;

        openmv->f.x = 195;
        openmv->f.y = 33;
        openmv->f.w = 90;
        openmv->f.h = 90;
		} 
		else 
		{
        // 其他情况（比如 deviceType 为 2 是 B_openmv）
        openmv->a.x = 43;
        openmv->a.y = 191;
        openmv->a.w = 60;
        openmv->a.h = 60;

        openmv->b.x = 119;
        openmv->b.y = 14;
        openmv->b.w = 60;
        openmv->b.h = 60;

        openmv->c.x = 234;
        openmv->c.y = 90;
        openmv->c.w = 60;
        openmv->c.h = 60;

        openmv->d.x = 20;
        openmv->d.y = 152;
        openmv->d.w = 60;
        openmv->d.h = 60;

        openmv->e.x = 127;
        openmv->e.y = 152;
        openmv->e.w = 60;
        openmv->e.h = 60;

        openmv->f.x = 234;
        openmv->f.y = 152;
        openmv->f.w = 60;
        openmv->f.h = 60;
		}
		// 初始化 num_score 数组为 0
    for (int i = 0; i < 7; i++)
    {
        openmv->a.num_score[i] = 0.0f;
        openmv->b.num_score[i] = 0.0f;
        openmv->c.num_score[i] = 0.0f;
        openmv->d.num_score[i] = 0.0f;
        openmv->e.num_score[i] = 0.0f;
        openmv->f.num_score[i] = 0.0f;
    }
    // 初始化 num 为 0
    openmv->a.num = 0;
    openmv->b.num = 0;
    openmv->c.num = 0;
    openmv->d.num = 0;
    openmv->e.num = 0;
    openmv->f.num = 0;
		// 初始化 up 和 down 为 0
    openmv->a.up = 0;
    openmv->a.down = 0;
    openmv->b.up = 0;
    openmv->b.down = 0;
    openmv->c.up = 0;
    openmv->c.down = 0;
    openmv->d.up = 0;
    openmv->d.down = 0;
    openmv->e.up = 0;
    openmv->e.down = 0;
    openmv->f.up = 0;
    openmv->f.down = 0;
		openmv->Ending_flags=0;
}

// 简化后的初始化函数
void init_all_openmv(openmvTypeDef *A_openmv, openmvTypeDef *B_openmv)
{
    init_openmv(A_openmv, 1);
    init_openmv(B_openmv, 2);
}

int32_t convert_to_int32(uint8_t hex_data[5])
{
    int32_t result = 0;

    // 第一个字节用于判断符号
    uint8_t sign = hex_data[0];

    // 提取数值部分（接下来的4个字节）
    result = (hex_data[1] << 24) | (hex_data[2] << 16) | (hex_data[3] << 8) | hex_data[4];

    // 根据符号调整结果
    if (sign == 0x01)
    { // 01表示负数
        result = -result;
    }

    return result;
}

#define PACKET_TAIL 0x6B // 定义数据包的尾部标识，其值为十六进制的 0x6B
#define PACKET_HEAD 0xAA // 定义数据包的头部标识，其值为十六进制的 0xAA
void motor_data_process(carTypeDef *ActuationSystem)
{
    static uint8_t receive_state = 0;  // 静态变量，用于记录当前的接收状态
    static uint8_t packet_length = 0;  // 静态变量，用于记录当前数据包的长度
    static uint8_t motor_index = 0;    // 静态变量，用于记录当前处理的电机索引
    static uint8_t i = 0;              // 静态变量，用于循环计数
    uint8_t HEX_Num_1[5];                // 定义一个数组，用于存储从接收到的数据中提取的 5 个十六进制数
		uint8_t HEX_Num_2[5];
    switch (receive_state)
    {
    case 0:
        if (ActuationSystem->data[0] >= 0x01 && ActuationSystem->data[0] <= 0x04)
        {
            motor_index = ActuationSystem->data[0] - 1;  // 计算电机索引，将接收到的数据减 1
            receive_state = 1;                // 切换到状态 1，表示开始接收数据包
            packet_length = 1;                // 初始化数据包长度为 1
            ActuationSystem->data[packet_length] = ActuationSystem->data[0];  // 将接收到的第一个数据存入 ActuationSystem->data 数组中
            packet_length++;  // 数据包长度加 1
        }
        break;
    case 1:  // 接收数据包的主体部分
        if (packet_length < 36)
        {
            ActuationSystem->data[packet_length] = ActuationSystem->data[0];  // 将接收到的数据存入 ActuationSystem->data 数组中
            packet_length++;  // 数据包长度加 1
        }

        // 数据接受完成，开始处理数据
        if (ActuationSystem->data[0] == PACKET_TAIL && packet_length >= 2)
        {
            MotorTypeDef *motors[4] = {&ActuationSystem->motor1, &ActuationSystem->motor2, &ActuationSystem->motor3, &ActuationSystem->motor4};  // 定义一个指针数组，指向 4 个电机结构体
            for (i = 0; i < 36; i++)
            {
                motors[motor_index]->received_data[i] = ActuationSystem->data[i + 1];  // 将接收到的数据复制到对应的电机结构体的 received_data 数组中
            }
//            motors[motor_index]->motor_received_flag = 1;  // 标记该电机已接收到数据

						
            if (motors[motor_index]->received_data[1] == Position_mode)/////////////////////处理是否到位
            {
                if (motors[motor_index]->received_data[2] == In_place)
                    motors[motor_index]->arrived_flag = 1;  // 标记该电机已到达指定位置
                else if (motors[motor_index]->received_data[2] == Correct_return)
                    motors[motor_index]->motor_received_flag = 2;  // 标记该电机已收到数据并成功返回数据
                for (i = 0; i < 36; i++)
                {
                    motors[motor_index]->position_data[i] = motors[motor_index]->received_data[i];  // 将接收到的数据复制到对应的电机结构体的 position_data 数组中
                }
            }

						
            if (motors[motor_index]->received_data[1] == Read_pulse_count)/////////////////////处理编码器值
            {
                for (i = 0; i < 5; i++)
                {
                    HEX_Num_1[i] = motors[motor_index]->received_data[i + 2];  // 从接收到的数据中提取 5 个十六进制数存入 HEX_Num 数组中
                }
                motors[motor_index]->Pulse_Value = convert_to_int32(HEX_Num_1);  // 将提取的 5 个十六进制数转换为 32 位整数，并赋值给对应的电机结构体的 Pulse_Value 成员
                for (i = 0; i < 36; i++)
                {
                    motors[motor_index]->Pulse_Value_data[i] = motors[motor_index]->received_data[i];  // 将接收到的数据复制到对应的电机结构体的 Pulse_Value_data 数组中
                }
            }

						
						if (motors[motor_index]->received_data[1] == Read_Degree_count)/////////////////////处理编码器值
            {
                for (i = 0; i < 5; i++)
                {
                    HEX_Num_2[i] = motors[motor_index]->received_data[i + 2];  // 从接收到的数据中提取 5 个十六进制数存入 HEX_Num 数组中
                }
                motors[motor_index]->Degree_Value = convert_to_int32(HEX_Num_2);  // 将提取的 5 个十六进制数转换为 32 位整数，并赋值给对应的电机结构体的 Pulse_Value 成员
								motors[motor_index]->Degree_Value /=(float)20.48;
//                for (i = 0; i < 36; i++)
//                {
//                    motors[motor_index]->Pulse_Value_data[i] = motors[motor_index]->received_data[i];  // 将接收到的数据复制到对应的电机结构体的 Pulse_Value_data 数组中
//                }
            }
						
						
            receive_state = 0;  // 切换到状态 0，准备接收下一个数据包
        }
        break;
    }
}

//// 计算 process_flags 中 1 的个数
//int count_ones(uint8_t process_flags)
//{
//    int count = 0;
//    while (process_flags)
//    {
//        count += process_flags & 1;
//        process_flags >>= 1;
//    }
//    return count;
//}
// openmv_data_process 函数，根据标志位处理 SignalTypeDef 实例
void openmv_data_process(openmvTypeDef *openmv_item, uint8_t *counter_ptr)
{
    static uint8_t receive_state = 0;  // 静态变量，用于记录当前的接收状态
    static uint8_t packet_length = 0;  // 静态变量，用于记录当前数据包的长度
    // 新增静态变量用于记录接收到完整数据包的次数
    static uint8_t packet_count = 0;

    switch (receive_state)
    {
        case 0:
            if (openmv_item->data[0] == PACKET_HEAD)
            {
                receive_state = 1;  // 切换到状态 1，表示开始接收数据包
                packet_length = 2;  // 初始化数据包长度为 2，因为 1 已经被处理了
                openmv_item->data[1] = PACKET_HEAD;  // 将 PACKET_HEAD 存到 data[1]
            }
            break;
        case 1:  // 接收数据包的主体部分
            if (packet_length < 36)
            {
                openmv_item->data[packet_length] = openmv_item->data[0];  // 将接收到的数据存入 data 数组中
                packet_length++;  // 数据包长度加 1
            }
            if (openmv_item->data[0] == PACKET_TAIL && packet_length >= 2)
            {
                if (openmv_item->data[1] == PACKET_HEAD && openmv_item->data[packet_length - 1] == PACKET_TAIL)
                {
                    packet_count++;
                    // 解析接收到的数据
                    uint8_t detected_num = openmv_item->data[2];
                    uint8_t detected_x = openmv_item->data[3];
                    uint8_t detected_y = openmv_item->data[4];
                    float score = (float)openmv_item->data[5] / 255.0f;

                    for (int i = 0; i < 6; i++)
                    {
                        uint8_t mask = 1 << i;
                        if (openmv_item->process_flags & mask)
                        {
                            SignalTypeDef *signal;
                            switch (i)
                            {
                                case 0:
                                    signal = &openmv_item->a;
                                    break;
                                case 1:
                                    signal = &openmv_item->b;
                                    break;
                                case 2:
                                    signal = &openmv_item->c;
                                    break;
                                case 3:
                                    signal = &openmv_item->d;
                                    break;
                                case 4:
                                    signal = &openmv_item->e;
                                    break;
                                case 5:
                                    signal = &openmv_item->f;
                                    break;
                            }
                            if (detected_x >= (signal->x - signal->w / 2) && detected_x <= (signal->x + signal->w / 2) &&
                                detected_y >= (signal->y - signal->h / 2) && detected_y <= (signal->y + signal->h / 2))
                            {
                                if (detected_num < 7)
                                {
                                    signal->num_score[detected_num] += score;
                                }
                            }
                        }
                    }

                    if (packet_count >= *counter_ptr)
                    {
                        packet_count = 0;

                        // 对 num_score 数组进行处理
                        for (int i = 0; i < 6; i++)
                        {
                            uint8_t mask = 1 << i;
                            if (openmv_item->process_flags & mask)
                            {
                                SignalTypeDef *signal;
                                switch (i)
                                {
                                    case 0:
                                        signal = &openmv_item->a;
                                        break;
                                    case 1:
                                        signal = &openmv_item->b;
                                        break;
                                    case 2:
                                        signal = &openmv_item->c;
                                        break;
                                    case 3:
                                        signal = &openmv_item->d;
                                        break;
                                    case 4:
                                        signal = &openmv_item->e;
                                        break;
                                    case 5:
                                        signal = &openmv_item->f;
                                        break;
                                }
                                // 除以 (float)*counter_ptr 与 process_flags 中 1 的个数的乘积
//                                int ones_count = count_ones(openmv_item->process_flags);
//                                for (int j = 0; j < 7; j++)
//                                {
//                                    signal->num_score[j] /= ((float)*counter_ptr / ones_count);
//                                }
                                // 找到最大值和最大值的序号
                                float max_score = 0.0f;
                                uint8_t max_index = 0;
//                                int all_less_than_half = 1;
                                for (int j = 0; j < 7; j++)
                                {
                                    if (signal->num_score[j] > max_score)
                                    {
                                        max_score = signal->num_score[j];
                                        max_index = j;
                                    }
//                                    if (signal->num_score[j] >= 0.5f)
//                                    {
//                                        all_less_than_half = 0;
//                                    }
                                }
																signal->num = max_index;
//                                // 根据规则更新 num
//                                if (all_less_than_half)
//                                {
//                                    signal->num = 0;
//                                }
//                                else
//                                {
//                                    signal->num = max_index;
//                                }
                            }
                        }openmv_item->Ending_flags=1;
                    }
                }
                receive_state = 0;  // 切换到状态 0，准备接收下一个数据包
                packet_length = 0;  // 重置数据包长度
            }
            break;
    }
}

// 全局函数指针数组（索引0-11对应顺序1-12）
GoToFunction goToFunctions[12] = {0};  // 初始化为空

void process_openmv(openmvTypeDef *A_openmv, openmvTypeDef *B_openmv) {
    SignalTypeDef *A_signals[] = {&A_openmv->a, &A_openmv->b, &A_openmv->c, &A_openmv->d, &A_openmv->e, &A_openmv->f};
    SignalTypeDef *B_signals[] = {&B_openmv->a, &B_openmv->b, &B_openmv->c, &B_openmv->d, &B_openmv->e, &B_openmv->f};

    // ---------------------- 数字处理逻辑（保持不变） ----------------------
    // 1. 处理A_openmv中num为0的情况
    for (int i = 0; i < 6; i++) {
        if (A_signals[i]->num == 0) {
            if (i == 0) {
                A_openmv->f.up = 12;
            } else {
                A_openmv->a.up = 12;
            }
            break;
        }
    }

    // 2. 查找A中缺失的num并设置B的down=11
    uint8_t missing_num = 0;
    int found[7] = {0};
    for (int i = 0; i < 6; i++) {
        if (A_signals[i]->num > 0 && A_signals[i]->num <= 6) {
            found[A_signals[i]->num] = 1;
        }
    }
    for (int i = 1; i <= 6; i++) {
        if (!found[i]) {
            missing_num = i;
            break;
        }
    }
    for (int i = 0; i < 6; i++) {
        if (B_signals[i]->num == missing_num) {
            B_signals[i]->down = 11;
            break;
        }
    }

    // 3. 为B_openmv分配奇数down值（1,3,5,7,9）
    uint8_t odd_num = 1;
    for (int i = 0; i < 6; i++) {
        if (B_signals[i]->down == 0) {
            B_signals[i]->down = odd_num;
            odd_num += 2;
            if (odd_num > 9) break;
        }
    }

    // 4. 为A_openmv分配偶数down值（B的down+1）
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (A_signals[i]->num == B_signals[j]->num) {
                A_signals[i]->down = B_signals[j]->down + 1;
            }
        }
    }

    // ---------------------- 函数指针填充逻辑（基于数字处理结果） ----------------------
    // 清空数组（避免残留旧数据）
    memset(goToFunctions, 0, sizeof(goToFunctions));

    // 1. 填充A_openmv的Down函数（值2,4,6,8,10→索引1,3,5,7,9）
    for (int i = 0; i < 6; i++) {
        uint8_t down = A_signals[i]->down;
        if (down > 0 && down <= 11) {
            int index = down - 1;  // down=2→index1（第2位）
            switch (i) {
                case 0: goToFunctions[index] = Go_to_A_a_Down; break;
                case 1: goToFunctions[index] = Go_to_A_b_Down; break;
                case 2: goToFunctions[index] = Go_to_A_c_Down; break;
                case 3: goToFunctions[index] = Go_to_A_d_Down; break;
                case 4: goToFunctions[index] = Go_to_A_e_Down; break;
                case 5: goToFunctions[index] = Go_to_A_f_Down; break;
            }
        }
    }

    // 2. 填充B_openmv的Down函数（值1,3,5,7,9,11→索引0,2,4,6,8,10）
    for (int i = 0; i < 6; i++) {
        uint8_t down = B_signals[i]->down;
        if (down > 0 && down <= 11) {
            int index = down - 1;  // down=3→index2（第3位）
            switch (i) {
                case 0: goToFunctions[index] = Go_to_B_a; break;
                case 1: goToFunctions[index] = Go_to_B_b; break;
                case 2: goToFunctions[index] = Go_to_B_c; break;
                case 3: goToFunctions[index] = Go_to_B_d; break;
                case 4: goToFunctions[index] = Go_to_B_e; break;
                case 5: goToFunctions[index] = Go_to_B_f; break;
            }
        }
    }

    // 3. 填充A_openmv的Up函数（up=12→固定索引11，第12位）
    for (int i = 0; i < 6; i++) {
        if (A_signals[i]->up == 12) {
            switch (i) {
                case 0: goToFunctions[11] = Go_to_A_a_Up; break;
                case 1: goToFunctions[11] = Go_to_A_b_Up; break;
                case 2: goToFunctions[11] = Go_to_A_c_Up; break;
                case 3: goToFunctions[11] = Go_to_A_d_Up; break;
                case 4: goToFunctions[11] = Go_to_A_e_Up; break;
                case 5: goToFunctions[11] = Go_to_A_f_Up; break;
            }
            break;  // 仅允许一个Up函数
        }
    }
}

// 按顺序调用函数（1-12位）
void execute_functions(void) {
    for (int i = 0; i < 12; i++) {
        if (goToFunctions[i]) {
            goToFunctions[i]();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
    if (huart->Instance == UART4)
    {
        motor_data_process(&Underframe);
        HAL_UART_Receive_IT(&huart4, Underframe.data, 1);
    }
    else if (huart->Instance == UART5)
    {
        motor_data_process(&Turntable);
        HAL_UART_Receive_IT(&huart5, Turntable.data, 1);
    }
    else if (huart->Instance == USART6)
    {
        motor_data_process(&Elevate);
        HAL_UART_Receive_IT(&huart6, Elevate.data, 1);
    }
    else if (huart->Instance == USART1)
    {
        
				openmvTypeDef *openmv = &A_openmv;
				if(!openmv->Ending_flags)
				{
					HAL_UART_Receive_IT(&huart1, openmv->data, 1);
					openmv_data_process(&A_openmv,&counter);
				}
        
    }	
		else if (huart->Instance == USART2)
    {
        
				openmvTypeDef *openmv = &B_openmv;
				if(!openmv->Ending_flags)
				{
					HAL_UART_Receive_IT(&huart2, openmv->data, 1);
					openmv_data_process(&B_openmv,&counter);
        }
    }
		
}    


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM7) 
		{


				Read_Degree_Value(&huart4,1);
				Delay_us(100);
				Read_Degree_Value(&huart5,0);
				Delay_us(100);
				Read_Degree_Value(&huart6,0);
				Delay_us(100);
//			OLED_ShowNum(1, 1, Underframe.motor1.arrived_flag, 2);
//			OLED_ShowHexNum(1, 4, Turntable.motor1.arrived_flag, 2);
//			OLED_ShowHexNum(1, 7, Elevate.motor1.arrived_flag, 2);
//			OLED_ShowSignedNum(1,1,Underframe.motor1.Degree_Value,8);
			
//			OLED_ShowSignedNum(2,1,Turntable.motor1.Degree_Value,8);
//			
//			OLED_ShowSignedNum(3,1,Elevate.motor1.Degree_Value,8);
//			OLED_ShowSignedNum(1,7,Underframe.motor2.Degree_Value,8);
//			OLED_ShowHexNum(1,1,A_openmv.data[0],3);
//			OLED_ShowHexNum(1,1,A_openmv.data[1],3);
//			OLED_ShowNum(1,5,A_openmv.data[2],3);
////			OLED_ShowNum(1,9,A_openmv.data[3],3);
////			OLED_ShowNum(1,13,A_openmv.data[4],3);
////			OLED_ShowNum(2,1,A_openmv.data[5],3);
////			OLED_ShowHexNum(2,5,A_openmv.data[6],3);
//			openmvTypeDef *pA = &A_openmv;
//			OLED_ShowNum(2,9,pA->Ending_flags,3);
			
//			OLED_ShowHexNum(1,1,B_openmv.data[0],3);
//			OLED_ShowHexNum(1,5,B_openmv.data[1],3);
//			OLED_ShowNum(1,9,B_openmv.data[2],3);
//			OLED_ShowNum(1,13,B_openmv.data[3],3);
//			OLED_ShowNum(2,1,B_openmv.data[4],3);

			
			
//			openmvTypeDef *pB = &B_openmv;
//			OLED_ShowNum(2,9,pB->Ending_flags,3);
				
//			OLED_ShowNum(2,13,A_openmv.a.num,3);
//			OLED_ShowHexNum(2,9,A_openmv.data[7],3);
			
			
			
		}   
}
//// 创建函数指针数组，按索引0-5对应a-f的Down函数
//const DownFunction down_functions[6] = {
//    Go_to_A_a_Down,
//    Go_to_A_b_Down,
//    Go_to_A_c_Down,
//    Go_to_A_d_Down,
//    Go_to_A_e_Down,
//    Go_to_A_f_Down
//};

//// 全局静态状态变量
//static uint8_t current_b_index = 0;     // 当前处理的B字母索引(0-5)
//static uint8_t b_process_complete = 0;  // 处理完成标志
//static uint8_t A_zero_index = 255;      // A中num=0的信号索引
//static uint8_t is_initialized = 0;      // 初始化完成标志

//// 实现处理函数（移除函数指针调用，改为返回索引）
//uint8_t process_b_signal(openmvTypeDef *A_openmv, openmvTypeDef *B_openmv) 
//{
//    if (b_process_complete) return 255; // 返回无效索引

//    SignalTypeDef *A_signals[] = {&A_openmv->a, &A_openmv->b, &A_openmv->c, &A_openmv->d, &A_openmv->e, &A_openmv->f};
//    SignalTypeDef *current_B_signal = NULL;
//    uint8_t function_index = 255; // 初始化为无效值

//    // 首次处理时查找A中的0信号索引
//    if (current_b_index == 0 && !is_initialized) {
//        for (int i = 0; i < 6; i++) {
//            if (A_signals[i]->num == 0) {
//                A_zero_index = i;
//                break;
//            }
//        }
//        is_initialized = 1;
//    }

//    // 获取当前处理的B字母
//    switch (current_b_index) {
//        case 0: current_B_signal = &B_openmv->a; break;
//        case 1: current_B_signal = &B_openmv->b; break;
//        case 2: current_B_signal = &B_openmv->c; break;
//        case 3: current_B_signal = &B_openmv->d; break;
//        case 4: current_B_signal = &B_openmv->e; break;
//        case 5: current_B_signal = &B_openmv->f; break;
//        default: return 255;
//    }

//    // 分配B的down值
//    uint8_t b_down = 2 * (current_b_index + 1) - 1;
//    current_B_signal->down = b_down;

//    // 查找A中匹配的信号
//    uint8_t target_num = current_B_signal->num;
//    for (int i = 0; i < 6; i++) {
//        if (A_signals[i]->num == target_num && A_signals[i]->num != 0) {
//            function_index = i;
//            break;
//        }
//    }

//    if (function_index == 255 && A_zero_index != 255) {
//        function_index = A_zero_index;
//    }

//    // 设置A的down值，但不调用函数
//    if (function_index < 6) {
//        A_signals[function_index]->down = b_down + 1;
//    }

//    // 步进至下一个B字母
//    if (current_b_index < 5) {
//        current_b_index++;
//    } else {
//        b_process_complete = 1;
//        current_b_index = 0;
//        is_initialized = 0;
//    }

//    return function_index; // 返回匹配的函数索引，由外部调用
//}

//// 实现获取状态的函数
//uint8_t get_current_b_index(void) {
//    return current_b_index;
//}

//uint8_t get_b_process_complete(void) {
//    return b_process_complete;
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	
//    if (huart->Instance == UART4)
//    {
//        motor_data_process(&Underframe);
//        HAL_UART_Receive_IT(&huart4, Underframe.data, 1);
//    }
//    else if (huart->Instance == UART5)
//    {
//        motor_data_process(&Turntable);
//        HAL_UART_Receive_IT(&huart5, Turntable.data, 1);
//    }
//    else if (huart->Instance == USART6)
//    {
//        motor_data_process(&Elevate);
//        HAL_UART_Receive_IT(&huart6, Elevate.data, 1);
//    }
//    else if (huart->Instance == USART1)
//    {
//        
//				openmvTypeDef *openmv = &A_openmv;
//				if(!openmv->Ending_flags)
//				{
//					HAL_UART_Receive_IT(&huart1, openmv->data, 1);
//					openmv_data_process(&A_openmv,&counter);
//				}
//        
//    }	
//		else if (huart->Instance == USART2)
//    {
//        
//				openmvTypeDef *openmv = &B_openmv;
//				if(!openmv->Ending_flags)
//				{
//					HAL_UART_Receive_IT(&huart2, openmv->data, 1);
//					openmv_data_process(&B_openmv,&counter);
//        }
//    }
//		
//}    
