#include "main.h"
#include "usart.h"
#include "uart.h"
#include "Emm_V5.h"


void Delay_us(uint32_t xus)
{
    
    uint32_t tickstart = HAL_GetTick(); 

    
    uint32_t delay = xus / 1000; 
    if (delay == 0) {
        delay = 1; 
    }

    while ((HAL_GetTick() - tickstart) < delay) {
       
    }
}

void Delay_ms(uint32_t Delay)
{
    
   uint32_t tickstart = HAL_GetTick();
	uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
  }
}


/**
  * @brief    位置模式
  * @param    huart: 串口句柄指针
  * @param    addr：电机地址
  * @param    signed_clk ：有符号脉冲数，正数为正转，负数为反转
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(UART_HandleTypeDef *huart, uint8_t addr, int32_t signed_clk, uint16_t vel, uint8_t acc, uint8_t raF, uint8_t snF)
{
    uint8_t cmd[16] = {0};
    uint8_t dir;

    // 根据有符号脉冲数的正负确定方向
    if (signed_clk >= 0) {
        dir = 0; // 正转，对应原 dir 的 0 为 CW
    } else {
        dir = 1; // 反转，对应原 dir 除 0 外的值为 CCW
        signed_clk = -signed_clk; // 取绝对值用于后续脉冲数处理
    }

    // 装载命令
    cmd[0]  =  addr;                      // 地址
    cmd[1]  =  0xFD;                      // 功能码
    cmd[2]  =  dir;                       // 方向
    cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
    cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
    cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
    cmd[6]  =  (uint8_t)(signed_clk >> 24);      // 脉冲数(bit24 - bit31)
    cmd[7]  =  (uint8_t)(signed_clk >> 16);      // 脉冲数(bit16 - bit23)
    cmd[8]  =  (uint8_t)(signed_clk >> 8);       // 脉冲数(bit8  - bit15)
    cmd[9]  =  (uint8_t)(signed_clk >> 0);       // 脉冲数(bit0  - bit7 )
    cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
    cmd[12] =  0x6B;                      // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, 13);
}    

/**
    * @brief    读取系统参数
    * @param    huart: 串口句柄指针
    * @param    addr  ：电机地址
    * @param    s     ：系统参数类型
    * @retval   地址 + 功能码 + 命令状态 + 校验字节
    */
void Emm_V5_Read_Sys_Params(UART_HandleTypeDef *huart, uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[i] = addr; ++i;                   // 地址

    switch(s)                             // 功能码
    {
        case S_VER  : cmd[i] = 0x1F; ++i; break;
        case S_RL   : cmd[i] = 0x20; ++i; break;
        case S_PID  : cmd[i] = 0x21; ++i; break;
        case S_VBUS : cmd[i] = 0x24; ++i; break;
        case S_CPHA : cmd[i] = 0x27; ++i; break;
        case S_ENCL : cmd[i] = 0x31; ++i; break;
        case S_IMPL : cmd[i] = 0x32; ++i; break;
        case S_TPOS : cmd[i] = 0x33; ++i; break;
        case S_VEL  : cmd[i] = 0x35; ++i; break;
        case S_CPOS : cmd[i] = 0x36; ++i; break;
        case S_PERR : cmd[i] = 0x37; ++i; break;
        case S_FLAG : cmd[i] = 0x3A; ++i; break;
        case S_ORG  : cmd[i] = 0x3B; ++i; break;
        case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
        case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
        default: break;
    }

    cmd[i] = 0x6B; ++i;                   // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, i);
}

void Read_Pulse_Value(UART_HandleTypeDef *huart, uint8_t addr)
{
    SysParams_t s = S_IMPL;
    Emm_V5_Read_Sys_Params(huart, addr, s);
}

void Read_Degree_Value(UART_HandleTypeDef *huart, uint8_t addr)
{
    SysParams_t s = S_CPOS;
    Emm_V5_Read_Sys_Params(huart, addr, s);
}


/**
    * @brief    修改开环/闭环控制模式
    * @param    huart: 串口句柄指针
    * @param    addr     ：电机地址
    * @param    svF      ：是否存储标志，false为不存储，true为存储
    * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
    * @retval   地址 + 功能码 + 命令状态 + 校验字节
    */
void Emm_V5_Modify_Ctrl_Mode(UART_HandleTypeDef *huart, uint8_t addr, uint8_t svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0x46;                       // 功能码
    cmd[2] =  0x69;                       // 辅助码
    cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
    cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
    cmd[5] =  0x6B;                       // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, 6);
}

/**
    * @brief    使能信号控制
    * @param    huart: 串口句柄指针
    * @param    addr  ：电机地址
    * @param    state ：使能状态     ，true为使能电机，false为关闭电机
    * @param    snF   ：多机同步标志 ，false为不启用，true为启用
    * @retval   地址 + 功能码 + 命令状态 + 校验字节
    */
void Emm_V5_En_Control(UART_HandleTypeDef *huart, uint8_t addr, uint8_t state, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0xF3;                       // 功能码
    cmd[2] =  0xAB;                       // 辅助码
    cmd[3] =  (uint8_t)state;             // 使能状态
    cmd[4] =  snF;                        // 多机同步运动标志
    cmd[5] =  0x6B;                       // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, 6);
}

/**
    * @brief    立即停止（所有控制模式都通用）
    * @param    huart: 串口句柄指针
    * @param    addr  ：电机地址
    * @param    snF   ：多机同步标志，false为不启用，true为启用
    * @retval   地址 + 功能码 + 命令状态 + 校验字节
    */
void Emm_V5_Stop_Now(UART_HandleTypeDef *huart, uint8_t addr, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0xFE;                       // 功能码
    cmd[2] =  0x98;                       // 辅助码
    cmd[3] =  snF;                        // 多机同步运动标志
    cmd[4] =  0x6B;                       // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, 5);
}

/**
    * @brief    多机同步运动
    * @param    huart: 串口句柄指针
    * @param    addr  ：电机地址
    * @retval   地址 + 功能码 + 命令状态 + 校验字节
    */
void Emm_V5_Synchronous_motion(UART_HandleTypeDef *huart, uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0xFF;                       // 功能码
    cmd[2] =  0x66;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, 4);
}

/**
    * @brief    修改回零参数
    * @param    huart: 串口句柄指针
    * @param    addr  ：电机地址
    * @param    svF   ：是否存储标志，false为不存储，true为存储
    * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    * @param    o_dir  ：回零方向，0为CW，其余值为CCW
    * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
    * @param    o_tm   ：回零超时时间，单位：毫秒
    * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
    * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
    * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
    * @param    potF   ：上电自动触发回零，false为不使能，true为使能
    * @retval   地址 + 功能码 + 命令状态 + 校验字节
    */
void Emm_V5_Origin_Modify_Params(UART_HandleTypeDef *huart, uint8_t addr, uint8_t svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, uint8_t potF)
{
    uint8_t cmd[32] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0x4C;                       // 功能码
    cmd[2] =  0xAE;                       // 辅助码
    cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
    cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[5] =  o_dir;                      // 回零方向
    cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
    cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
    cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
    cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
    cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
    cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
    cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
    cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
    cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
    cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
    cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
    cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
    cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
    cmd[19] =  0x6B;                      // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, 20);
}

/**
    * @brief    触发回零
    * @param    huart: 串口句柄指针
    * @param    addr   ：电机地址
    * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    * @param    snF   ：多机同步标志，false为不启用，true为启用
    * @retval   地址 + 功能码 + 命令状态 + 校验字节
    */
void Emm_V5_Origin_Trigger_Return(UART_HandleTypeDef *huart, uint8_t addr, uint8_t o_mode, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0x9A;                       // 功能码
    cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
    cmd[4] =  0x6B;                       // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, 5);
}

/**
    * @brief    将当前位置清零
    * @param    huart: 串口句柄指针
    * @param    addr  ：电机地址
    * @retval   地址 + 功能码 + 命令状态 + 校验字节
    */
void Emm_V5_Reset_CurPos_To_Zero(UART_HandleTypeDef *huart, uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0x0A;                       // 功能码
    cmd[2] =  0x6D;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送命令
    uart_SendCmd(huart, cmd, 4);
}













