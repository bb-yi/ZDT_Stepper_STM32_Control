#ifndef _ZDT_STEPPER_H_
#define _ZDT_STEPPER_H_
#include "usart.h"
#define Stepper_Uart_Handle huart2
// 使能
#define Enable 1
#define Disable 0
// 舵机同步
#define SYNC_ENABLE 1  // 多机同步
#define SYNC_DISABLE 0 // 不使能多机同步
// 方向
#define CW 1  // 负转速 顺时针，正对转轴看
#define CCW 0 // 正转速 逆时针
// 位置模式
#define REL_POS_MODE 0 // 相对位置模式
#define ABS_POS_MODE 1 // 绝对位置模式

void ZDT_Stepper_USRT_Init(void);
void ZDT_Stepper_USRT_RX_callback(uint16_t Size);
void ZDT_Stepper_Enable(uint8_t id, uint8_t enable, uint8_t sync_flag);
void ZDT_Stepper_Set_Speed(uint8_t id, uint8_t dir, uint16_t speed_rate, float speed_f, uint8_t sync_flag);
void set_motor_position(uint8_t id, uint8_t dir, float speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag);
void ZDT_Stepper_init(void);
#endif