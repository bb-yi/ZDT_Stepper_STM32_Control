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
// 是否存储
#define STORE 1    // 存储标志
#define NO_STORE 0 // 不存储标
typedef struct
{
    uint16_t firmware_version;         // 固件版本
    uint16_t hardware_version;         // 硬件版本
    uint16_t phase_resistance;         // 相电阻 (mΩ)
    uint16_t phase_inductance;         // 相电感 (µH)
    uint16_t bus_voltage;              // 总线电压 (mV)
    uint16_t bus_average_current;      // 总线平均电流 (mA)
    uint16_t phase_current;            // 相电流 (A)
    uint16_t encoder_raw_value;        // 编码器原始值
    uint16_t encoder_calibrated_value; // 经过线性化校准后的编码器值
    float target_position;             // 电机目标位置 (°)
    float current_speed;               // 电机实时转速 (RPM)
    float current_position;            // 电机实时位置 (°)
    float position_error;              // 电机位置误差 (°)
    int8_t driver_temperature;         // 驱动器实时温度 (°C)
    uint8_t motor_status_flags;        // 电机状态标志位
    uint8_t motor_enabled;             // 电机使能状态
    uint8_t motor_position_reached;    // 电机位置到位标志位
    uint8_t motor_stall;               // 电机堵转标志位
    uint8_t motor_stall_protection;    // 电机编码器校准标志位
} ZDTStepperData;

void ZDT_Stepper_USRT_Init(void);
void ZDT_Stepper_USRT_RX_callback(uint16_t Size);
void ZDT_Stepper_Enable(uint8_t id, uint8_t enable, uint8_t sync_flag);
void ZDT_Stepper_Set_Speed(uint8_t id, uint8_t dir, uint16_t speed_rate, float speed_f, uint8_t sync_flag);                                                                                // 设置步进电机速度
void ZDT_Stepper_Set_position(uint8_t id, uint8_t dir, float speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag);                                                   // 设置步进电机位置
void ZDT_Stepper_Set_T_position(uint8_t id, uint8_t dir, uint16_t accel_accel, uint16_t decel_accel, float max_speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag); // 设置步进电机梯形加减速位置
void ZDT_Stepper_torque_control(uint8_t id, uint8_t dir, uint16_t current_rate, uint16_t current, uint8_t sync_flag);                                                                      // 力矩控制模式
void ZDT_Stepper_start_sync_motion(uint8_t id);                                                                                                                                            // 开启多机同步运动
void ZDT_Stepper_stop(uint8_t id, uint8_t sync_flag);                                                                                                                                      // 立即停止
void ZDT_Stepper_clear_current_position(uint8_t id);                                                                                                                                       // 角度清零
void ZDT_Stepper_trigger_zero_return(uint8_t id, uint8_t return_mode, uint8_t sync_flag);                                                                                                  // 触发回零

void ZDT_Stepper_release_stall_protection(uint8_t id);    // 解除堵转保护
void ZDT_Stepper_trigger_encoder_calibration(uint8_t id); // 触发编码器校准
void ZDT_Stepper_set_microstepping(uint8_t motor_id, uint8_t store_flag, uint8_t microstepping_value);

void ZDT_Stepper_Read_version(uint8_t id);
void ZDT_Stepper_Read_resistance_and_inductance(uint8_t id);
void ZDT_Stepper_Read_bus_voltage(uint8_t id);
void ZDT_Stepper_Read_bus_average_current(uint8_t id);
void ZDT_Stepper_Read_phase_current(uint8_t id);
void ZDT_Stepper_Read_encoder_raw_value(uint8_t id);
void ZDT_Stepper_Read_encoder_calibrated_value(uint8_t id);
void ZDT_Stepper_Read_target_position(uint8_t id);
void ZDT_Stepper_Read_current_speed(uint8_t id);
void ZDT_Stepper_Read_current_position(uint8_t id);
void ZDT_Stepper_Read_position_error(uint8_t id);
void ZDT_Stepper_Read_driver_temperature(uint8_t id);
void ZDT_Stepper_Read_motor_status_flags(uint8_t id);
void ZDT_Stepper_init(void);
#endif