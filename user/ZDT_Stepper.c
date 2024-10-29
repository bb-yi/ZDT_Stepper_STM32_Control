#include "ZDT_Stepper.h"

extern DMA_HandleTypeDef hdma_usart2_rx;
uint8_t USART2_Rx_data[50] = {0};

void print_hex_array(const uint8_t *data)
{
    for (size_t i = 0; i < sizeof(data); i++)
    {
        printf("%02X ", data[i]); // 每个字节打印为两位十六进制
    }
    printf("\n");
}

void ZDT_Stepper_USRT_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&Stepper_Uart_Handle, USART2_Rx_data, sizeof(USART2_Rx_data));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}
ZDTStepperData stepperdata;
// 根据指令标识赋值的函数
void assign_message_based_on_command(uint8_t command_id, char *message, size_t message_size)
{
    // 清空字符串
    memset(message, 0, message_size);

    // 根据指令标识赋值
    switch (command_id)
    {
    case 0xF3:
        snprintf(message, message_size, "电机使能");
        break;
    case 0xF5:
        snprintf(message, message_size, "力矩控制");
        break;
    case 0xF6:
        snprintf(message, message_size, "速度控制");
        break;
    case 0xFB:
        snprintf(message, message_size, "直通位置控制");
        break;
    case 0xFD:
        snprintf(message, message_size, "梯形位置控制");
        break;
    case 0xFE:
        snprintf(message, message_size, "立即停止");
        break;
    case 0xFF:
        snprintf(message, message_size, "多机同步运动");
        break;
    case 0x0A:
        snprintf(message, message_size, "角度清零");
        break;
    case 0x0E:
        snprintf(message, message_size, "解除堵转保护");
        break;
    case 0x06:
        snprintf(message, message_size, "触发编码器校准");
        break;
    case 0x84:
        snprintf(message, message_size, "修改细分");
        break;
    default:
        snprintf(message, message_size, "未知的指令");
        break;
    }
}
void receive_motor_status(uint8_t *data, uint16_t size)
{
    char message[50];       // 字符串缓冲区
    char statu_message[13]; // 状态字符串缓冲区
    if (data == NULL)
        return; // 检查输入是否为空
    if (size == 4 && data[1] != 0x3A)
    {
        uint8_t motor_id = data[0];   // 电机地址
        uint8_t command_id = data[1]; // 指令标识
        uint8_t status = data[2];     // 执行状态
        uint8_t checksum = data[3];   // 校验字节
        if (checksum != 0x6B)
        {
            printf("校验失败!\n");
            return; // 校验失败，返回
        }
        snprintf(statu_message, sizeof(statu_message), (status == 0x02) ? "执行成功" : "执行失败");
        assign_message_based_on_command(command_id, message, sizeof(message));
        // printf("电机 %d:%s:%s\n", motor_id, message, statu_message);
        // 这里可以根据需要进一步处理接收到的指令
    }
    else
    {
        uint8_t motor_id = data[0];        // 电机地址
        uint8_t command_id = data[1];      // 指令标识
        uint8_t checksum = data[size - 1]; // 校验字节
        uint32_t temp;
        if (checksum != 0x6B)
        {
            printf("校验失败!\n");
            return; // 校验失败，返回
        }

        // 清空消息
        snprintf(message, sizeof(message), "电机 %d: ", motor_id);

        switch (command_id)
        {
        case 0x1F:
            stepperdata.firmware_version = (data[2] << 8) | data[3]; // 固件版本
            stepperdata.hardware_version = (data[4] << 8) | data[5]; // 硬件版本
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "固件版本: %d, 硬件版本: %d", stepperdata.firmware_version, stepperdata.hardware_version);
            break;

        case 0x20:
            stepperdata.phase_resistance = (data[2] << 8) | data[3]; // 相电阻
            stepperdata.phase_inductance = (data[4] << 8) | data[5]; // 相电感
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "相电阻: %d mΩ, 相电感: %d µH", stepperdata.phase_resistance, stepperdata.phase_inductance);
            break;

        case 0x24:
            stepperdata.bus_voltage = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "总线电压: %d mV", stepperdata.bus_voltage);
            break;

        case 0x26:
            stepperdata.bus_average_current = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "总线平均电流: %d mA", stepperdata.bus_average_current);
            break;

        case 0x27:
            stepperdata.phase_current = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "相电流: %d mA", stepperdata.phase_current);
            break;

        case 0x29:
            stepperdata.encoder_raw_value = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "编码器原始值: %d", stepperdata.encoder_raw_value);
            break;

        case 0x31:
            stepperdata.encoder_calibrated_value = (data[2] << 8) | data[3];
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "经过线性化校准后的编码器值: %d", stepperdata.encoder_calibrated_value);
            break;

        case 0x33:
            temp = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 组合四个字节
            stepperdata.target_position = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "目标位置: %.1f °", stepperdata.target_position);
            break;

        case 0x35:
            temp = (data[3] << 8) | data[4]; // 组合两个字节
            stepperdata.current_speed = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "实时转速: %.1f RPM", stepperdata.current_speed);
            break;

        case 0x36:
            temp = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 组合四个字节
            stepperdata.current_position = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "实时位置: %.1f °", stepperdata.current_position);
            break;

        case 0x37:
            temp = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 组合四个字节
            stepperdata.position_error = (data[2] == 0x01) ? -((float)temp) / 10.0f : ((float)temp) / 10.0f;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "位置误差: %.1f °", stepperdata.position_error);
            break;

        case 0x39:
            stepperdata.driver_temperature = (data[2] == 0x01) ? -((int8_t)data[3]) : ((int8_t)data[3]);
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "驱动器实时温度: %.d °C", stepperdata.driver_temperature);
            break;

        case 0x3A:
            stepperdata.motor_status_flags = data[2];
            stepperdata.motor_enabled = (data[2] & 0x01);
            stepperdata.motor_position_reached = (data[2] & 0x02) >> 1;
            stepperdata.motor_stall = (data[2] & 0x04) >> 2;
            stepperdata.motor_stall_protection = (data[2] & 0x08) >> 3;
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "使能 %d, 位置到位 %d, 电机堵转 %d, 堵转保护 %d", stepperdata.motor_enabled, stepperdata.motor_position_reached, stepperdata.motor_stall, stepperdata.motor_stall_protection);
            break;

        default:
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "未知命令");
            break;
        }

        // 最后统一打印
        printf("%s\n", message);
    }
}
void ZDT_Stepper_USRT_RX_callback(uint16_t Size)
{
    // HAL_UART_Transmit(&huart1, USART2_Rx_data, Size, HAL_MAX_DELAY);
    // printf("\r\n");
    // print_hex_array(USART2_Rx_data);
    ZDT_Stepper_USRT_Init();
    receive_motor_status(USART2_Rx_data, Size);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void ZDT_Stepper_UART_Send_Data(uint8_t *data, size_t length)
{
    HAL_UART_Transmit(&Stepper_Uart_Handle, data, length, HAL_MAX_DELAY);
}

void ZDT_Stepper_Enable(uint8_t id, uint8_t enable, uint8_t sync_flag)
{
    uint8_t command[6];
    command[0] = id;        // 地址
    command[1] = 0xF3;      // 命令头
    command[2] = 0xAB;      // 命令码
    command[3] = enable;    // 使能状态 (1: 使能, 0: 不使能)
    command[4] = sync_flag; // 多机同步标志位 (0: 不同步, 1: 同步)
    command[5] = 0x6B;      // 校验字节

    ZDT_Stepper_UART_Send_Data(command, 6);
}

/**
 * @brief 设置步进电机速度
 *
 * @param id 地址
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param speed_rate 加速度 单位 RPM/s
 * @param speed 速度 单位 RPM
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_Set_Speed(uint8_t id, uint8_t dir, uint16_t speed_rate, float speed_f, uint8_t sync_flag)
{
    uint16_t speed = (uint16_t)(speed_f * 10); // 转换为两个字节的数据
    // 命令格式
    uint8_t command[9];
    command[0] = id;                       // 地址
    command[1] = 0xF6;                     // 命令头
    command[2] = dir;                      // 符号 (01: 负转速, 00: 正转速)
    command[3] = (speed_rate >> 8) & 0xFF; // 速度斜率高位
    command[4] = speed_rate & 0xFF;        // 速度斜率低位
    command[5] = (speed >> 8) & 0xFF;      // 速度高位
    command[6] = speed & 0xFF;             // 速度低位
    command[7] = sync_flag;                // 多机同步标志位
    command[8] = 0x6B;                     // 校验字节

    ZDT_Stepper_UART_Send_Data(command, 9);
}

/**
 * @brief 设置步进电机位置
 *
 * @param id 地址
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param speed_f 速度  单位 RPM
 * @param position_angle_f 位置角度 单位 度
 * @param position_mode 模式 REL_POS_MODE 相对位置模式 ABS_POS_MODE 绝对位置模式
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_Set_position(uint8_t id, uint8_t dir, float speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag)
{
    // 转换速度为两个字节
    uint16_t speed = (uint16_t)(speed_f * 10); // 速度乘以 10

    // 转换位置角度为四个字节
    uint32_t position_angle = (uint32_t)(position_angle_f * 10); // 直接使用 int32_t 进行转换

    // 命令格式
    uint8_t command[12];
    command[0] = id;                            // 地址
    command[1] = 0xFB;                          // 命令头
    command[2] = dir;                           // 符号 (01: 负位置, 00: 正位置)
    command[3] = (speed >> 8) & 0xFF;           // 速度高位
    command[4] = speed & 0xFF;                  // 速度低位
    command[5] = (position_angle >> 24) & 0xFF; // 位置角度高位
    command[6] = (position_angle >> 16) & 0xFF; // 位置角度次高位
    command[7] = (position_angle >> 8) & 0xFF;  // 位置角度次低位
    command[8] = position_angle & 0xFF;         // 位置角度低位
    command[9] = position_mode;                 // 相对/绝对位置标志
    command[10] = sync_flag;                    // 多机同步标志位
    command[11] = 0x6B;                         // 校验字节

    ZDT_Stepper_UART_Send_Data(command, 12);
}
/**
 * @brief 设置步进电机梯形加减速位置
 *
 * @param id 地址
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param accel_accel 加速度 单位 RPM/s
 * @param decel_accel 减速度 单位 RPM/s
 * @param max_speed_f 最大速度 单位 RPM
 * @param position_angle_f 目标位置 单位 度
 * @param position_mode 位置模式 REL_POS_MODE 相对位置模式 ABS_POS_MODE 绝对位置模式
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_Set_T_position(uint8_t id, uint8_t dir, uint16_t accel_accel, uint16_t decel_accel, float max_speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag)
{
    // 转换速度为两个字节
    uint16_t max_speed = (uint16_t)(max_speed_f * 10); // 速度乘以 10

    // 转换位置角度为四个字节
    uint32_t position_angle = (uint32_t)(position_angle_f * 10); // 直接使用 int32_t 进行转换
    // 命令格式
    uint8_t command[16];
    command[0] = id;                             // 地址
    command[1] = 0xFD;                           // 命令头
    command[2] = dir;                            // 符号 (01: 负位置, 00: 正位置)
    command[3] = (accel_accel >> 8) & 0xFF;      // 加速加速度高位
    command[4] = accel_accel & 0xFF;             // 加速加速度低位
    command[5] = (decel_accel >> 8) & 0xFF;      // 减速加速度高位
    command[6] = decel_accel & 0xFF;             // 减速加速度低位
    command[7] = (max_speed >> 8) & 0xFF;        // 最大速度高位
    command[8] = max_speed & 0xFF;               // 最大速度低位
    command[9] = (position_angle >> 24) & 0xFF;  // 位置角度高位
    command[10] = (position_angle >> 16) & 0xFF; // 位置角度次高位
    command[11] = (position_angle >> 8) & 0xFF;  // 位置角度次低位
    command[12] = position_angle & 0xFF;         // 位置角度低位
    command[13] = position_mode;                 // 相对/绝对位置标志
    command[14] = sync_flag;                     // 多机同步标志位
    command[15] = 0x6B;                          // 校验字节

    ZDT_Stepper_UART_Send_Data(command, 16);
}
/**
 * @brief 力矩控制模式
 *
 * @param id 地址
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param current_rate 电流加速度 单位 mA/s
 * @param current 目标电流 单位 mA
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_torque_control(uint8_t id, uint8_t dir, uint16_t current_rate, uint16_t current, uint8_t sync_flag)
{
    // 命令格式
    uint8_t command[9];
    command[0] = id;                         // 地址
    command[1] = 0xF5;                       // 命令头
    command[2] = dir;                        // 符号 (00: 正电流, 01: 负电流)
    command[3] = (current_rate >> 8) & 0xFF; // 电流斜率高位
    command[4] = current_rate & 0xFF;        // 电流斜率低位
    command[5] = (current >> 8) & 0xFF;      // 电流高位
    command[6] = current & 0xFF;             // 电流低位
    command[7] = sync_flag;                  // 多机同步标志位
    command[8] = 0x6B;                       // 校验字节

    ZDT_Stepper_UART_Send_Data(command, 9);
}

/**
 * @brief 开启多机同步运动
 *
 * @param id 地址 多机同步用00
 */
void ZDT_Stepper_start_sync_motion(uint8_t id)
{
    // 命令格式
    uint8_t command[4];
    command[0] = id;   // 地址
    command[1] = 0xFF; // 命令头
    command[2] = 0x66; // 多机同步运动命令
    command[3] = 0x6B; // 校验字节

    ZDT_Stepper_UART_Send_Data(command, 4);
}
/**
 * @brief 立即停止
 *
 * @param id 地址
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_stop(uint8_t id, uint8_t sync_flag)
{
    // 命令格式
    uint8_t command[5];
    command[0] = id;        // 地址
    command[1] = 0xFE;      // 命令头
    command[2] = 0x98;      // 立即停止命令
    command[3] = sync_flag; // 多机同步标志位
    command[4] = 0x6B;      // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 5);
}
/**
 * @brief 角度清零
 *
 * @param id 地址
 */
void ZDT_Stepper_clear_current_position(uint8_t id)
{
    // 命令格式
    uint8_t command[4];
    command[0] = id;   // 地址
    command[1] = 0x0A; // 命令头
    command[2] = 0x6D; // 固定字节
    command[3] = 0x6B; // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 4);
}
/**
 * @brief 触发回零
 *
 * @param id 地址
 * @param return_mode 回零模式
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void ZDT_Stepper_trigger_zero_return(uint8_t id, uint8_t return_mode, uint8_t sync_flag)
{
    // 命令格式
    uint8_t command[5];
    command[0] = id;          // 地址
    command[1] = 0x9A;        // 命令头
    command[2] = return_mode; // 回零模式
    command[3] = sync_flag;   // 多机同步标志
    command[3] = 0x6B;        // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 5);
}
/**
 * @brief 解除堵转保护
 *
 * @param id 地址
 */
void ZDT_Stepper_release_stall_protection(uint8_t id)
{
    // 命令格式
    uint8_t command[4];
    command[0] = id;   // 地址
    command[1] = 0x0E; // 命令头
    command[2] = 0x52; // 固定字节
    command[3] = 0x6B; // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 4);
}
/**
 * @brief 触发编码器校准
 *
 * @param id 地址
 */
void ZDT_Stepper_trigger_encoder_calibration(uint8_t id)
{
    // 命令格式
    uint8_t command[4];
    command[0] = id;   // 地址
    command[1] = 0x06; // 命令头
    command[2] = 0x45; // 固定字节
    command[3] = 0x6B; // 校验字节

    ZDT_Stepper_UART_Send_Data(command, 4);
}
void ZDT_Stepper_set_microstepping(uint8_t motor_id, uint8_t store_flag, uint8_t microstepping_value)
{
    // 确保细分值在合理范围内（例如，0-255）
    if (microstepping_value > 0xFF)
    {
        // 处理超出范围的情况
        return;
    }

    // 命令格式
    uint8_t command[6];
    command[0] = motor_id;            // 电机地址
    command[1] = 0x84;                // 命令标识
    command[2] = 0x8A;                // 子命令标识
    command[3] = store_flag;          // 是否存储标志
    command[4] = microstepping_value; // 细分值
    command[5] = 0x6B;                // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 6);
}

/**
 * @brief 读取固件和硬件版本
 *
 * @param id 地址
 */
void ZDT_Stepper_Read_version(uint8_t id)
{
    // 命令格式
    uint8_t command[3];
    command[0] = id;   // 地址
    command[1] = 0x1F; // 命令头
    command[2] = 0x6B; // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
/**
 * @brief 读取相电阻和相电感
 *
 * @param id 地址
 */
void ZDT_Stepper_Read_resistance_and_inductance(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x20;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_bus_voltage(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x24;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_bus_average_current(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x26;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_phase_current(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x27;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
/**
 * @brief 读取编码器原始值
 * 编码器原始值一圈的数值是 0-16383，即 0-16383 表示 0-360°
 * @param id 地址
 */
void ZDT_Stepper_Read_encoder_raw_value(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x29;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
/**
 * @brief 经过线性化校准后的编码器值
 * 经过线性化校准后，内部对编码器值进行了 4 倍频，一圈的数值是 0-65535
 * @param id 地址
 */
void ZDT_Stepper_Read_encoder_calibrated_value(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x31;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_target_position(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x33;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_current_speed(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x35;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_current_position(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x36;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_position_error(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x37;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_driver_temperature(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x39;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_Read_motor_status_flags(uint8_t id)
{
    uint8_t command[3]; // 包含地址、命令和校验字节
    command[0] = id;    // 电机地址
    command[1] = 0x3A;  // 命令头
    command[2] = 0x6B;  // 校验字节
    ZDT_Stepper_UART_Send_Data(command, 3);
}
void ZDT_Stepper_init(void)
{
    ZDT_Stepper_USRT_Init();
    ZDT_Stepper_Enable(0, Enable, SYNC_DISABLE);
}