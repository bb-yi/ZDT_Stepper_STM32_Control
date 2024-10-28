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

void ZDT_Stepper_USRT_RX_callback(uint16_t Size)
{
    // HAL_UART_Transmit(&huart1, USART2_Rx_data, Size, HAL_MAX_DELAY);
    // printf("\r\n");
    print_hex_array(USART2_Rx_data);
    ZDT_Stepper_USRT_Init();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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

    HAL_UART_Transmit(&Stepper_Uart_Handle, command, sizeof(command), HAL_MAX_DELAY);
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

    HAL_UART_Transmit(&Stepper_Uart_Handle, command, sizeof(command), HAL_MAX_DELAY);
}

/**
 * @brief 设置步进电机位置
 *
 * @param id 地址
 * @param dir 方向 CW 反转 顺时针 CCW 正转 逆时针
 * @param speed_f 速度
 * @param position_angle_f 位置角度
 * @param position_mode 模式 REL_POS_MODE 相对位置模式 ABS_POS_MODE 绝对位置模式
 * @param sync_flag 多机同步标志位  SYNC_ENABLE 同步  SYNC_DISABLE 不同步
 */
void set_motor_position(uint8_t id, uint8_t dir, float speed_f, float position_angle_f, uint8_t position_mode, uint8_t sync_flag)
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

    HAL_UART_Transmit(&Stepper_Uart_Handle, command, sizeof(command), HAL_MAX_DELAY);
}
void set_motor_T_position(uint8_t id, uint8_t sign, uint16_t accel_accel, uint16_t decel_accel, uint16_t max_speed, int32_t position_angle, uint8_t position_mode, uint8_t sync_flag)
{
    // 命令格式
    uint8_t command[16];
    command[0] = id;                        // 地址
    command[1] = 0xFD;                      // 命令头
    command[2] = sign;                      // 符号 (01: 负位置, 00: 正位置)
    command[3] = (accel_accel >> 8) & 0xFF; // 加速加速度高位
    command[4] = accel_accel & 0xFF;        // 加速加速度低位
    command[5] = (decel_accel >> 8) & 0xFF; // 减速加速度高位
    command[6] = decel_accel & 0xFF;        // 减速加速度低位
    command[7] = (max_speed >> 8) & 0xFF;   // 最大速度高位
    command[8] = max_speed & 0xFF;          // 最大速度低位
    command[9] = (position >> 24) & 0xFF;   // 位置角度高位
    command[10] = (position >> 16) & 0xFF;  // 位置角度次高位
    command[11] = (position >> 8) & 0xFF;   // 位置角度次低位
    command[12] = position & 0xFF;          // 位置角度低位
    command[13] = position_mode;            // 相对/绝对位置标志
    command[14] = sync_flag;                // 多机同步标志位

    // 计算校验字节
    uint8_t checksum = calculate_checksum(command, 15);

    // 发送数据到串口2
    send_data_via_serial2(command, 15);
    send_data_via_serial2(&checksum, 1);
}
void ZDT_Stepper_init(void)
{
    ZDT_Stepper_USRT_Init();
    ZDT_Stepper_Enable(0, Enable, SYNC_DISABLE);
}