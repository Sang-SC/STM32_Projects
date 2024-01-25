#include "my_motor.h"

extern uint8_t CANRxDataBuffer[8];  // CAN接收数据缓存
extern float AS5048_angle;
extern volatile uint8_t timer4_flag;

uint8_t read_multi_turn_angle_flag = 0;        // 读取电机角度标志位
uint8_t multi_turn_position_control_flag = 0;  // 控制电机角度标志位
int64_t motor_angle_int64 = 0;                 // 当前电机角度
float motor_angle_float = 0;                   // 当前电机角度，换算关系式为：motor_angle_float = motor_angle_int64 / 6 / 100，其中 6 代表减速比
float joint_angle_float = 0;                   // 当前关节角度，motor_angle_float 基础上，加上编码器角度

/**
 * @brief 定义中断函数中处理电机数据帧的函数
 *
 */
void Process_Motor_Data(void) {
    // 根据电机响应帧的命令字节做不同处理
    switch (CANRxDataBuffer[0]) {
        case 0x92:  // readMultiTurnAngle 响应帧的命令字节
            motor_angle_int64 = CANRxDataBuffer[1] | (CANRxDataBuffer[2] << 8) | CANRxDataBuffer[3] << 16 | (CANRxDataBuffer[4] << 24) | (CANRxDataBuffer[5] << 32) | (CANRxDataBuffer[6] << 40) |
                                (CANRxDataBuffer[7] << 48);  // 更新角度
            read_multi_turn_angle_flag = 1;                  // 读取电机角度标志位置1
            motor_angle_float = (float)motor_angle_int64 / 6 / 100;
            joint_angle_float = motor_angle_float + AS5048_angle;
            break;
        case 0xA4:  // multiTurnPositionControl 响应帧的命令字节
            multi_turn_position_control_flag = 1;
        default:
            break;
    }

    // 清空接收缓冲区
    for (int i = 0; i < 8; i++) {
        CANRxDataBuffer[i] = 0;
    }
}

/**
 * @brief 读取多圈角度命令
 *
 */
void readMultiTurnAngle(void) {
    static uint8_t read_multi_turn_angle_frame[8] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN_Send_Data(&hcan1, 0x141, read_multi_turn_angle_frame, 8);
    while (read_multi_turn_angle_flag != 1) {
    }
    read_multi_turn_angle_flag = 0;
}

/**
 * @brief 多圈位置闭环控制命令 2
 *
 * @param angle_des
 * @param max_speed
 */
void multiTurnPositionControl2(float angle_des, uint16_t max_speed) {
    static uint8_t multi_turn_position_control_frame[8] = {0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (max_speed < 1 || max_speed > 1500) {
        user_spdlog_error("max_speed should be in range [1, 1000]");
        return;
    }

    int32_t angle_des_int32 = (int32_t)(angle_des * 6 * 100);
    multi_turn_position_control_frame[2] = max_speed;
    multi_turn_position_control_frame[3] = max_speed >> 8;
    multi_turn_position_control_frame[4] = angle_des_int32;
    multi_turn_position_control_frame[5] = angle_des_int32 >> 8;
    multi_turn_position_control_frame[6] = angle_des_int32 >> 16;
    multi_turn_position_control_frame[7] = angle_des_int32 >> 24;

    CAN_Send_Data(&hcan1, 0x141, multi_turn_position_control_frame, 8);
    while (multi_turn_position_control_flag != 1) {
    }
    multi_turn_position_control_flag = 0;

    // ?更新角度（可删）
    readMultiTurnAngle();
}

/**
 * @brief 读取角度测试函数
 *
 */
void Read_Angle_Test(void) {
    while (1) {
        readMultiTurnAngle();
        user_spdlog_info("motor_angle_int64 = %lld", motor_angle_int64);
        user_spdlog_info("motor_angle_float = %f", motor_angle_float);
        user_spdlog_info("joint_angle_float = %f", joint_angle_float);
        HAL_Delay(200);
    }
}

/**
 * @brief 方波测试函数，位置环控制
 *
 * @param amplitude
 * @param max_speed
 */
void Square_Wave_Test(float amplitude, uint16_t max_speed) {
    // 轨迹点个数
    int point_num = 200;

    // 计算每个方向的步进角度
    float step_angle = amplitude / point_num;

    // 从零点开始逆时针方向的方波
    for (int i = point_num / 2; i < point_num; i++) {
        float angle_des = amplitude - 2 * i * step_angle;
        multiTurnPositionControl2(angle_des, max_speed);
        UART_Draw_Trajectory(angle_des, motor_angle_float, joint_angle_float);
        HAL_Delay(1000 / point_num);
    }

    while (1) {
        // 顺时针方向的方波
        for (int i = 0; i < point_num; i++) {
            float angle_des = -amplitude + 2 * i * step_angle;
            multiTurnPositionControl2(angle_des, max_speed);
            UART_Draw_Trajectory(angle_des, motor_angle_float, joint_angle_float);
            HAL_Delay(1000 / point_num);
        }

        // 逆时针方向的方波
        for (int i = 0; i < point_num; i++) {
            float angle_des = amplitude - 2 * i * step_angle;
            multiTurnPositionControl2(angle_des, max_speed);
            UART_Draw_Trajectory(angle_des, motor_angle_float, joint_angle_float);
            HAL_Delay(1000 / point_num);
        }
    }
}

/**
 * @brief 正弦波测试函数，位置环控制
 *
 */
void Sine_Wave_Test(float amplitude, uint16_t max_speed) {
    double PI = 3.1415926;
    float angle_des;

    static double frequency = 0.5;
    static double t = 0;
    // 正弦波
    while (1) {
        if (timer4_flag) {
            timer4_flag = 0;
            t += 0.002;
            // user_spdlog_debug("t = %f", t);

            angle_des = amplitude * sin(2.0 * PI * t * frequency);
            multiTurnPositionControl2(angle_des, max_speed);
            // UART_Draw_Float(angle_des);
            // UART_Draw_Trajectory(angle_des, motor_angle_float, joint_angle_float);              // 串口绘图，绘制期望轨迹和实际轨迹
            UART_Draw_Trajectory2(angle_des, motor_angle_float, joint_angle_float, frequency);  // 串口绘图，绘制期望轨迹和实际轨迹
            // user_spdlog_debug("AS5048_angle = %f", AS5048_angle);

            if (frequency < 5) {
                frequency += 0.0005;
            }
        }
    }
}

/**
 * @brief 串口绘图，绘制期望轨迹和实际轨迹
 *
 * @param float_data1 期望轨迹
 * @param float_data2 实际轨迹
 */
void UART_Draw_Trajectory(float float_data1, float float_data2, float float_data3) {
    // user_spdlog_debug("float_data1 = %f, float_data2 = %f", float_data1, float_data2);
    uint8_t UARTTxDataBuffer[14] = {0xAA, 0X55};
    if (float_data1 > 180) {
        float_data1 -= 360;
    }
    if (float_data2 > 180) {
        float_data2 -= 360;
    }
    if (float_data3 > 180) {
        float_data3 -= 360;
    }
    for (uint8_t i = 0; i < 4; i++) {
        UARTTxDataBuffer[i + 2] = *((char *)(&float_data1) + i);
    }
    for (uint8_t i = 0; i < 4; i++) {
        UARTTxDataBuffer[i + 6] = *((char *)(&float_data2) + i);
    }
    for (uint8_t i = 0; i < 4; i++) {
        UARTTxDataBuffer[i + 10] = *((char *)(&float_data3) + i);
    }
    HAL_UART_Transmit(&huart1, UARTTxDataBuffer, 14, 1000);
}

/**
 * @brief 串口绘图，绘制四个浮点数
 *
 * @param float_data1
 * @param float_data2
 * @param float_data3
 * @param float_data4
 */
void UART_Draw_Trajectory2(float float_data1, float float_data2, float float_data3, float float_data4) {
    // user_spdlog_debug("float_data1 = %f, float_data2 = %f", float_data1, float_data2);
    uint8_t UARTTxDataBuffer[18] = {0xAA, 0X55};
    if (float_data1 > 180) {
        float_data1 -= 360;
    }
    if (float_data2 > 180) {
        float_data2 -= 360;
    }
    if (float_data3 > 180) {
        float_data3 -= 360;
    }
    if (float_data4 > 180) {
        float_data4 -= 360;
    }

    for (uint8_t i = 0; i < 4; i++) {
        UARTTxDataBuffer[i + 2] = *((char *)(&float_data1) + i);
    }
    for (uint8_t i = 0; i < 4; i++) {
        UARTTxDataBuffer[i + 6] = *((char *)(&float_data2) + i);
    }
    for (uint8_t i = 0; i < 4; i++) {
        UARTTxDataBuffer[i + 10] = *((char *)(&float_data3) + i);
    }
    for (uint8_t i = 0; i < 4; i++) {
        UARTTxDataBuffer[i + 14] = *((char *)(&float_data4) + i);
    }
    HAL_UART_Transmit(&huart1, UARTTxDataBuffer, 18, 1000);
}