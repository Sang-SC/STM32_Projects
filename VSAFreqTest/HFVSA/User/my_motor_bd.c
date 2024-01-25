/**
 * @file my_motor_bd.c
 * @author your name (you@domain.com)
 * @brief 接口文件
 * @version 0.1
 * @date 2024-01-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "my_motor_bd.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern volatile uint8_t timer4_flag;
extern float AS5048_angle;

int32_t motor_angle_int32 = 0; // 当前电机角度
float motor_angle_float = 0;   // 当前电机角度，换算关系式为：motor_angle_float = motor_angle_int32 / 24 / 100，其中 24 代表减速比


void motorEnable(void)
{
	uint8_t motor_enable_request[36] = {0xAA, 0x01, 0x77, 0x00, 0x01, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0xFF, 0xFF, 0x0D, 0x0A};  // 请求帧
	uint8_t motor_enable_response[40]; // 响应帧

	HAL_UART_Transmit(&huart3, motor_enable_request, 36, 2);
	HAL_UART_Receive(&huart3, motor_enable_response, 40, 2);
	if (motor_enable_response[0] == 0xAA)
	{
		user_spdlog_trace("motorEnable");
	}
}

void motorDisable(void)
{
	uint8_t motor_disable_request[36] = {0xAA, 0x01, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0xFF, 0xFF, 0x0D, 0x0A};  // 请求帧
	uint8_t motor_disable_response[40]; // 响应帧

	HAL_UART_Transmit(&huart3, motor_disable_request, 36, 2);
	HAL_UART_Receive(&huart3, motor_disable_response, 40, 2);
	if (motor_disable_response[0] == 0xAA)
	{
		user_spdlog_trace("motorDisable");
	}
}

void positionControl(float angle_des)
{
	static uint8_t position_control_request[36] = {0xAA, 0x01, 0x77, 0x01, 0x00, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
										0xFF, 0xFF, 0x0D, 0x0A};  // 请求帧
	uint8_t position_control_response[40]; // 响应帧

	if (angle_des >= 10 || angle_des <= -10)
	{
		user_spdlog_error("angle_des error!");
		return;
	}

	int32_t angle_des_int32 = (int32_t)(angle_des * 100 * 24);
	position_control_request[4] = angle_des_int32;
	position_control_request[5] = angle_des_int32 >> 8;
	position_control_request[6] = angle_des_int32 >> 16;
	position_control_request[7] = angle_des_int32 >> 24;

	// user_spdlog_trace("position_control_request[4] = %x", position_control_request[4]);
	// user_spdlog_trace("position_control_request[5] = %x", position_control_request[5]);
	// user_spdlog_trace("position_control_request[6] = %x", position_control_request[6]);
	// user_spdlog_trace("position_control_request[7] = %x", position_control_request[7]);

	HAL_UART_Transmit(&huart3, position_control_request, 36, 100);

	HAL_UART_Receive(&huart3, position_control_response, 40, 100);

	motor_angle_int32 = position_control_response[20] | (position_control_response[21] << 8) | (position_control_response[22] << 16) | (position_control_response[23] << 24);
	motor_angle_float = motor_angle_int32 / 24.0 / 100.0;

	// if (position_control_response[0] == 0xAA)
	// {
	// 	UART_Draw_Float(motor_angle_float);
	// } else {
	// 	motorDisable();
	// }
	

	// user_spdlog_trace("position_control_request[20] = %x", position_control_request[20]);
	// user_spdlog_trace("position_control_request[21] = %x", position_control_request[21]);
	// user_spdlog_trace("position_control_request[22] = %x", position_control_request[22]);
	// user_spdlog_trace("position_control_request[23] = %x", position_control_request[23]);

	// user_spdlog_debug("motor_angle_float = %.3f", motor_angle_float);
}


/**
 * @brief 正弦波测试函数，位置环控制
 *
 */
void Sine_Wave_Test(float amplitude) {
    double PI = 3.1415926;
    float angle_des;

	static double frequency = 0.5;
	static double t = 0;
    // 正弦波
    while (1) {
		if (timer4_flag)
		{

			timer4_flag = 0;
			
			if (frequency < 5)
			{
				frequency += 0.0005;
			}
			
			t += 0.002;
			// user_spdlog_debug("t = %f", t);
		
			angle_des = amplitude * sin(2.0 * PI * t * frequency);
			positionControl(angle_des);
//			UART_Draw_Trajectory(angle_des, motor_angle_float, - AS5048_angle);
			UART_Draw_Trajectory2(angle_des, motor_angle_float, - AS5048_angle, frequency);
			

		}
    }
}


/**
 * @brief 串口绘图，绘制一个浮点数
 * 
 * @param float_data 
 */
void UART_Draw_Float(float float_data) {
    uint8_t UARTTxDataBuffer[6] = {0xAA, 0X55};
    for (uint8_t i = 0; i < 4; i++) {
        UARTTxDataBuffer[i + 2] = *((char *)(&float_data) + i);
    }
    HAL_UART_Transmit(&huart1, UARTTxDataBuffer, 6, 1000);
}

/**
 * @brief 串口绘图，绘制三个浮点数
 * 
 * @param float_data1 
 * @param float_data2 
 * @param float_data3 
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