#ifndef __MY_MOTOR_H
#define __MY_MOTOR_H

#include <math.h>

#include "can.h"
#include "my_can.h"
#include "my_spdlog.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

void Process_Motor_Data(void);
void readMultiTurnAngle(void);
void multiTurnPositionControl2(float angle_des, uint16_t max_speed);
void Read_Angle_Test(void);
void Square_Wave_Test(float amplitude, uint16_t max_speed);
void Sine_Wave_Test(float amplitude, uint16_t max_speed);
void UART_Draw_Trajectory(float float_data1, float float_data2, float float_data3);
void UART_Draw_Trajectory2(float float_data1, float float_data2, float float_data3, float float_data4);

#endif
