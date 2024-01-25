#ifndef __MY_MOTOR_BD_H
#define __MY_MOTOR_BD_H

#include <math.h>

#include "my_spdlog.h"
#include "stm32f4xx_hal.h"
#include "usart.h"


void motorEnable(void);
void motorDisable(void);
void positionControl(float angle_des);
void UART_Draw_Trajectory(float float_data1, float float_data2, float float_data3);
void UART_Draw_Float(float float_data);
void UART_Draw_Trajectory2(float float_data1, float float_data2, float float_data3, float float_data4);
void Sine_Wave_Test(float amplitude);

#endif
