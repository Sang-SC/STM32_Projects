/**
 * @file my_spdlog.c
 * @author your name (you@domain.com)
 * @brief stm32 串口日志打印的源文件
 * @version 0.1
 * @date 2024-01-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "my_spdlog.h"

extern UART_HandleTypeDef huart1;  // ! 更换成实际使用的stm32串口

// ifdef 语句用于判断是否为C++编译器，如果是则按照C++语法编译，否则按照C语法编译
#ifdef __cplusplus
extern "C" {
#endif

// 重定向printf到串口
int fputc(int ch, FILE* f) {
  uint8_t temp[1] = {uint8_t(ch)};
  HAL_UART_Transmit(&huart1, temp, 1, 2);  // ! 更换成实际使用的stm32串口
  return ch;
}

#ifdef __cplusplus
}
#endif
