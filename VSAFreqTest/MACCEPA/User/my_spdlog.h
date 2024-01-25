/**
 * @file my_spdlog.h
 * @author your name (you@domain.com)
 * @brief stm32 串口日志打印的头文件，只需要在 main.c 文件中包含即可。
 *        注意要在 keil5 中配置勾选 Options——Target——Use MicroLIB，否则虽然不报错，但是无法打印。
 * @version 0.1
 * @date 2024-01-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __MY_SPDLOG_H__
#define __MY_SPDLOG_H__

/*-----------------------------1、配置 printf 命令-----------------------------------*/
// 重定向printf到串口
#include <stdio.h>

// ! 更换成实际使用的stm32型号
#include "stm32f4xx_hal.h"

/*-----------------------------2、配置自定义日志命令-----------------------------------*/
// 设置日志级别
#define USER_SPDLOG_TRACE
//#define USER_SPDLOG_DEBUG

// 日志级别定义
#ifdef USER_SPDLOG_TRACE
#define user_spdlog_trace(format, ...) \
  printf("[trace]: " format "\r\n", ##__VA_ARGS__)
#define USER_SPDLOG_DEBUG
#else
#define user_spdlog_trace(format, ...)
#endif

#ifdef USER_SPDLOG_DEBUG
#define user_spdlog_debug(format, ...) \
  printf("[debug]: " format "\r\n", ##__VA_ARGS__)
#define USER_SPDLOG_INFO
#else
#define user_spdlog_debug(format, ...)
#endif

#ifdef USER_SPDLOG_INFO
#define user_spdlog_info(format, ...) \
  printf("[info]: " format "\r\n", ##__VA_ARGS__)
#define USER_SPDLOG_WARN
#else
#define user_spdlog_info(format, ...)
#endif

#ifdef USER_SPDLOG_WARN
#define user_spdlog_warn(format, ...) \
  printf("[warn]: " format "\r\n", ##__VA_ARGS__)
#define USER_SPDLOG_ERROR
#else
#define user_spdlog_warn(format, ...)
#endif

#ifdef USER_SPDLOG_ERROR
#define user_spdlog_error(format, ...) \
  printf("[error]: " format "\r\n", ##__VA_ARGS__)
#else
#define user_spdlog_error(format, ...)
#endif

#endif  // __MY_SPDLOG_H__
