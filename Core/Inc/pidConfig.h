/*
 * pidConfig.h
 *
 *  Created on: 2026年4月2日
 *      Author: 35973
 */

#ifndef INC_PIDCONFIG_H_
#define INC_PIDCONFIG_H_



// ========== 电机1 PID 参数 ==========
#define MOTOR1_Kp         0.1f
#define MOTOR1_Ki         0.0f
#define MOTOR1_Kd         0.0f
#define MOTOR1_INTEGRAL_LIMIT   100.0f   // 积分累积上限
#define MOTOR1_OUTPUT_MAX       50.0f   // 输出上限（占空比%）
#define MOTOR1_OUTPUT_MIN      -50.0f   // 输出下限

// ========== 电机2 PID 参数 ==========
#define MOTOR2_Kp         0.1f
#define MOTOR2_Ki         0.0f
#define MOTOR2_Kd         0.0f
#define MOTOR2_INTEGRAL_LIMIT   100.0f
#define MOTOR2_OUTPUT_MAX       50.0f
#define MOTOR2_OUTPUT_MIN      -50.0f

// ========== 角度环 PID 参数 ==========
// 角度PD直接输出PWM占空比(%), 不再经过速度环
// D项直接拿gyro_dps(陀螺仪角速度), 延迟<1ms
#define ANGLE_Kp                10.0f
#define ANGLE_Ki                0.0f
#define ANGLE_Kd                8.0f
#define ANGLE_INTEGRAL_LIMIT   10.0f
#define ANGLE_OUTPUT_MAX       80.0f   // 平衡PWM占空比上限(%)
#define ANGLE_OUTPUT_MIN      -80.0f

// 速度指令转换 (仅REMOTE/TRACKING模式用)
#define SPEED_CMD_GAIN		20.0f

// ========== 通用 PID 配置 ==========
#define PID_DEFAULT_INTEGRAL_LIMIT   100.0f
#define PID_DEFAULT_OUTPUT_MAX       100.0f
#define PID_DEFAULT_OUTPUT_MIN      -100.0f

#endif


