/*
 * motor.h
 *
 *  Created on: 2026年4月2日
 *      Author: 35973
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include<stdint.h>
#include"main.h"
#include"pid.h"

/*max delta for clamp() output*/
#define DU_MAX	50
#define PWM_PERIOD  999

typedef enum{
	IDLE=0,
	BALANCE,
	REMOTE,
	TRACKING,
}Robot_State_t;

/* ===== Motion command (app layer → driver layer) ===== */
typedef struct Move_Cmd{
	float v;        /* linear speed, encoder counts/s */
	int32_t turn;   /* differential speed, encoder counts/s */
}Move_Cmd;

typedef struct Wifi_Cmd{
	int32_t v;
	int32_t turn;
	int32_t mode_request;   /* 0=none, or Robot_State_t value */
}Wifi_Cmd;

/* ===== Hardware binding (init once, read-only after) ===== */
typedef struct Motor_HW{
	TIM_HandleTypeDef *htim_pwm;
	uint32_t           pwm_ch;       /* CHx (CHxN follows in complementary mode) */
	int32_t            pwm_period;   /* ARR value, TIM1 = 999 */
}Motor_HW;

/* ===== Pure software runtime state ===== */
typedef struct Motor_Ctrl{
	Pid_t   pid;
	int32_t prev_out;
	int32_t out;
}Motor_Ctrl;

/* ===== One motor = HW + Ctrl ===== */
typedef struct Motor_Unit{
	Motor_HW   hw;
	Motor_Ctrl ctrl;
}Motor_Unit;

/* ===== Robot top-level ===== */
typedef struct Robot_Ctx{
	Robot_State_t  state;
	Motor_Unit     left;
	Motor_Unit     right;
}Robot_Ctx;

/*
 * =======Driver the motor========
 * */

void MotorTask_Create(void);
void Robot_Init(void);

#endif /* INC_MOTOR_H_ */
