/*
 * sensor.h
 *
 *  Created on: 2026年4月2日
 *      Author: 35973
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_
#include<stdint.h>
#include"cmsis_os.h"
#include"main.h"
#include"tim.h"

#define MPU6050_ADDR 0xD0
#define Enc_TIM_L	htim5
#define Enc_TIM_R	htim4


typedef struct EncData_T{
	uint16_t enc_L;
	uint16_t enc_R;
}EncData_T;

typedef struct CS100AData_T{
	uint32_t echo_start;   /* TIM8 ticks at rising edge */
	uint32_t echo_width;   /* pulse width in TIM8 ticks  */
	uint8_t  echo_ready;   /* 1 = new measurement ready  */
}CS100AData_T;

typedef struct MPU6050Data_T{
	//accelerate xyz
	int16_t accel_X;
	int16_t accel_Y;
	int16_t accel_Z;

	//Gyro xyz
	int16_t gyro_X;
	int16_t gyro_Y;
	int16_t gyro_Z;
}MPU6050Data_T;

typedef struct SensorData_t{
	float          pitch;       /* complementary-filtered pitch angle (deg) */
	float          gyro_dps;    /* raw gyro Y angular velocity (°/s), for D-term */
	MPU6050Data_T  mpu;
	EncData_T      enc;
	CS100AData_T   csa;

}SensorData_t;


void MPU6050_Init(void);
void MPU6050_CalibratePitch(void);
void collect_data(SensorData_t*);
void Sensor_Update(SensorData_t*, float dt);
void CS100A_Trigger(void);
float CS100A_EchoWidth_to_mm(uint32_t echo_width);
//Timer cb ,define at main.c
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_SENSOR_H_ */
