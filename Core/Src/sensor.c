#include"sensor.h"
#include"sensorConfig.h"
#include"i2c.h"
#include<math.h>

#define GYRO_LSB_PER_DPS  131.0f   /* ±250°/s range */

static void MPU6050_read(MPU6050Data_T* data);
static void Encoder_read(EncData_T* data);

void MPU6050_Init(void)
{
	uint8_t val;
	HAL_StatusTypeDef ret;

	/* Retry WHO_AM_I until device responds (cold-boot can need several attempts) */
	for (int retry = 0; retry < 20; retry++) {
		ret = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x75, I2C_MEMADD_SIZE_8BIT,
		                       &val, 1, 100);
		if (ret == HAL_OK && val == 0x68) break;
		HAL_Delay(10);
	}

	/* Wake up device (clear sleep bit) */
	val = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);

	/* Gyro: ±250°/s */
	val = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);

	/* Accel: ±2g */
	val = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/* complementary filter state */
static float pitch_filtered = 0.0f;
static float gyro_bias_Y = 0.0f;   /* °/s, calibrated at boot */

void MPU6050_CalibratePitch(void)
{
#define CALIB_SAMPLES 100
	float ax_sum = 0.0f, az_sum = 0.0f, gy_sum = 0.0f;
	MPU6050Data_T raw;

	for (int i = 0; i < CALIB_SAMPLES; i++) {
		MPU6050_read(&raw);
		ax_sum += (float)raw.accel_X;
		az_sum += (float)raw.accel_Z;
		gy_sum += (float)raw.gyro_Y;
		HAL_Delay(2);
	}

	gyro_bias_Y = (gy_sum / CALIB_SAMPLES) / GYRO_LSB_PER_DPS;

	float ax = ax_sum / CALIB_SAMPLES;
	float az = az_sum / CALIB_SAMPLES;
	pitch_filtered = atan2f(ax, az) * 180.0f / (float)M_PI;
}

static void ComplementaryFilter(MPU6050Data_T *mpu, float dt)
{
	/* gyro: integrate angular velocity, bias subtracted */
	float gyro_rate = (float)mpu->gyro_Y / GYRO_LSB_PER_DPS - gyro_bias_Y;
	float pitch_gyro = pitch_filtered + gyro_rate * dt;

	/* accel: absolute angle from gravity vector */
	float pitch_accel = atan2f((float)mpu->accel_X, (float)mpu->accel_Z)
	                  * 180.0f / (float)M_PI;

	/* fuse: trust gyro 98%, accel 2% */
	pitch_filtered = 0.95f * pitch_gyro + 0.05f * pitch_accel;
}

#define SENSOR_DT  0.005f   /* TIM2 period = 5 ms */

/* Called from TIM2 ISR: encoder + MPU6050 + complementary filter */
void collect_data(SensorData_t* data){
	Encoder_read(&data->enc);
	// MPU6050 + filter moved to Sensor_Update() in thread context
}

/* Called from Motor Task thread: I2C read + complementary filter */
void Sensor_Update(SensorData_t* data, float dt){
	MPU6050_read(&data->mpu);
	ComplementaryFilter(&data->mpu, dt);
	data->pitch = pitch_filtered;
}

static void MPU6050_read(MPU6050Data_T* data){
	uint8_t buf[14];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, buf, 14, 100);

	 data->accel_X = (buf[0] << 8) | buf[1];
	data->accel_Y = (buf[2] << 8) | buf[3];
	data->accel_Z = (buf[4] << 8) | buf[5];
	data->gyro_X = (buf[8] << 8) | buf[9];
	data->gyro_Y = (buf[10] << 8) | buf[11];
	data->gyro_Z = (buf[12] << 8) | buf[13];
}


static void Encoder_read(EncData_T* data){
	data->enc_L = (uint16_t)__HAL_TIM_GET_COUNTER(&Enc_TIM_L);
	data->enc_R = (uint16_t)__HAL_TIM_GET_COUNTER(&Enc_TIM_R);
}

void CS100A_Trigger(void)
{
	HAL_GPIO_WritePin(CS100A_TRIG_GPIO_Port, CS100A_TRIG_Pin, GPIO_PIN_RESET);
	for (volatile uint32_t d = 0; d < 80; d++) { __NOP(); }
	HAL_GPIO_WritePin(CS100A_TRIG_GPIO_Port, CS100A_TRIG_Pin, GPIO_PIN_SET);
}

float CS100A_EchoWidth_to_mm(uint32_t echo_width)
{
	/* TIM8 @ 16 MHz, 1 tick = 62.5 ns
	   sound ≈ 343 m/s round-trip → 1 tick = 0.0107 mm */
	return (float)echo_width * 0.0107f;
}
