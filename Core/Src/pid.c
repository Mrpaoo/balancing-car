#include"pid.h"
#include "pidConfig.h"

static inline float clampf(float v, float lo, float hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}


// @param pid: pid handler
// @param setpoint: expected motor run at this PWM, contorl the motor
// @param measurement: sample the encoder of Motor
// @param dt_sec: idk, maybe a
float Pid_Update(Pid_t* pid, float setpoint, float measurement, float dt_sec)
{
	//make sure the measurement come from real hardware e.g. encoder of wheel
    float err = setpoint - measurement;

    if (dt_sec > 0.0f) {
        pid->integral += err * dt_sec;
        pid->integral = clampf(pid->integral, -pid->integral_limit, pid->integral_limit);

        #define DER_LPF_ALPHA  0.10f   /* gyro D-term LPF: cutoff ~8Hz@100Hz */
        float der;
        if (pid->use_direct_deriv) {
            pid->der_filtered += DER_LPF_ALPHA * (pid->direct_deriv - pid->der_filtered);
            der = pid->der_filtered;
        } else {
            float raw_der = (err - pid->prev_error) / dt_sec;
            pid->prev_error = err;
            pid->der_filtered += DER_LPF_ALPHA * (raw_der - pid->der_filtered);
            der = pid->der_filtered;
        }

        pid->output = pid->Kp * err + pid->Ki * pid->integral + pid->Kd * der;
    }

    pid->output = clampf(pid->output, pid->output_min, pid->output_max);
    return pid->output;
}

 void Motor1Pid_Init(Pid_t* pid)
{
    pid->Kp = MOTOR1_Kp;
    pid->Ki = MOTOR1_Ki;
    pid->Kd = MOTOR1_Kd;
    pid->integral_limit = MOTOR1_INTEGRAL_LIMIT;
    pid->output_max = MOTOR1_OUTPUT_MAX;
    pid->output_min = MOTOR1_OUTPUT_MIN;
    Pid_Reset(pid);
}

 void Motor2Pid_Init(Pid_t* pid)
{
    pid->Kp = MOTOR2_Kp;
    pid->Ki = MOTOR2_Ki;
    pid->Kd = MOTOR2_Kd;
    pid->integral_limit = MOTOR2_INTEGRAL_LIMIT;
    pid->output_max = MOTOR2_OUTPUT_MAX;
    pid->output_min = MOTOR2_OUTPUT_MIN;
    Pid_Reset(pid);
}

 void AnglePid_Init(Pid_t* pid)
{
    pid->Kp = ANGLE_Kp;
    pid->Ki = ANGLE_Ki;
    pid->Kd = ANGLE_Kd;
    pid->integral_limit = ANGLE_INTEGRAL_LIMIT;
    pid->output_max = ANGLE_OUTPUT_MAX;
    pid->output_min = ANGLE_OUTPUT_MIN;
    pid->use_direct_deriv = 1;
    Pid_Reset(pid);
    pid->use_direct_deriv = 1;  /* re-apply after reset */
}

void Pid_Reset(Pid_t* pid)
{
   pid->integral = 0.0f;
   pid->prev_error = 0.0f;
   pid->der_filtered = 0.0f;
   pid->output = 0.0f;
   pid->direct_deriv = 0.0f;
   pid->use_direct_deriv = 0;
}
