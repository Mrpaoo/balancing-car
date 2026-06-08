/*
 * pid.h
 *
 *  Created on: 2026 4 /2
 *      Author: 35973
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct Pid_t{
	/* ===== Config (read-only after init) ===== */
		float Kp, Ki, Kd;
	    float integral_limit;          // |integral| clamp
	    float output_max, output_min;  // output clamp range

	    /* ===== State (runtime, changes every update) ===== */
	    float integral;
	    float prev_error;
	    float der_filtered;  /* low-pass filtered derivative */
	    float output;	/*only suggestion, not real output motor drive*/

	    /* ===== External derivative source ===== */
	    uint8_t use_direct_deriv;
	    float   direct_deriv;  /* pre-computed d(error)/dt, e.g. -gyro_dps */

}Pid_t;

void Motor1Pid_Init(Pid_t* pid);
void Motor2Pid_Init(Pid_t* pid);
void AnglePid_Init(Pid_t* pid);
void Pid_Reset(Pid_t* pid);
float Pid_Update(Pid_t* pid,float setpoint,float measurement, float dt_sec);


#endif /* INC_PID_H_ */
