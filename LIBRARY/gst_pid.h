/*
 * gst_pid.h
 *
 *  Created on: 23 апр. 2020 г.
 *      Author: Ксения
 */

#ifndef GST_PID_H_
#define GST_PID_H_

typedef struct
{
	float Kp;
	float Ki;
	float Kd;

	float hold;
	float out;
	float e;



	float P;
	float I;
	float D;
}PIDtypedef;

PIDtypedef* PID_init(PIDtypedef* PID, float *KP, float *KI, float *KD, float *hold);
void integral_part(PIDtypedef *PID, float PID_e);
void differential_part(PIDtypedef *PID, float PID_e);
void proportional_part(PIDtypedef *PID, float PID_e);
void PID(PIDtypedef *PID, float PID_e);

#endif /* GST_PID_H_ */
