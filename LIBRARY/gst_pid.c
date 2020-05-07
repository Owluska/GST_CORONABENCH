/*
 * gst_pid.c
 *
 *  Created on: 23 апр. 2020 г.
 *      Author: Ксения
 */
#include "gst_pid.h"
//. is the member of a structure
//-> is the member of a POINTED TO structure
//That is, a->b is equivalent to (*a).b

PIDtypedef* PID_init(PIDtypedef* PID, float *KP, float *KI, float *KD, float *hold)
{

	PID->Kp = *KP;
	PID->Ki = *KI;
	PID->Kd = *KD;

	PID->hold = *hold;

	PID->out = 0;
	PID->e = 0;


	return PID;
}




void integral_part(PIDtypedef *PID, float PID_e)
{
	PID->e=PID_e;
	PID->I += PID->Ki * PID->e;
}

void differential_part(PIDtypedef *PID, float PID_e)
{

	//PID->e=PID_e;
	PID->D = PID->Kd * (PID->e - PID_e);
	PID->e=PID_e;
}

void proportional_part(PIDtypedef *PID, float PID_e)
{
	PID->e = PID_e;
	PID->P=PID->Kp*PID->e;
}

void PID(PIDtypedef *PID, float PID_e)
{
	proportional_part(PID, PID_e);
	differential_part(PID, PID_e);
	integral_part(PID, PID_e);
	PID->out=PID->P+PID->I+PID->D;
}
