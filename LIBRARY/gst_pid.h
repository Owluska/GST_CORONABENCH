/*
 * gst_pid.h
 *
 *  Created on: 23 апр. 2020 г.
 *      Author: Ксения
 */

#ifndef GST_PID_H_
#define GST_PID_H_
/**
 * @brief  Parameters for PID calculations
 */
typedef struct
{
	float Kp;     /*!< Proportional part coefficient */
	float Ki;     /*!< Integral part coefficient */
	float Kd;     /*!< Differential part coefficient */

	float hold;   /*!< hold position value */
	float out;    /*!< PID out */
	float e;      /*!< PID error */


	float P;      /*!< Proportional part*/
	float I;      /*!< Integral part */
	float D;      /*!< Differential part*/
}PIDtypedef;
/**
 * @brief  Inits PID data structure
 * @param  *PID: Pointer to @ref PIDtypedef structure saving PID state
 * @param *KP: proportional coefficient value
 * @param *KI: integral coefficient value
 * @param *KD: differential coefficient value
 * @param *hold: PID hold position value
 * @retval pointer to new @ref PIDtypedef
 */
PIDtypedef* PID_init(PIDtypedef *PID, float *KP, float *KI, float *KD, float *hold);
/**
 * @brief  Calculates integral part of PID and rewrites PID structure
 * @param  *PID: Pointer to @ref PIDtypedef structure saving actual PID state
 * @param PID_e: PID error value
 */
void integral_part(PIDtypedef *PID, float PID_e);
/**
 * @brief  Calculates differential part of PID and rewrites PID structure
 * I = Iprevious+Ki * err;
 * @param  *PID: Pointer to @ref PIDtypedef structure saving actual PID state
 * @param PID_e: PID error value
 */
void differential_part(PIDtypedef *PID, float PID_e);
/**
 * @brief  Calculates proportional part of PID and rewrites PID structure
 * P=Kp*error
 * @param  *PID: Pointer to @ref PIDtypedef structure saving actual PID state
 * @param PID_e: PID error value
 */
void proportional_part(PIDtypedef *PID, float PID_e);
/**
 * @brief  Calculates PID and rewrites PID structure
 * @param  *PID: Pointer to @ref PIDtypedef structure saving actual PID state
 * @param PID_e: PID error value
 */
void PID(PIDtypedef *PID, float PID_e);

#endif /* GST_PID_H_ */
