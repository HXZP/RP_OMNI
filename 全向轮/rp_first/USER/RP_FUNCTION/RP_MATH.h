#ifndef __RP_MATH_H
#define __RP_MATH_H

#include "stm32f4xx_hal.h"
#include "type.h"


#define pi 3.141592654

/* ��ֵ���� */
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define anti_constrain(x, min, max)	(((x<max) && (x>min))?(0):x)

#define abs(x) 					((x)>0? (x):(-(x)))



typedef struct{

	float X;
	float Y;
  float Z;


}RP_xyz;


typedef struct{

	float q0;
	float q1;
  float q2;
  float q3;

}RP_q;












int16_t RampInt(int16_t final, int16_t now, int16_t ramp);
float RampFloat(float final, float now, float ramp);
float DeathZoom(float input, float center, float death);
float RP_GetAbsoluteMax(float *data,uint16_t num);
float RP_Limit(float tar,float range);














#endif


