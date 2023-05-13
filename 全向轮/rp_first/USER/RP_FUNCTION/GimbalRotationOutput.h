#ifndef __GIMBAL_ROTATION_OUTPUT_H
#define __GIMBAL_ROTATION_OUTPUT_H



typedef struct{

	float q0;
	float q1;
	float q2;
	float q3;
	
}fun_quaternion_t;



void RP_RotationOutput_Chassis2Gimb(float chasRoll,float chasPitch,float chasYaw,
	                                  float gimbRoll,float gimbPitch,float gimbYaw,
		                                float *outRoll,float *outPitch,float *outYaw);






#endif


