#include "GimbalRotationOutput.h"
#include "arm_math.h"

/*
float chasRoll,float chasPitch,float chasYaw:底盘姿态欧拉角 单位弧度
float gimbRoll,float gimbPitch,float gimbYaw:云台电机角度   单位弧度
float *outRoll,float *outPitch,float *outYaw:电机输出值     单位 无
*/
void RP_RotationOutput_Chassis2Gimb(float chasRoll,float chasPitch,float chasYaw,
	                                  float gimbRoll,float gimbPitch,float gimbYaw,
		                                float *outRoll,float *outPitch,float *outYaw);

///*
//旋转 左乘
//*/
//static void RP_QuaternionMartix_LeftMult(float q0,float q1,float q2,float q3,float x,float y,float z,float *ax,float *ay,float *az)
//{
//	
//	float a,b,c,d;
//	
//	a = q0;
//	b = q1;
//	c = q2;
//	d = q3;
//	
//	*ax = z*(2*a*c + 2*b*d) - y*(2*a*d - 2*b*c) + x*(a*a + b*b - c*c - d*d);
//	*ay = x*(2*a*d + 2*b*c) - z*(2*a*b - 2*c*d) + y*(a*a - b*b + c*c - d*d);
//	*az = y*(2*a*b + 2*c*d) - x*(2*a*c - 2*b*d) + z*(a*a - b*b - c*c + d*d);

//}



/*
映射 右乘
*/
static void RP_QuaternionMartix_RightMult(float q0,float q1,float q2,float q3,float x,float y,float z,float *ax,float *ay,float *az)
{
	
	float a,b,c,d;
	
	a = q0;
	b = q1;
	c = q2;
	d = q3;
	
	*ax = y*(2*a*d + 2*b*c) - z*(2*a*c - 2*b*d) + x*(a*a + b*b - c*c - d*d);
	*ay = z*(2*a*b + 2*c*d) - x*(2*a*d - 2*b*c) + y*(a*a - b*b + c*c - d*d);
	*az = x*(2*a*c + 2*b*d) - y*(2*a*b - 2*c*d) + z*(a*a - b*b - c*c + d*d);

}

//static void RP_Quaternion_2_EulerAngle(float q0,float q1,float q2,float q3,float *r,float *p,float *y)
//{

//	float norm,sintemp,costemp;

//	norm = inVSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//	q0 = q0 *norm;
//	q1 = q1 *norm;
//	q2 = q2 *norm;
//	q3 = q3 *norm;
//	
//	sintemp = 2 * q1 * q3 -2 * q0* q2;
//	arm_sqrt_f32(1 - sintemp * sintemp, &costemp);
//	
//	arm_atan2_f32(2 * q2 * q3 + 2 * q0 * q1, q0*q0 - q1 * q1 - q2 * q2 + q3 * q3, r);
//  arm_atan2_f32(sintemp, costemp, p);
//	arm_atan2_f32(2 *(q1 * q2 + q0* q3),     q0*q0 + q1 * q1 - q2 * q2 - q3 * q3, y);
//	
//	*p = -*p; 

//}

static void RP_EulerAngle_2_Quaternion(float *q0,float *q1,float *q2,float *q3,double r,double p,double y)
{
    double cy = arm_cos_f32(y * 0.5);
    double sy = arm_sin_f32(y * 0.5);
    double cp = arm_cos_f32(p * 0.5);
    double sp = arm_sin_f32(p * 0.5);
    double cr = arm_cos_f32(r * 0.5);
    double sr = arm_sin_f32(r * 0.5);
 
    *q0 = cy * cp * cr + sy * sp * sr;
    *q1 = cy * cp * sr - sy * sp * cr;
    *q2 = sy * cp * sr + cy * sp * cr;
    *q3 = sy * cp * cr - cy * sp * sr;
}



/*
float chasRoll,float chasPitch,float chasYaw:底盘姿态欧拉角
float gimbRoll,float gimbPitch,float gimbYaw:云台电机角度
float *outRoll,float *outPitch,float *outYaw:电机输出值
*/
void RP_RotationOutput_Chassis2Gimb(float chasRoll,float chasPitch,float chasYaw,
	                                  float gimbRoll,float gimbPitch,float gimbYaw,
		                                float *outRoll,float *outPitch,float *outYaw)
{
    fun_quaternion_t motorQ,c2gQ;
	
    RP_EulerAngle_2_Quaternion(&motorQ.q0,&motorQ.q1,&motorQ.q2,&motorQ.q3,gimbRoll,gimbPitch,gimbYaw);

    //转换相对云台的底盘姿态  //剔除z轴影响	 
	  chasYaw = 0;													 
		RP_QuaternionMartix_RightMult(motorQ.q0,
																  motorQ.q1,
																  motorQ.q2,
																  motorQ.q3,	
															 
																 chasRoll, chasPitch, chasYaw,
																 &chasRoll,&chasPitch,&chasYaw); 
		//获取云台底盘相对四元数			  														 
		RP_EulerAngle_2_Quaternion(&c2gQ.q0,
															 &c2gQ.q1,
															 &c2gQ.q2,
															 &c2gQ.q3,
															 
															 chasRoll,chasPitch,chasYaw);															
													
    //解算实际输出															 
		RP_QuaternionMartix_RightMult(c2gQ.q0,
																  c2gQ.q1,
																  c2gQ.q2,
																  c2gQ.q3,

																 *outRoll,*outPitch,*outYaw,
																 outRoll, outPitch, outYaw);




}






























