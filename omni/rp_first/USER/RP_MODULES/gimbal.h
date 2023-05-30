#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "type.h"

typedef enum { 
	
	GIMB_NO,
	GIMB_OK,
	
} gimbal_state;

typedef enum { 
	
	GIMB_LOCK,
	GIMB_UNLOCK,
	
} gimbal_Lock;

typedef enum { 
	
	GIMB_MOTOR_ONLINE,
	GIMB_MOTOR_ERR,
	
} gimbal_MotorState;


typedef struct{

	float X;
	float Y;
	float Z;

}gimbal_xyz;


typedef struct gimbal_info_struct{

	gimbal_MotorState MotorState;
  gimbal_Lock       Lock;	
	
	gimbal_state YReach;
	gimbal_state ZReach;
	
	float elevation;
	float depression;
	
	gimbal_xyz AssemblyVector;
	
}gimbal_info;

typedef struct gimbal_time_struct{

	uint32_t LockTime;
	uint32_t unLockTime;	

}gimbal_time;

typedef struct gimbal_data_struct{

	gimbal_xyz Angle;
	gimbal_xyz Speed;
	gimbal_xyz Torque;
	
	gimbal_xyz AngleSet;
	gimbal_xyz AngleErr;
	
}gimbal_data;


typedef struct gimbal_struct{

	gimbal_info info;
	gimbal_data data;
  gimbal_time time;
	
  void (*ModifyLock)(struct gimbal_struct *gimb,gimbal_Lock type);
	void (*ModifyXYZSet)(struct gimbal_struct *gimb,float setX,float setY,float setZ);	
	float(*ModifyRange)(float data,float min,float max);
	
	void (*Updata)(struct gimbal_struct *gimb,float ax,float ay,float az,float rx,float ry,float rz);
	void (*Resolving)(struct gimbal_struct *gimb);
	void (*Translation)(struct gimbal_struct *gimb,float chasX,float chasY,float chasZ);
	void (*Ctrl)(struct gimbal_struct *gimb);
	
}gimbal;

extern gimbal head;

#endif
