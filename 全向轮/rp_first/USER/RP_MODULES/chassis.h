#ifndef __CHASSIS_H
#define __CHASSIS_H


#include "RP_FUNCTION.h"

typedef enum { 
	
	CHAS_OMNI,
	CHAS_MECA,
	CHAS_HELM,
	
} chassis_Type;

typedef enum { 
	
	CHAS_LOCK,
	CHAS_UNLOCK,
	
} chassis_Lock;

typedef enum { 
	
	CHAS_ONLINE,
	CHAS_ERR,
	
} chassis_State;

typedef enum { 
	
	CHAS_FORWARD,
	CHAS_BACKWARD,
	
} chassis_Direction;


typedef struct {

	float x;
	float y;
	float z;
	
}chassis_xyz;


typedef struct chassis_info_struct{

	chassis_Type       Type;
	
  chassis_State      MotorState;
	
  chassis_Direction  Direction;
	chassis_Lock       Lock;
	
	float ReductionRatio;
  float WheelRadius;
	float VehicleLength;
	float VehicleWide;
	
	float OriginAngle;
		
//	float RotationRate;
	
}chassis_info;

typedef struct chassis_time_struct{

	uint32_t LockTime;
	uint32_t unLockTime;

}chassis_time;

typedef struct chassis_data_struct{

	float       VelocityMax;
	chassis_xyz VelocitySet;
	chassis_xyz VelocityReal;

	float       WheelrpmMax;	
	float       WheelSet[4];
	float       WheelReal[4];

	float       PowerLimit;
	float       PowerBuff;
	
	float       DirAngle;
	
}chassis_data;




typedef struct chassis_struct{

	chassis_info info;
	chassis_data data;
  chassis_time time;

  void (*ModifyLock)(struct chassis_struct *chas,chassis_Lock type);
  void (*ModifyrpmMax)(struct chassis_struct *chas,float max);
	void (*ModifyXYZSet)(struct chassis_struct *chas,float setX,float setY,float setZ);
	void (*ModifyOriginAngle)(struct chassis_struct *chas,float angle);

	void (*Updata)(struct chassis_struct *chas);
	void (*Resolving)(struct chassis_struct *chas);
	void (*Ctrl)(struct chassis_struct *chas);

	
}chassis;

extern chassis omni;

#endif









