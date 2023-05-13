#ifndef __CHASSIS_H
#define __CHASSIS_H

typedef enum { 
	
	CHAS_LOCK,
	CHAS_UNLOCK,
	
} chassis_Lock;

typedef enum { 
	
	CHAS_MASTER,
	CHAS_FOLLOW,
	
} chassis_CtrlMode;

typedef enum { 
	
	CHAS_MOTOR_ONLINE,
	CHAS_MOTOR_ERR,
	
} chassis_MotorState;

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

	chassis_CtrlMode   CtrlMode;
  chassis_MotorState MotorState;
  chassis_Direction  Direction;
	chassis_Lock       Lock;
	
	float ReductionRatio;
  float WheelRadius;
	float VehicleLength;
	float VehicleWide;
	
}chassis_info;


typedef struct chassis_data_struct{

	float       VelocityMax;
	chassis_xyz VelocitySet;
	chassis_xyz VelocityReal;

	float       WheelrpmMax;	
	float       WheelSet[4];
	float       WheelReal[4];

}chassis_data;




typedef struct chassis_struct{

	chassis_info info;
	chassis_data data;




}chassis;
















#endif





