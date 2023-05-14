#ifndef __RIFLE_H
#define __RIFLE_H

#include "type.h"

typedef enum { 
	
	RIFLE_LOCK,
	RIFLE_UNLOCK,
	
} rifle_Lock;

typedef enum { 
	
	RIFLE_MOTOR_ONLINE,
	RIFLE_MOTOR_ERR,
	
} rifle_MotorState;

typedef struct rifle_info_struct{

	rifle_MotorState MotorState;
  rifle_Lock       Lock;	
	
	
}rifle_info;


typedef struct rifle_time_struct{

	uint32_t LockTime;
	uint32_t unLockTime;	

}rifle_time;

typedef struct rifle_data_struct{

	float FriRpmR;
	float FriRpmL;
	
	float FriTempR;
	float FriTempL;
	
	float SpeedLimit;
	float HeatLimit;
	
	float Speed;
	float SpeedPre;
	
	float SpeedLowNum;
	float SpeedHightNum;
	
	float SpeedUpNum;
	float SpeedDownNum;	
	
	float BulletNum;
	
}rifle_data;

typedef struct rifle_struct{

	rifle_info info;
	rifle_data data;
  rifle_time time;
	
	

}rifle;

#endif

