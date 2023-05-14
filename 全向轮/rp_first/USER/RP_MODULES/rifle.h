#ifndef __RIFLE_H
#define __RIFLE_H

#include "type.h"

typedef enum { 
	
	RIFLE_NO,
	RIFLE_OK,
	RIFLE_ING,	
	
} rifle_state;


typedef enum { 
	
	RIFLE_LOCK,
	RIFLE_UNLOCK,
	
} rifle_Lock;

typedef enum { 
	
	RIFLE_MOTOR_ONLINE,
	RIFLE_MOTOR_ERR,
	
} rifle_MotorState;

typedef enum { 
	
	RIFLE_SHOOT_STOP,
	
	RIFLE_SHOOT_STUCK,
	
	RIFLE_SHOOT_ONCE,
	RIFLE_SHOOT_STAY,
	RIFLE_SHOOT_7,	
	
} rifle_ShootType;

typedef struct rifle_info_struct{

	rifle_MotorState MotorState;
  rifle_Lock       Lock;	
	
	rifle_state  FriEnable;
	rifle_state  FriSpeedReach;	
	
	rifle_state  FriStucking;
	rifle_state  BoxStucking;

	rifle_state  HeatEnable;
	
	rifle_state  ShootReady;
	rifle_state  Shooting;	
	
	rifle_ShootType ShootType;
//	rifle_state  SpeedRecordFlag;
	
}rifle_info;


typedef struct rifle_time_struct{

	uint32_t LockTime;
	uint32_t unLockTime;	
	
	uint32_t StuckTime;
	uint32_t StuckTimeStart;		

	uint32_t FriEnableTime;
		
}rifle_time;

typedef struct{

	int16_t Speed0;
	int16_t Speed15;	
	
	int16_t Speed18;
	int16_t Speed20;		

	int16_t Speed22;
	int16_t Speed30;

}rifle_friSet;


typedef struct rifle_data_struct{

	float FriSet;
	float FriRpmR;
	float FriRpmL;
	
	float FriTempR;
	float FriTempL;
	
	int16_t SpeedLimit;
	int16_t SpeedLimitPre;
	
	int16_t HeatLimit;
	
	float Heat;
		
	float Speed;
	float SpeedPre;
	
	float BoxSpeed;
	float BoxPosition;	
	
	float BoxSpeedSet;
	float BoxPositionSet;
	
	int16_t SpeedLowNum;
	int16_t SpeedHightCom;
	
	int16_t SpeedUpNum;
	int16_t SpeedDownNum;	
	
	float BulletNum;

	uint16_t HeatEnableNum;
	
}rifle_data;




typedef struct rifle_struct{

	rifle_info   info;
	rifle_data   data;
  rifle_time   time;
	rifle_friSet friTable;
	

}rifle;

#endif

