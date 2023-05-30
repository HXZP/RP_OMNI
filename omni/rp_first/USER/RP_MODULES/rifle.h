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
	
	RIFLE_SHOOT_SET,
	RIFLE_SHOOT_STAY,
	
} rifle_ShootType;


typedef enum { 
	
	RIFLE_POSITION,
	
	RIFLE_SPEED,
	
} rifle_BoxType;


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
	
	rifle_state  Magazine;
	
	rifle_state  IgnoreHeat;

	rifle_state  AutoSetEnable;
	
	rifle_ShootType ShootType;
	rifle_BoxType   BoxType; 
	
	
	
	
	
//	rifle_state  SpeedRecordFlag;
	
}rifle_info;


typedef struct rifle_time_struct{

	uint32_t LockTime;
	uint32_t unLockTime;	

	uint32_t StuckJudegStart;	

	uint32_t StuckStart;

	uint32_t FriEnableTime;
	
	uint32_t ShootSetTime;
		
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
	
	float BoxSpeed;
	float BoxPosition;	
	float BoxTorque;
	
	float BoxNumSet;
	float BoxSpeedSet;
	float BoxPositionSet;
	
	float BoxSpeedErr;
	float BoxPositionErr;
	
	int16_t SpeedLimit;
	int16_t SpeedLimitPre;
	
	uint16_t HeatLimit;
	
	float Heat;	
	float Speed;
	float SpeedPre;
	
	uint16_t SpeedLowNum;
	int16_t  SpeedHightCom;
	
	uint16_t SpeedUpNum;
	uint16_t SpeedDownNum;	
	
	uint16_t BulletNum;

	uint16_t HeatEnableNum;

	float MagazineCCR;
	
	uint16_t SpeedDistribute[30];
	
}rifle_data;




typedef struct rifle_struct{

	rifle_info   info;
	rifle_data   data;
  rifle_time   time;
	rifle_friSet friTable;
	
	void (*ModifyLock)(struct rifle_struct *self,rifle_Lock type);
	void (*ModifyState)(rifle_state *self,rifle_state type);

	void (*ModifyFri)(struct rifle_struct *self,rifle_state state);
	void (*ModifyMagazine)(struct rifle_struct *self,rifle_state state);
	void (*ModifyShootType)(struct rifle_struct *self,rifle_ShootType type,uint16_t number);
	
	void (*Updata)(struct rifle_struct *self);
	void (*Resolving)(struct rifle_struct *self);	
	
	void (*BoxCtrl)(struct rifle_struct *self);	
	void (*FrictionCtrl)(struct rifle_struct *self);
  void (*MagazineCtrl)(struct rifle_struct *self);
	
	void (*Ctrl)(struct rifle_struct *self);	
}rifle;



extern rifle gun;







#endif

