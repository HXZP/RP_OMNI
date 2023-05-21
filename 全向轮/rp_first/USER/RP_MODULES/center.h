#ifndef __CENTER_H
#define __CENTER_H

#include "type.h"
#include "string.h"





typedef enum { 
	
	RP_NO,
	RP_ING,
	RP_OK,
	
} Center_State;

typedef enum { 
	
	CTRL_NULL,
	
	CTRL_RC,
	CTRL_KM,
	
} Center_CtrlMode;

typedef enum { 
	
	MOVE_NULL,
	
	MOVE_MASTER,
	MOVE_FOLLOW,
	
} Center_MoveMode;

typedef enum { 
	
	RIFLE_NULL,
	
	RIFLE_STOP,
	
	RIFLE_SET1,
	RIFLE_SET6,	
	
	RIFLE_STAY,
	

	
} Center_RifleMode;

typedef enum { 
	
	VISION_NULL,
	
	VISION_AIM,
	
	VISION_BIG_BUFF,
	VISION_MIN_BUFF,
	
} Center_VisionMode;

typedef struct center_info_struct{

  Center_State    SysInit;
	
	Center_State    CtrlInit;
	Center_State    MoveInit;	
  Center_State    RifleInit;
  Center_State    VisionInit;	
	
	Center_State    RifleLock;

	Center_State    FrictionSwitch;
	Center_State    MagazineSwitch;
	
	Center_State    RotateSwitch;
	
	Center_CtrlMode   CtrlMode;	
	Center_MoveMode   MoveMode;
	Center_RifleMode  RifleMode;
  Center_VisionMode VisionMode;	
	
}center_info;

typedef struct center_time_struct{

	uint32_t AimEnterTimeStart;
	uint32_t InitTimeStart;	
	
	uint32_t CenterStartTime;		
	
}center_time;

typedef struct center_data_struct{

	float GimbYawTarget;
	float GimbPitTarget;
	
	float VelocityX;
	float VelocityY;
	float VelocityZ;
	
	float Channel[4];
		
//	float MasterDire;
	
	float RotateVelocity;
	
}center_data;

typedef struct center_struct{

	center_info info;
	center_time time;
	center_data data;
	
	
	void (*modifyState)(Center_State *self,Center_State state);
	
	void (*modifyCtrlMode)(struct center_struct *self,Center_CtrlMode state);
	void (*modifyMoveMode)(struct center_struct *self,Center_MoveMode state);
	void (*modifyRifleMode)(struct center_struct *self,Center_RifleMode state);
	void (*modifyVisionMode)(struct center_struct *self,Center_VisionMode state);
	
	void (*Switch)(struct center_struct *self);
	void (*Updata)(struct center_struct *self);
	void (*Ctrl)(struct center_struct *self);
	
}center;

extern center Center;


#endif

