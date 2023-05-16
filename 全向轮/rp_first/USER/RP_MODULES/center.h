#ifndef __CENTER_H
#define __CENTER_H

#include "type.h"

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
	MOVE_VISION,
	
} Center_MoveMode;

typedef enum { 
	
	RIFLE_NULL,
	
	RIFLE_STOP,
	
	RIFLE_MODE1,
	RIFLE_MODE2,
	RIFLE_MODE3,
	
} Center_RifleMode;

typedef struct center_info_struct{

  Center_State    SysInit;
	
	Center_State    CtrlInit;
	Center_State    MoveInit;	
  Center_State    RifleInit;
	
	Center_CtrlMode  CtrlMode;	
	Center_MoveMode  MoveMode;
	Center_RifleMode RifleMode;
}center_info;

typedef struct center_time_struct{

	uint32_t InitTimeUse;
	uint32_t InitTimeStart;	
	uint32_t InitTimeEnd;	
	
}center_time;

typedef struct center_data_struct{

	float GimbYawTarget;
	float GimbPitTarget;
	
}center_data;

typedef struct center_struct{

	center_info info;
	center_time time;
	center_data data;
	
	
	void (*modifyState)(Center_State *self,Center_State state);
	void (*modifyCtrlMode)(struct center_struct *self,Center_CtrlMode state);
	void (*modifyMoveMode)(struct center_struct *self,Center_MoveMode state);
	void (*modifyRifleMode)(struct center_struct *self,Center_RifleMode state);
	
	void (*Switch)(void);
	void (*Ctrl)(void);
	
}center;

extern center Center;


#endif

