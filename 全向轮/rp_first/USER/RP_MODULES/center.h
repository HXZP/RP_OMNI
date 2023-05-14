#ifndef __CENTER_H
#define __CENTER_H

#include "type.h"

typedef enum { 
	
	RP_NO,
	RP_ING,
	RP_OK,
	
} Center_State;

typedef enum { 
	
	MOVE_MASTER,
	MOVE_FOLLOW,
	MOVE_VISION,
	
} Center_MoveMode;

typedef enum { 
	
	CTRL_RC,
	CTRL_KM,
	
} Center_CtrlMode;

typedef struct center_info_struct{

  Center_State    SysInit;
	Center_State    CtrlInit;
	Center_State    MoveInit;	
	Center_State    GimbInit;
  Center_State    RifleInit;
	
	Center_CtrlMode CtrlMode;	
	
}center_info;

typedef struct center_time_struct{

	uint32_t InitTimeUse;
	uint32_t InitTimeStart;	
	uint32_t InitTimeEnd;	
}center_time;


typedef struct center_struct{

	center_info info;
	center_time time;
	
	void (*modifyState)(Center_State *self,Center_State state);
	
}center;

#endif

