#include "center.h"

#include "DEVICES.h"
#include "chassis.h"
#include "gimbal.h"
#include "rifle.h"

void Center_ModifiyState(Center_State *self,Center_State state);
void Center_ModifiyCtrlMode(center *self,Center_CtrlMode state);
void Center_ModifiyMoveMode(center *self,Center_MoveMode state);
void Center_ModifiyRifleMode(center *self,Center_RifleMode state);
void Center_Switch(void);


center Center = {

	.modifyState = Center_ModifiyState,

  .modifyCtrlMode  = Center_ModifiyCtrlMode,
	.modifyMoveMode  = Center_ModifiyMoveMode,
  .modifyRifleMode = Center_ModifiyRifleMode,

	.Switch = Center_Switch,
};

void Center_Switch(void)
{
	if(rc.info.state == RC_OFFLINE){
		
		head.ModifyLock(&head,GIMB_LOCK);
		omni.ModifyLock(&omni,CHAS_LOCK);

	}
	else{
		
		switch(SW1){
		
			case SW_UP:
				Center.modifyCtrlMode(&Center,CTRL_KM);
				break;
			
			default:
				Center.modifyCtrlMode(&Center,CTRL_RC);
				break;		
		
		}
		
		if(Center.info.CtrlMode == CTRL_RC){
		

			
			
			
		}
		else if(Center.info.CtrlMode == CTRL_KM){
		
		
		
			
		
		
		}
	}
}

void Center_Ctrl(void)
{	

}


void Center_CtrlModeInit(void)
{




}


void Center_MoveModeInit(void)
{




}

void Center_RifleModeInit(void)
{




}


void Center_ModifiyState(Center_State *self,Center_State state)
{
	if(*self != state){
	
		*self = state;
	}
}



void Center_ModifiyCtrlMode(center *self,Center_CtrlMode state)
{
	if(self->info.CtrlMode == state)return;

	if(self->info.CtrlMode != state && self->info.CtrlInit != RP_OK){
	
		self->info.CtrlMode = CTRL_NULL;
		Center.modifyState(&Center.info.CtrlInit, RP_ING);
	}
	
	if(self->info.CtrlInit == RP_ING){
	
		Center_CtrlModeInit();

	}
	
}

void Center_ModifiyMoveMode(center *self,Center_MoveMode state)
{
	if(self->info.MoveMode == state)return;
	
	if(self->info.MoveMode != state && self->info.MoveMode != MOVE_NULL){
	
		self->info.MoveMode = MOVE_NULL;
		Center.modifyState(&Center.info.MoveInit, RP_ING);		
	}

	if(self->info.MoveInit == RP_ING){
	
		Center_MoveModeInit();

	}	
}

void Center_ModifiyRifleMode(center *self,Center_RifleMode state)
{
	if(self->info.RifleMode != state && self->info.RifleMode != RIFLE_NULL){
	
		self->info.RifleMode = RIFLE_NULL;
		Center.modifyState(&Center.info.RifleInit, RP_ING);		
	}

	if(self->info.RifleInit == RP_ING){
	
		Center_RifleModeInit();

	}		
}


















