#include "center.h"

#include "DEVICES.h"
#include "chassis.h"
#include "gimbal.h"
#include "rifle.h"

void Center_ModifiyState(Center_State *self,Center_State state);


center Center = {

	.modifyState = Center_ModifiyState,





};

void Center_Ctrl(void)
{
	if(rc.info.state == RC_OFFLINE){
		
		Center.modifyState(&Center.info.SysInit,  RP_NO);
		Center.modifyState(&Center.info.CtrlInit, RP_NO);
		Center.modifyState(&Center.info.MoveInit, RP_NO);
		Center.modifyState(&Center.info.GimbInit, RP_NO);
		Center.modifyState(&Center.info.RifleInit,RP_NO);
		
		head.ModifyLock(&head,GIMB_LOCK);
	  omni.ModifyLock(&omni,CHAS_LOCK);
	}
	else{
	
		if(Center.info.SysInit == RP_NO){
		
			Center.modifyState(&Center.info.CtrlInit,RP_NO);
			
		}
		
		
	
	
	
	
	}






}


void Center_ModifiyState(Center_State *self,Center_State state)
{
	if(*self != state){
	
		*self = state;
	}
}

























