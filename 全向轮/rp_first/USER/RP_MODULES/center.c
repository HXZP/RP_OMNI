



/*






目前没什么思路以下都是大便







*/







#include "center.h"

#include "DEVICES.h"
#include "chassis.h"
#include "gimbal.h"
#include "rifle.h"
#include "RP_CONFIG.h"



void Center_ModifiyState(Center_State *self,Center_State state);
void Center_ModifiyCtrlMode(center *self,Center_CtrlMode state);
void Center_ModifiyMoveMode(center *self,Center_MoveMode state);
void Center_ModifiyRifleMode(center *self,Center_RifleMode state);

void Center_Switch(center *self);
void Center_Ctrl(center *self);

center Center = {

	
	.info.SysInit = RP_NO,
	
	.modifyState = Center_ModifiyState,

  .modifyCtrlMode  = Center_ModifiyCtrlMode,
	.modifyMoveMode  = Center_ModifiyMoveMode,
  .modifyRifleMode = Center_ModifiyRifleMode,

	.Switch = Center_Switch,
	.Ctrl   = Center_Ctrl,
};

void Center_Switch(center *self)
{
	if(rc.info.state == RC_OFFLINE){
		
		self->info.RifleLock = RP_NO;
		
		self->info.VisionSwitch = RP_NO;
		
		self->modifyCtrlMode(self,CTRL_NULL);
		self->modifyMoveMode(self,MOVE_NULL);
		self->modifyRifleMode(self,RIFLE_NULL);
		
		gun.ModifyLock(&gun,RIFLE_LOCK);
		head.ModifyLock(&head,GIMB_LOCK);
		omni.ModifyLock(&omni,CHAS_LOCK);

	}
	else{
		
		/* 控制模式 */
		switch(SW1){
		
			case SW_UP:
				self->modifyCtrlMode(self,CTRL_KM);
				break;
			
			default:
				self->modifyCtrlMode(self,CTRL_RC);
				break;		
		}
		
		
		/* 遥控模式 */
		if(self->info.CtrlMode == CTRL_RC){
			
			/* 移动模式 */
			switch(SW1){
			
				case SW_MID:
					self->modifyMoveMode(self,MOVE_MASTER);
					break;
				
				case SW_DOWN:
					self->modifyMoveMode(self,MOVE_FOLLOW);
					break;	
			}
			
			/* 云台模式 */
			if(self->info.MoveMode == MOVE_MASTER){
				
				
			}
			/* 底盘模式 */
			else if(self->info.MoveMode == MOVE_FOLLOW){
			
				
			}
			/* 移动模式end */
			
							
			/* 发射模式 */
			
			if(rc.data.tw_step[0]){
			
				self->info.FrictionSwitch = RP_OK;
			}
			else{
			
				self->info.FrictionSwitch = RP_NO;
			}
			
			if(rc.data.tw_step[1]){
			
				self->info.MagazineSwitch = RP_OK;
			}
			else{
			
				self->info.MagazineSwitch = RP_NO;
			}
			
			switch(SW2){
			
				case SW_MID:
					self->info.RifleLock = RP_OK;
					self->modifyRifleMode(self,RIFLE_STOP);
					break;
				
				case SW_UP:
					
					self->info.RifleLock = RP_OK;
					self->modifyRifleMode(self,RIFLE_STAY);
					break;		
				
				case SW_DOWN:
					self->info.RifleLock = RP_OK;
					self->modifyRifleMode(self,RIFLE_SET);
					break;				
			}
			
			if(self->info.RifleLock == RP_NO){
			
				self->modifyRifleMode(self,RIFLE_STOP);
			
			}
			/* 发射模式end */
			
		}
		/* 遥控模式end */
		
		
		/* 键盘模式 */
		else if(self->info.CtrlMode == CTRL_KM){
		
			self->info.RifleLock = RP_OK;
		
			
			
		
		}
		/* 键盘模式end */
	}
	
	/* 控制模式end */
}



void Center_CtrlModeInit(center *self)
{




}


void Center_MoveModeInit(center *self)
{




}

void Center_RifleModeInit(center *self)
{




}



void Center_Ctrl(center *self)
{	
	if(self->info.SysInit == RP_NO){
	
		/*等待imu数据收敛切换更低的kp用于控制使用*/
		if(HAL_GetTick() > 1000){

			imu.algo.KP = IMU_PID_KP_CONTROL;
			
			self->info.SysInit = RP_OK;
		}
		
		
	}
	
	
	
	
	if(self->info.MoveMode == MOVE_MASTER){
	
		head.Updata(&head,
		            imu.data.rpy.roll,
		            imu.data.rpy.pitch,
		            imu.data.rpy.yaw,
		            imu.data.worldGyr.x,
		            imu.data.worldGyr.y,
		            imu.data.worldGyr.z);	
	}
	else if(self->info.MoveMode == MOVE_FOLLOW){
		
		head.Updata(&head,
		            0,
		            motor[GIMB_P].rx_info.angle,
		            motor[GIMB_Y].rx_info.angle,
		            imu.data.acc_gyr.gyr_x,
		            imu.data.acc_gyr.gyr_y,
		            imu.data.acc_gyr.gyr_z);	
	}
	
	omni.Updata(&omni);
	gun.Updata(&gun);
	
/*---------------------------------------------------------------*/	
	
	head.Resolving(&head);
	omni.Resolving(&omni);
	
	if(self->info.MoveMode == MOVE_MASTER){
		
		head.Translation(&head,
										 master[M1].data.chasRPY.x,
										 master[M1].data.chasRPY.y,
										 master[M1].data.chasRPY.z);
	}	

/*---------------------------------------------------------------*/	
	
	head.Ctrl(&head);
	omni.Ctrl(&omni);
	
	gun.FrictionCtrl(&gun);
	gun.MagazineCtrl(&gun);
	gun.BoxCtrl(&gun);
	
	
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

	if(self->info.CtrlInit == RP_ING){
	
		Center_CtrlModeInit(self);

	}	
	
	if(self->info.CtrlMode != state){
	
		self->info.CtrlMode = CTRL_NULL;
		self->modifyState(&Center.info.CtrlInit, RP_ING);
	}
}

void Center_ModifiyMoveMode(center *self,Center_MoveMode state)
{
	if(self->info.MoveMode == state)return;

	if(self->info.MoveInit == RP_ING){
	
		Center_MoveModeInit(self);

	}
	
	if(self->info.MoveMode != state){
	
		self->info.MoveMode = MOVE_NULL;
		self->modifyState(&Center.info.MoveInit, RP_ING);		
	}
}

void Center_ModifiyRifleMode(center *self,Center_RifleMode state)
{
	if(self->info.RifleMode == state)return;

	if(self->info.RifleInit == RP_ING){
	
		Center_RifleModeInit(self);

	}		
	
	if(self->info.RifleMode != state){
	
		self->info.RifleMode = RIFLE_NULL;
		self->modifyState(&Center.info.RifleInit, RP_ING);		
	}
}


















