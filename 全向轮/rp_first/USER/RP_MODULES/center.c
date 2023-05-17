/*

ң�������ƣ�
sw1
1����
3��̨ģʽ
2����ģʽ

sw2
ң����ģʽ��
1����
3ֹͣ
2����

����ģʽ��
1��
3��
2����һ��

���ֿ���
ң����ģʽ��
���� Ħ����
ƫ�� ���ո�
��̨ģʽ
���� ����
ƫ�� ���
���� С����

����ģʽ��
���� �����λ
ƫ�� �����λ
ƫ�� �����λ
���� �����λ


*/


#include "center.h"

#include "DEVICES.h"
#include "RP_CONFIG.h"

#include "chassis.h"
#include "gimbal.h"
#include "rifle.h"

#include "string.h"

//0-100
#define ROTATE_VELOCITY_DEFUALT 50

#define IMU_ROLL imu.data.rpy.roll
#define IMU_PITCH imu.data.rpy.pitch
#define IMU_YAW imu.data.rpy.yaw


void Center_ModifiyState(Center_State *self,Center_State state);
void Center_ModifiyCtrlMode(center *self,Center_CtrlMode state);
void Center_ModifiyMoveMode(center *self,Center_MoveMode state);
void Center_ModifiyRifleMode(center *self,Center_RifleMode state);
void Center_ModifiyVisionMode(center *self,Center_VisionMode state);

void Center_Switch(center *self);
void Center_Updata(center *self);
void Center_Ctrl(center *self);

void Remote_Ctrl(center *self);
void KeyMouse_Ctrl(center *self);

center Center = {

	.info.SysInit = RP_NO,
	.info.CtrlInit = RP_NO,	
	.info.MoveInit = RP_NO,	
	.info.RifleInit = RP_NO,	
	.info.VisionInit = RP_NO,	

	.info.RifleLock = RP_NO,
	
	.info.FrictionSwitch = RP_NO,
	.info.MagazineSwitch = RP_NO,
	.info.RotateSwitch   = RP_NO,
	
	.info.CtrlMode   = CTRL_NULL,
	.info.MoveMode   = MOVE_NULL,	
	.info.RifleMode  = RIFLE_NULL,	
	.info.VisionMode = VISION_NULL,
	
	.data.RotateVelocity = ROTATE_VELOCITY_DEFUALT,
	
	.modifyState = Center_ModifiyState,

  .modifyCtrlMode  = Center_ModifiyCtrlMode,
	.modifyMoveMode  = Center_ModifiyMoveMode,
  .modifyRifleMode = Center_ModifiyRifleMode,
	.modifyVisionMode = Center_ModifiyVisionMode,

	.Switch = Center_Switch,
	.Updata = Center_Updata,
	.Ctrl   = Center_Ctrl,
};


#if (RP_CENTER == 1)

/*============================================================================*/

/*
 *@note ��ʼ����
**/
void Center_CtrlModeInit(center *self,Center_CtrlMode state)
{
	rc.data.tw_step[0] = 0;
  rc.data.tw_step[1] = 0;
  rc.data.tw_step[2] = 0;
  rc.data.tw_step[3] = 0;
	
	self->modifyMoveMode(self,MOVE_NULL);
	self->modifyRifleMode(self,RIFLE_NULL);	
	self->modifyVisionMode(self,VISION_NULL);
	
	self->info.FrictionSwitch = RP_NO;
	self->info.MagazineSwitch = RP_NO;
	self->info.RotateSwitch = RP_NO;
	self->info.RifleLock = RP_NO;
	
	self->info.CtrlInit = RP_OK;
}


void Center_MoveModeInit(center *self,Center_MoveMode state)
{
	if(state == MOVE_MASTER){
	
		self->data.GimbPitTarget = imu.data.rpy.pitch;
		self->data.GimbYawTarget = imu.data.rpy.yaw;
		
	}
	else if(state == MOVE_FOLLOW){
	
		self->data.GimbPitTarget = 0;
		self->data.GimbYawTarget = 0;		
		
	}
	
	if(omni.info.Direction == CHAS_FORWARD){

		omni.ModifyOriginAngle(&omni,0);
	}
	else if(omni.info.Direction == CHAS_BACKWARD){
	
		omni.ModifyOriginAngle(&omni,180);
	}
	
	self->modifyVisionMode(self,VISION_NULL);
	self->info.RotateSwitch = RP_NO;
	
	self->info.MoveInit = RP_OK;
	
}

void Center_RifleModeInit(center *self,Center_RifleMode state)
{

	

	self->info.RifleInit = RP_OK;
}

void Center_VisionModeInit(center *self,Center_VisionMode state)
{

	self->data.GimbPitTarget = imu.data.rpy.pitch;
	self->data.GimbYawTarget = imu.data.rpy.yaw;
	
	self->info.VisionInit = RP_OK;
}

/*============================================================================*/


/*============================================================================*/


/*
 *@note ���ƿ�������
**/
void Center_Switch(center *self)
{
	if(rc.info.state == RC_OFFLINE){
		
		self->time.InitTimeStart = HAL_GetTick();
		
		self->info.RifleLock = RP_NO;
		
		self->info.FrictionSwitch = RP_NO;
		self->info.MagazineSwitch = RP_NO;
		
		self->modifyCtrlMode(self,CTRL_NULL);
		self->modifyMoveMode(self,MOVE_NULL);
		self->modifyRifleMode(self,RIFLE_NULL);
		
		gun.ModifyLock(&gun,RIFLE_LOCK);
		head.ModifyLock(&head,GIMB_LOCK);
		omni.ModifyLock(&omni,CHAS_LOCK);

	}
	else{
		
		
		if(self->info.RifleLock == RP_NO){
		
			self->info.RifleLock = RP_ING;
		}
		
		/* ����ģʽ */
		switch(SW1){
		
			case SW_UP:
				self->modifyCtrlMode(self,CTRL_KM);
				break;
			
			default:
				self->modifyCtrlMode(self,CTRL_RC);
				break;		
		}
		
		
		/* ң��ģʽ */
		if(self->info.CtrlMode == CTRL_RC){

			Remote_Ctrl(self);
			
		}
		/* ң��ģʽend */
		
		
		/* ����ģʽ */
		else if(self->info.CtrlMode == CTRL_KM){
		
			self->info.RifleLock = RP_OK;

			KeyMouse_Ctrl(self);
		  
		}
		/* ����ģʽend */
	}
	
	/* ����ģʽend */
}



/*
 *@note �ӿ��أ�ң����
**/
void Remote_Ctrl(center *self)
{
	
	//����ͨ��ֵ
	self->data.Channel[0] = rc.data.ch0;
	self->data.Channel[1] = rc.data.ch1;	
	self->data.Channel[2] = rc.data.ch2;	
	self->data.Channel[3] = rc.data.ch3;	
	
	/* �ƶ�ģʽ */
	switch(SW1){
	
		case SW_MID:
			self->modifyMoveMode(self,MOVE_MASTER);
			break;
		
		case SW_DOWN:
			self->modifyMoveMode(self,MOVE_FOLLOW);
			break;	
	}
	
	/* ��̨ģʽ */
	if(self->info.MoveMode == MOVE_MASTER){
		
		//����С����
		if(rc.data.tw_step[2]){
		
			self->info.RotateSwitch = RP_OK;
		}
		else{
			self->data.RotateVelocity = ROTATE_VELOCITY_DEFUALT;
			self->info.RotateSwitch = RP_NO;
		}
			
		
		if(rc.data.tw_step[3]){

			self->modifyVisionMode(self,VISION_BIG_BUFF);
		}				
		else if(rc.data.thumbwheel < 0 && rc.data.thumbwheel > -300){
		
			if(HAL_GetTick() - self->time.AimEnterTimeStart > 200){
			
				self->modifyVisionMode(self,VISION_AIM);
			}
		}
		else{
		
			self->time.AimEnterTimeStart = HAL_GetTick();
			
			self->modifyVisionMode(self,VISION_NULL);
		}
	}
	/* ����ģʽ */
	else if(self->info.MoveMode == MOVE_FOLLOW){
	
		
	}
	/* �ƶ�ģʽend */
	
					
	/* ����ģʽ */			
	switch(SW2){
	
		case SW_MID:
			self->info.RifleLock = RP_OK;
			self->modifyRifleMode(self,RIFLE_STOP);
			break;
		
		case SW_UP:

			self->modifyRifleMode(self,RIFLE_STAY);
			break;		
		
		case SW_DOWN:

			self->modifyRifleMode(self,RIFLE_SET1);
			break;				
	}
	
	//δ�Է���������и�λ
	if(self->info.RifleLock == RP_NO){
	
		self->modifyRifleMode(self,RIFLE_STOP);
	
	}
	/* ����ģʽend */
	
	/* ���ֿ��� */
	if(rc.data.tw_step[0]){
	
		self->modifyState(&self->info.FrictionSwitch,RP_OK);
	}
	else{
	
		self->modifyState(&self->info.FrictionSwitch,RP_NO);
	}
	
	if(rc.data.tw_step[1]){
	
		self->modifyState(&self->info.MagazineSwitch,RP_NO);
	}
	else{
	
		self->modifyState(&self->info.MagazineSwitch,RP_OK);
	}
	/* ���ֿ���end */
			

}



/*
 *@note �ӿ��أ�����
**/
void KeyMouse_Ctrl(center *self)
{
	//����ͨ��ֵ
	self->data.Channel[0] = rc.data.ch[0];
	self->data.Channel[1] = rc.data.ch[1];
	self->data.Channel[2] = rc.data.ch[2];
	self->data.Channel[3] = rc.data.ch[3];
	





}









void Center_Updata(center *self)
{
	//gimb
	if(self->info.MoveMode == MOVE_MASTER){

		self->data.GimbPitTarget = +(float)self->data.Channel[1]/660.f;
		self->data.GimbYawTarget = -(float)self->data.Channel[0]/660.f;
		
		self->data.GimbPitTarget = RP_Limit(self->data.GimbPitTarget,360);
		self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget,360);
		
	}
	else if(self->info.MoveMode == MOVE_FOLLOW){
	
		self->data.GimbPitTarget = 0;
		self->data.GimbYawTarget = 0;
	}
	head.ModifyXYZSet(&head,NULL,&self->data.GimbPitTarget,&self->data.GimbYawTarget);
	
	//chas
	if(self->info.MoveMode == MOVE_MASTER){

		if(self->info.RotateSwitch == RP_OK){
			
			self->data.VelocityX = (float)self->data.Channel[3]/660*100;
			self->data.VelocityY =-(float)self->data.Channel[2]/660*100;
		  self->data.VelocityZ = self->data.RotateVelocity;
			
		}
		else{
		
			self->data.VelocityX = (float)self->data.Channel[3]/660*100;
			self->data.VelocityY =-(float)self->data.Channel[2]/660*100;
		  self->data.VelocityZ =-(float)self->data.Channel[0]/660*100;
		}
		
	}	
	else if(self->info.MoveMode == MOVE_FOLLOW){
	
		self->data.VelocityX = (float)self->data.Channel[3]/660*100;
		self->data.VelocityY =-(float)self->data.Channel[2]/660*100;
		self->data.VelocityZ =-(float)self->data.Channel[0]/660*100;
		
		//�Դ�ʱ�������Ϊԭ�㣬Ҳ����˵�͵���λ�ò������Ƕ�
		omni.ModifyOriginAngle(&omni,((float)motor[GIMB_Y].rx_info.angle)/22.5f);
	}
	
	//rifle
	if(self->info.FrictionSwitch == RP_OK){
	
		gun.ModifyFri(&gun,RIFLE_OK);
	}
	else if(self->info.FrictionSwitch == RP_NO){
	
		gun.ModifyFri(&gun,RIFLE_NO);
	}

	if(self->info.MagazineSwitch == RP_OK){
	
		gun.ModifyMagazine(&gun,RIFLE_OK);
	}
	else if(self->info.MagazineSwitch == RP_NO){
	
		gun.ModifyMagazine(&gun,RIFLE_NO);
	}

	if(self->info.RifleMode == RIFLE_SET1){
	
		gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,1);
	
	}
	else if(self->info.RifleMode == RIFLE_SET6){
	
		gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,6);
	
	}
	else if(self->info.RifleMode == RIFLE_STAY){
	
		if(gun.data.HeatEnableNum > 5){
		
			gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,4000);		
		}
		else{
		
			gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,2000);		
		}
	}
	else if(self->info.RifleMode == RIFLE_STOP){
	
		gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,0);
	
	}	
}





void Center_Ctrl(center *self)
{
	if(self->info.SysInit == RP_ING){
	
		/*�ȴ�imu���������л����͵�kp���ڿ���ʹ��*/
		if(HAL_GetTick() > 1000){

			imu.algo.KP = IMU_PID_KP_CONTROL;

		}
		
		//ͷ����λ
		head.ModifyLock(&head,GIMB_UNLOCK);
		self->info.MoveMode = MOVE_FOLLOW;
		head.ModifyXYZSet(&head,0,0,0);
		
		if(head.info.YReach == GIMB_OK && head.info.YReach == GIMB_OK){

		  gun.ModifyLock(&gun,RIFLE_UNLOCK);
		  omni.ModifyLock(&omni,CHAS_UNLOCK);
			
			self->info.MoveMode = MOVE_NULL;
			self->info.SysInit = RP_OK;		
		}
		
		
		//��ʱǿ�Ƹ�λ
		if(HAL_GetTick() - self->time.InitTimeStart > 3000){
			
		  gun.ModifyLock(&gun,RIFLE_UNLOCK);
		  omni.ModifyLock(&omni,CHAS_UNLOCK);				
			
			self->info.MoveMode = MOVE_NULL;
			self->info.SysInit = RP_OK;
		}
	}
	
/*------------------------------------------------------------------*/	
	
	
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
		            head.info.AssemblyVector.Y*motor[GIMB_P].rx_info.angle,
		            head.info.AssemblyVector.Z*motor[GIMB_Y].rx_info.angle,
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



/*------------------------------------------------------------------*/	
/*------------------------------------------------------------------*/	




















/*
ģʽ�л�

�Ƚϴ�ʱ��ģʽ������
�����ͬ�򷵻�
�����ͬ����δ���г�ʼ���������ʼ����״̬

�����ʱ���ڳ�ʼ��״̬������ʼ������
��ʼ����ɺ�״̬��Ϊ����ɳ�ʼ��
��ʼ����ɱ�־�������޸Ĵ�ʱ��ģʽ

*/
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
	
		Center_CtrlModeInit(self,state);
		
		if(self->info.CtrlInit == RP_OK){
		
			self->info.CtrlMode = state;
		}
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
	
		Center_MoveModeInit(self,state);

		if(self->info.MoveInit == RP_OK){
		
			self->info.MoveMode = state;
		}		
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
	
		Center_RifleModeInit(self,state);

		if(self->info.RifleInit == RP_OK){
		
			self->info.RifleMode = state;
		}				
	}		
	
	if(self->info.RifleMode != state){
	
		self->info.RifleMode = RIFLE_NULL;
		self->modifyState(&Center.info.RifleInit, RP_ING);		
	}
}

void Center_ModifiyVisionMode(center *self,Center_VisionMode state)
{
	if(self->info.VisionMode == state)return;

	if(self->info.VisionInit == RP_ING){
	
		Center_VisionModeInit(self,state);

		if(self->info.VisionInit == RP_OK){
		
			self->info.VisionMode = state;
		}				
	}		
	
	if(self->info.VisionMode != state){
	
		self->info.VisionMode = VISION_NULL;
		self->modifyState(&Center.info.VisionInit, RP_ING);		
	}
}











#endif




