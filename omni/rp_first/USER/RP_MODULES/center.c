/*

ң�������ƣ�

sw1 = 2|3
3��̨ģʽ
2����ģʽ

	sw2
	1����
	3ֹͣ
	2����

	���ֿ���
	ң����ģʽ��
	���� Ħ����
	ƫ�� ���ո�
	��̨ģʽ
	���� ����
	ƫ�� ���
	���� С����

sw1 = 1
1����-����ģʽ

	sw2
	1����ģʽ
	3����ģʽ1
	2����ģʽ2

	����
	���� ����/�˳�����ģʽ
	ƫ�� �����λ
	ƫ�� �����λ
	���� �����λ

************************************************

�����߼���
	����
		����freertos�������������Դ����
	�������ȼ�ԭ��
		��Ϣ����Ϊ������ȼ�
		����Ϊ������ȼ�
		���Ź�Ϊ������ȼ�
	�ļ���
		������
			�������
			�߼������ͼ�
			ͬ�ȼ��ɰ���
			.h������׼ͷ�ļ����ͼ�ͷ�ļ�
			.c����ͬ��ͷ�ļ����߼�ͷ�ļ���������
		  �߼������ݲ������ڵͼ����߼��޸ĵͼ�����Ҫͨ�����ú����޸ģ�������	
		���ݣ�
			��ģ��Ϊ����
			�豸
			����
			ģ��
			����


ģʽ����:
1.����ͬһ��е�ṹ�������豸
2.��ͬһʱ�䲻��ͬʱִ�е����ֳ���
3.��֮Ϊ����ģʽ

************************************************

ģʽ�л����룺
����ģʽ
����ģʽ�л�������һ��ģʽ�л�����ֻ�ܽ���һ�ֻ������л�
����ȻҲ����д�ɸ��õģ������ܴ�����Դ�໥ʹ�õ����⣩
����ģʽ��ʼ������

�л����̣�
	ʹ���л�����
	�Ƚ�Ŀ��ģʽ�͵�ǰģʽ

		��ͬ��
		����

		��ͬ��
		��ǰģʽΪNULL
		��ʼ����־λ����
		���г�ʼ��
		��ʼ�����
		�л�Ŀ��ģʽ
	�����ڳ�ʼ����ֻ����Ŀ��ģʽ������Ҫ�ĳ�ʼ��������������һģʽ�����
		
ִ�У�
	ִֻ�е�ǰģʽ
	ΪNULLʱ��ִ��
	(����ִ�����ݺͳ�ʼ�����ݴ��ڽ�������ִ������д�ڳ�ʼ��
	
ģ���д˼·��
	��Ҫ�����Ĳ���
	1�����ݸ���
	2������
	3������
	4���޸Ĳ���API
	��Ҫ��ͨ��API�������п��ƣ���������һֱ����
	
	
	

	

*/

#include "center.h"


#include "RP_CONFIG.h"
#include "DEVICES.h"
#include "RP_FUNCTION.h"

#include "chassis.h"
#include "gimbal.h"
#include "rifle.h"


//0-100

#define IMU_INIT_TIME           2000
#define CENTER_INIT_OUT_TIME    3000

extern float yaw_imu_out_pid_param[7];
extern float yaw_imu_inn_pid_param[7];
extern float pit_imu_out_pid_param[7];
extern float pit_imu_inn_pid_param[7];

extern float yaw_mec_out_pid_param[7];
extern float yaw_mec_inn_pid_param[7];
extern float pit_mec_out_pid_param[7];
extern float pit_mec_inn_pid_param[7];


static void Center_ModifiyState(Center_State *self,Center_State state);

static void Center_ModifiyCtrlMode(center *self,Center_CtrlMode state);
static void Center_ModifiyMoveMode(center *self,Center_MoveMode state);
static void Center_ModifiyRifleMode(center *self,Center_RifleMode state);
static void Center_ModifiyVisionMode(center *self,Center_VisionMode state);

/*����*/
static Center_State Center_Switch(center *self);
static Center_State Remote_Ctrl(center *self);
static Center_State KeyMouse_Ctrl(center *self);

static Center_State Center_ModeInit(center *self);
static Center_State Center_Updata(center *self);
static Center_State Center_Ctrl(center *self);



center Center = {

	.info.SysInit    = RP_NO,
	.info.CtrlInit   = RP_NO,	
	.info.MoveInit   = RP_NO,	
	.info.RifleInit  = RP_NO,	
	.info.VisionInit = RP_NO,	

	.info.RifleLock = RP_NO,
	
	.info.FrictionSwitch = RP_NO,
	.info.MagazineSwitch = RP_NO,
	.info.RotateSwitch   = RP_NO,
	
	.info.AutoFireSwitch = RP_NO,
	.info.AutoGimbSwitch = RP_NO,	
	
	
	.info.CtrlMode   = CTRL_NULL,
	.info.MoveMode   = MOVE_NULL,	
	.info.RifleMode  = RIFLE_NULL,	
	.info.VisionMode = VISION_NULL,
	
	.info.MoveCommand = RP_NO,
	.info.MoveCrazzy  = RP_NO,
	
	.info.ErrDevices = 0,
	
	.time.InitTimeStart     = 0,
	.time.AimEnterTimeStart = 0,
	.time.CenterStartTime   = 0,
	
	.data.RotateVelocity = ROTATE_VELOCITY_DEFUALT,
	
	.modifyState = Center_ModifiyState,

  .modifyCtrlMode   = Center_ModifiyCtrlMode,
	.modifyMoveMode   = Center_ModifiyMoveMode,
  .modifyRifleMode  = Center_ModifiyRifleMode,
	.modifyVisionMode = Center_ModifiyVisionMode,

	.Switch = Center_Switch,
	.ModeInit = Center_ModeInit,
	.Updata = Center_Updata,
	.Ctrl   = Center_Ctrl,
};



/*============================================================================*/

/*
 *@note ��ʼ����
**/
Center_State Center_SysInit(center *self)
{
	if(HAL_GetTick() < 1000){
	
		imu.algo.KP = 5;
		
		return RP_ING;
	}
	else{
	
		imu.algo.KP = 0.1f;
	  
	}

	
	if(head.info.Lock == GIMB_LOCK){
	
		self->time.GimbalReachTime = HAL_GetTick();
		
		head.ModifyLock(&head,GIMB_UNLOCK);
		
	}

	self->modifyMoveMode(self,MOVE_FOLLOW);
	
	//ͷ����λ
	head.Updata(&head,0,
							head.info.AssemblyVector.Y*motor[GIMB_P].rx_info.angle_offset/22.755f,
							head.info.AssemblyVector.Z*motor[GIMB_Y].rx_info.angle_offset/22.755f,
							imu.data.acc_gyr.gyr_x,
							imu.data.acc_gyr.gyr_y,
							imu.data.acc_gyr.gyr_z);
	
	self->data.GimbYawTarget = 0;
	self->data.GimbPitTarget = 0;
	
	head.ModifyXYZSet(&head,0,self->data.GimbPitTarget,self->data.GimbYawTarget);
	
	if(head.info.YReach == GIMB_OK && head.info.ZReach == GIMB_OK){

		if(HAL_GetTick() - self->time.GimbalReachTime > 500){
		
			self->info.SysInit  = RP_OK;
		}
	}
	else{

		self->time.GimbalReachTime = HAL_GetTick();
	}
	
	//��ʱǿ�Ƹ�λ��̨
	if(HAL_GetTick() - self->time.InitTimeStart > CENTER_INIT_OUT_TIME){
		
		self->info.SysInit  = RP_OK;
	}
	
	//��λ�ɹ� ������һ������
	if(self->info.SysInit == RP_OK){
	
		gun.ModifyLock(&gun,RIFLE_UNLOCK);
		
		omni.ModifyLock(&omni,CHAS_UNLOCK);				
		
		self->modifyMoveMode(self,MOVE_NULL);

	}
	
	return RP_OK;
}



void Center_CtrlModeInit(center *self,Center_CtrlMode state)
{
	rc.data.tw_step[0] = 0;
  rc.data.tw_step[1] = 0;
  rc.data.tw_step[2] = 0;
  rc.data.tw_step[3] = 0;

	self->modifyMoveMode(self,MOVE_MASTER);
	
	self->modifyRifleMode(self,RIFLE_NULL);	
	
	self->modifyVisionMode(self,VISION_NULL);
	
	self->info.RifleLock      = RP_NO;	
	
	self->info.FrictionSwitch = RP_NO;
	
	self->info.RotateSwitch   = RP_NO;
	
	self->info.MagazineSwitch = RP_NO;
	
	self->info.MoveCrazzy = RP_OK;

	self->info.CtrlInit = RP_OK;
}


void Center_MoveModeInit(center *self,Center_MoveMode state)
{
	/* rc */	
	rc.data.tw_step[0] = 0;
  rc.data.tw_step[1] = 0;
  rc.data.tw_step[2] = 0;
  rc.data.tw_step[3] = 0;

	/* gimb */
	motor[GIMB_Y].pid.angle.info.init_flag    = M_DEINIT;
  motor[GIMB_Y].pid.angle_in.info.init_flag = M_DEINIT;
	
	motor[GIMB_P].pid.angle.info.init_flag    = M_DEINIT;
	motor[GIMB_P].pid.angle_in.info.init_flag = M_DEINIT;

	if(state == MOVE_MASTER){
		
		motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.angle,   yaw_imu_out_pid_param);
		motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.angle_in,yaw_imu_inn_pid_param);	

		motor[GIMB_P].pid_init(&motor[GIMB_P].pid.angle,   pit_imu_out_pid_param);
		motor[GIMB_P].pid_init(&motor[GIMB_P].pid.angle_in,pit_imu_inn_pid_param);			
		
		self->data.GimbPitTarget = RP_Limit(imu.data.rpy.pitch,360);
		self->data.GimbYawTarget = RP_Limit(imu.data.rpy.yaw,360);
		
	}
	else if(state == MOVE_FOLLOW){

		motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.angle,   yaw_mec_out_pid_param);
		motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.angle_in,yaw_mec_inn_pid_param);	

		motor[GIMB_P].pid_init(&motor[GIMB_P].pid.angle,   pit_mec_out_pid_param);
		motor[GIMB_P].pid_init(&motor[GIMB_P].pid.angle_in,pit_mec_inn_pid_param);	
				
		self->data.GimbPitTarget = RP_Limit(head.info.AssemblyVector.Y * motor[GIMB_P].rx_info.angle_offset/22.755f,360);
		self->data.GimbYawTarget = (omni.info.Direction == CHAS_FORWARD ? 0 : 180);
		
	}
	
	/* chas */
	if(omni.info.Direction == CHAS_FORWARD){

		omni.ModifyOriginAngle(&omni,0);
		omni.ModifyDireMaster(&omni,omni.info.Direction);
	}
	else if(omni.info.Direction == CHAS_BACKWARD){
	
		omni.ModifyOriginAngle(&omni,180);
		omni.ModifyDireMaster(&omni,omni.info.Direction);
	}
	
	omni.ModifyDistribute(&omni,CHAS_FAIR);
	
	self->modifyVisionMode(self,VISION_NULL);
	
	self->info.RotateSwitch = RP_NO;
	
	self->info.FastTurn = RP_NO;

	self->info.MoveInit = RP_OK;
	
}

void Center_RifleModeInit(center *self,Center_RifleMode state)
{

	

	self->info.RifleInit = RP_OK;
}

void Center_VisionModeInit(center *self,Center_VisionMode state)
{
	if(state == MOVE_MASTER){
	
		self->data.GimbPitTarget = RP_Limit(imu.data.rpy.pitch,360);
		self->data.GimbYawTarget = RP_Limit(imu.data.rpy.yaw,360);
		
	}
	else if(state == MOVE_FOLLOW){
	
		self->data.GimbPitTarget = head.info.AssemblyVector.Y * motor[GIMB_P].rx_info.angle_offset/22.755f;
		
		if(omni.info.Direction == CHAS_FORWARD){

			self->data.GimbYawTarget = 0;
		}
		else if(omni.info.Direction == CHAS_BACKWARD){
		
			self->data.GimbYawTarget = 180;
		}
	}
	
	if(omni.info.Direction == CHAS_FORWARD){

		omni.ModifyOriginAngle(&omni,0);
	}
	else if(omni.info.Direction == CHAS_BACKWARD){
	
		omni.ModifyOriginAngle(&omni,180);
	}
	
	//Ĭ�Ͽ����������
	if(state != VISION_NULL){

		self->modifyState(&self->info.AutoFireSwitch,RP_OK);
	}
	
	self->info.VisionInit = RP_OK;
}


Center_State Center_ModeInit(center *self)
{
  /* init handle */
	if(self->info.CtrlInit == RP_ING){
	
		Center_CtrlModeInit(self,self->info.CtrlModeTar);
		
		if(self->info.CtrlInit == RP_OK){
		
			self->info.CtrlMode = self->info.CtrlModeTar;
		}
	}

	if(self->info.MoveInit == RP_ING){
	
		Center_MoveModeInit(self,self->info.MoveModeTar);
		
		if(self->info.MoveInit == RP_OK){
		
			self->info.MoveMode = self->info.MoveModeTar;
		}
	}

	if(self->info.RifleInit == RP_ING){
	
		Center_RifleModeInit(self,self->info.RifleModeTar);
		
		if(self->info.RifleInit == RP_OK){
		
			self->info.RifleMode = self->info.RifleModeTar;
		}
	}
	
	if(self->info.CtrlInit == RP_ING){
	
		Center_VisionModeInit(self,self->info.VisionModeTar);
		
		if(self->info.CtrlInit == RP_OK){
		
			self->info.VisionMode = self->info.VisionModeTar;
		}
	}	
	
	return RP_OK;
}



/*============================================================================*/


/*============================================================================*/


/*
 *@note ���ƿ������� �޸�Ŀ��״̬
**/

Center_State Center_Switch(center *self)
{
	
	if(self->time.CenterStartTime == 0){
	
		self->time.CenterStartTime = HAL_GetTick()+1;
		
	}
	
	if(rc.info.state == RC_OFFLINE){
		
		self->time.InitTimeStart = HAL_GetTick();
		
//		memset(&self->info,0,sizeof(self->info));
//		gun.ModifyLock(&gun,RIFLE_LOCK);
//		head.ModifyLock(&head,GIMB_LOCK);
//		omni.ModifyLock(&omni,CHAS_LOCK);

		self->info.SysInit    = RP_NO;
		self->info.CtrlInit   = RP_NO;
		self->info.MoveInit   = RP_NO;		
		self->info.RifleInit  = RP_NO;
		self->info.VisionInit = RP_NO;
		
		self->info.RifleLock = RP_NO;
		
		self->info.FrictionSwitch = RP_NO;
		self->info.MagazineSwitch = RP_NO;
		self->info.RotateSwitch   = RP_NO;
		self->info.AutoFireSwitch = RP_NO;
		
		self->modifyCtrlMode(self,CTRL_NULL);
		self->modifyMoveMode(self,MOVE_NULL);
		self->modifyRifleMode(self,RIFLE_NULL);
		
		gun.ModifyLock(&gun,RIFLE_LOCK);
		head.ModifyLock(&head,GIMB_LOCK);
		omni.ModifyLock(&omni,CHAS_LOCK);
		
		
		
	}
	else{
		
		if(self->info.SysInit == RP_NO){
		
			self->info.SysInit = RP_ING;
			
			return RP_ING;
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
	
	return RP_OK;
}



/*
 *@note �ӿ��أ�ң����
**/
Center_State Remote_Ctrl(center *self)
{
	
	//����ͨ��ֵ
	self->data.Channel[0] = (float)rc.data.ch0/660.f*100;
	self->data.Channel[1] = (float)rc.data.ch1/660.f*100;
	self->data.Channel[2] = (float)rc.data.ch2/660.f*100;	
	self->data.Channel[3] = (float)rc.data.ch3/660.f*100;	
	
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

		gun.ModifyState(&gun.info.AutoSetEnable,RIFLE_NO);
		
		//����С����
		if(rc.data.tw_step[2]){
		
			self->modifyState(&self->info.RotateSwitch,RP_OK);
		}
		else{
			self->data.RotateVelocity = ROTATE_VELOCITY_DEFUALT;
		
      self->modifyState(&self->info.RotateSwitch,RP_NO);
		}

		//�����Ӿ�
		if(rc.data.tw_step[3]){

			self->modifyVisionMode(self,VISION_BIG_BUFF);
		}
		else if(rc.data.thumbwheel < 20 && rc.data.thumbwheel > 200){
		
			if(HAL_GetTick() - self->time.AimEnterTimeStart > 200){
			
				self->modifyVisionMode(self,VISION_AIM);
			}
		}
		else{
			
			self->modifyVisionMode(self,VISION_NULL);
			
			self->info.AutoFireSwitch = RP_NO;
			
			self->info.AutoGimbSwitch = RP_NO;
			
			self->time.AimEnterTimeStart = HAL_GetTick();			
		}
	}
	/* ����ģʽ */
	else if(self->info.MoveMode == MOVE_FOLLOW){

		gun.ModifyState(&gun.info.AutoSetEnable,RIFLE_OK);
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
	
	if(self->info.FrictionSwitch == RP_NO){
	
		self->info.RifleLock = RP_NO;
	}
	
	/* �������δ�˳����� */
	if(self->info.RifleLock == RP_NO){
	
		self->modifyRifleMode(self,RIFLE_STOP);
	}
	
	/* ����ģʽend */
	
	/* ���ֿ��� */
	
	//Ħ����
	if(rc.data.tw_step[0]){
	
		self->modifyState(&self->info.FrictionSwitch,RP_OK);
	}
	else{
	
		self->modifyState(&self->info.FrictionSwitch,RP_NO);
	}
	//����
	if(rc.data.tw_step[1]){
	
		self->modifyState(&self->info.MagazineSwitch,RP_OK);
	}
	else{
	
		self->modifyState(&self->info.MagazineSwitch,RP_NO);
	}
	/* ���ֿ���end */

	
	return RP_OK;
}



/*
 * @note �ӿ��أ�����
**/
Center_State KeyMouse_Ctrl(center *self)
{
	//����ͨ��ֵ
	self->data.Channel[0] = rc.data.ch[0];
	self->data.Channel[1] = rc.data.ch[1];
	self->data.Channel[2] = rc.data.ch[2]/660.f*100;
	self->data.Channel[3] = rc.data.ch[3]/660.f*100;
	
	
	/* ����ת�� */

  switch(rc.keyMouse.Q.State)
  {
    case UP: 
      break;
    case PRESS: 
			if(self->info.FastTurnType == FT_NULL){
			
				self->info.FastTurnType = FT_L90;
				self->info.FastTurn = RP_OK;
			}
		
      break;
    case SHORT_DOWN: 		
      break;
    case DOWN: 
      break;
    case RELAX: 
      break;
  }
	
  switch(rc.keyMouse.E.State)
  {
    case UP: 
      break;
    case PRESS: 
			if(self->info.FastTurnType == FT_NULL){
			
				self->info.FastTurnType = FT_R90;
				self->info.FastTurn = RP_OK;
			}
      break;
    case SHORT_DOWN: 		
      break;
    case DOWN: 
      break;
    case RELAX: 
      break;
  }	
	
  switch(rc.keyMouse.C.State)
  {
    case UP: 
      break;
    case PRESS: 
			if(self->info.FastTurnType == FT_NULL){
			
				self->info.FastTurnType = FT_180;
				self->info.FastTurn = RP_OK;
			}			

      break;
    case SHORT_DOWN: 		
      break;
    case DOWN: 
      break;
    case RELAX: 
      break;
  }	
	
	/* С���� */
  switch(rc.keyMouse.F.State)
  {
    case UP: 
      break;
    case PRESS: 
			self->data.RotateVelocity = ROTATE_VELOCITY_DEFUALT;
      self->modifyState(&self->info.RotateSwitch,RP_OK);	
      break;
		
    case SHORT_DOWN: 		
      break;
    case DOWN: 
      break;
    case RELAX: 
      break;
  }
	
	/* ���� */
  switch(rc.keyMouse.B.State)
  {
    case UP: 
      break;
    case PRESS: 
      self->modifyState(&self->info.MagazineSwitch,RP_OK);	
      self->modifyState(&self->info.FrictionSwitch,RP_NO);
      break;
		
    case SHORT_DOWN: 		
			self->modifyState(&self->info.MagazineSwitch,RP_OK);	
      break;
    case DOWN: 
			self->modifyState(&self->info.MagazineSwitch,RP_OK);	
      break;
    case RELAX: 
      break;
  }	
	
	/* ������ */
  switch(rc.keyMouse.R.State)
  {
    case UP: 
      break;
    case PRESS: 
			self->modifyRifleMode(self,RIFLE_SET6);
      break;
    case SHORT_DOWN: 		

      break;
    case DOWN: 
	
      break;
    case RELAX: 
      break;
  }
	
	/* ��λ */
  switch(rc.keyMouse.CTRL.State)
  {
    case UP: 

      break;
		
    case PRESS: 
			self->modifyState(&self->info.MagazineSwitch,RP_NO);
		
		  self->modifyState(&self->info.RotateSwitch,RP_NO);
		
		  omni.ModifyOriginAngle(&omni,0);
      break;
		
    case SHORT_DOWN:
			self->modifyMoveMode(self,MOVE_MASTER);
      break;
		
    case DOWN: 
	    self->modifyMoveMode(self,MOVE_FOLLOW);
      break;
		
    case RELAX: 
      break;
  }	
	
	/* ��� */
  switch(rc.keyMouse.Mouse_L.State)
  {
    case UP: 
      self->modifyRifleMode(self,RIFLE_STOP);
      break;
		
    case PRESS: 
			if(self->info.MagazineSwitch == RP_OK){
			
				self->modifyState(&self->info.MagazineSwitch,RP_NO);
			}

			else if(self->info.FrictionSwitch == RP_NO){
			
				self->modifyState(&self->info.FrictionSwitch,RP_OK);
			}
		  else{
				
        self->modifyRifleMode(self,RIFLE_SET1);			
			}
      break;
		
    case SHORT_DOWN: 	
      break;
		
    case DOWN: 
			self->modifyRifleMode(self,RIFLE_STAY);			
      break;
    case RELAX: 
      break;
  }		
	
	return RP_OK;
	
}














/*------------------------------------------------------------------*/	


/*
 * ����Ŀ��ģʽ ���õײ�API�����޸�״̬
**/


/* ---------------------------Center_Updata��-------------------------- */

void Center_StateUpdata(center *self)
{
	/* �ƶ����� */
	if(abs(self->data.Channel[0]) > 10 || abs(self->data.Channel[2]) > 10 || abs(self->data.Channel[3]) > 10){
	
		self->info.MoveCommand = RP_OK;
	}
	else{
	
		self->info.MoveCommand = RP_NO;
	}
	
	if(abs(self->data.Channel[2]) > 50 || abs(self->data.Channel[3]) > 50){
		
		self->info.MoveXYCommand = RP_OK;
	}
	else{
	
		self->info.MoveXYCommand = RP_NO;
	}	
	
  /* ����ʹ�ò��� */
	//���
	if(self->info.MoveCommand == RP_OK){
	
		cap.modifyLimit(&cap,cap.data.tx.output_power_limit,0);
	}
	else if(self->info.MoveCommand == RP_NO){
	
		cap.modifyLimit(&cap,cap.data.tx.output_power_limit,150);
	}
	
	//�ŵ�
	if(self->info.MoveCrazzy == RP_OK){
	
		cap.modifyLimit(&cap,150,cap.data.tx.input_power_limit);
		omni.ModifyrpmMax(&omni,WHEEL_SPEED_MAX);
		
	}
	else if(self->info.MoveCrazzy == RP_NO){
	
		cap.modifyLimit(&cap,75,cap.data.tx.input_power_limit);
		omni.ModifyrpmMax(&omni,WHEEL_POWER_SPEED_MAX);
	}
}




void Center_GimbalStrategy(center *self)
{
	//��̨����
	if(self->info.MoveMode == MOVE_MASTER){
		
    //����ת��ֵ
		if(self->info.FastTurn == RP_OK){
		
			switch((uint8_t)self->info.FastTurnType){
			
				case FT_R90:
					self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget - 90,360);
					break;
			
				case FT_L90:
					self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget + 90,360);
					break;

				case FT_R45:
					self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget - 45,360);
					break;

				case FT_L45:
					self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget + 45,360);
					break;		
				
				case FT_180:
					self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget + 180,360);
					break;					
			}

			self->info.FastTurn = RP_ING;
			
			self->time.FTStartTime = HAL_GetTick();
		}
		else if(self->info.FastTurn == RP_ING){
		
			if(head.info.YReach == GIMB_OK){
			
				self->info.FastTurn = RP_NO;
			}
			
			if(HAL_GetTick() - self->time.FTStartTime > 1000){
			
				self->info.FastTurn = RP_NO;
			}
			
			if(self->info.FastTurn == RP_NO){
			
				self->info.FastTurnType = FT_NULL;
			}
		}
		//����ת��
		else{
		
			if(anti_constrain(RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360),head.info.depression,360 - head.info.elevation)){
			
				self->data.GimbPitTarget = RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360);
			}

			if(abs(head.data.AngleErr.Z) <= 90){
						
				self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget - (float)self->data.Channel[0]/100,360);
			}
		}
	}
	//���̸���
	else if(self->info.MoveMode == MOVE_FOLLOW){

		if(anti_constrain(RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360),head.info.depression,360 - head.info.elevation)){
		
			self->data.GimbPitTarget  = RP_Limit(self->data.GimbPitTarget+(float)self->data.Channel[1]/100,360);
		}
	}
	
	
	//ǿ����̨��ƽ
	if(self->info.MagazineSwitch == RP_OK){
	
		self->data.GimbPitTarget = 0;
	}

	//pit��λ
	if(RP_Limit(self->data.GimbPitTarget,360) > 0 && RP_Limit(self->data.GimbPitTarget,360) < 180){
	
		self->data.GimbPitTarget = (self->data.GimbPitTarget > head.info.depression ? head.info.depression : self->data.GimbPitTarget);
	}
	else if(RP_Limit(self->data.GimbPitTarget,360) > 180 && RP_Limit(self->data.GimbPitTarget,360) < 360){
	
		self->data.GimbPitTarget = (self->data.GimbPitTarget < 360 - head.info.elevation ? 360 - head.info.elevation : self->data.GimbPitTarget);
	}

	//��ֵ
	head.ModifyXYZSet(&head,0,self->data.GimbPitTarget,self->data.GimbYawTarget);
}



void Center_MoveStrategy(center *self)
{	
	float angle;	
	
	//�ƶ�����
	if(self->info.MoveXYCommand == RP_OK){
	
		omni.ModifyDistribute(&omni,CHAS_LINEAR);
	}			
	else{
	
		omni.ModifyDistribute(&omni,CHAS_ROTATE);
	}
	
	//��̨ģʽ��ֵ
	if(self->info.MoveMode == MOVE_MASTER){

		if(self->info.RotateSwitch == RP_OK){
			
			self->data.VelocityX = (float)self->data.Channel[3];
			self->data.VelocityY =-(float)self->data.Channel[2];
		  self->data.VelocityZ = self->data.RotateVelocity;
			
			//С���ݹ�������Ӧ
			if(omni.data.PowerBuff > 40){
			
				self->data.RotateVelocity = (self->data.RotateVelocity >= 100 ? 100 : self->data.RotateVelocity+0.005f);
			}
			else{
			
				self->data.RotateVelocity = (self->data.RotateVelocity <= 30 ? 30 : self->data.RotateVelocity-0.01f);
			}
		}
		else{
		
			angle = omni.data.DirAngle;
			angle = RP_HalfTurn(angle,360);
			
			self->data.VelocityX =  (float)self->data.Channel[3];
			self->data.VelocityY = -(float)self->data.Channel[2];
		  self->data.VelocityZ =  angle/(180)*80;
		}
	}
	//����ģʽ��ֵ
	else if(self->info.MoveMode == MOVE_FOLLOW){
	
		self->data.VelocityX = (float)self->data.Channel[3];
		self->data.VelocityY =-(float)self->data.Channel[2];
		self->data.VelocityZ =-(float)self->data.Channel[0];
		
		//�Դ�ʱ�������Ϊԭ�㣬Ҳ����˵�͵���λ�ò������Ƕ�
		omni.ModifyOriginAngle(&omni,((float)motor[GIMB_Y].rx_info.angle_offset)/22.755f);
	}
	
	omni.ModifyXYZSet(&omni,self->data.VelocityX,self->data.VelocityY,self->data.VelocityZ);
		
}


void Center_ShootStrategy(center *self)
{
	//Ħ����
	if(self->info.FrictionSwitch == RP_OK){
	
		gun.ModifyFri(&gun,RIFLE_OK);
	}
	else if(self->info.FrictionSwitch == RP_NO){
	
		gun.ModifyFri(&gun,RIFLE_NO);
	}

	//���ָ�
	if(self->info.MagazineSwitch == RP_OK){
	
		gun.ModifyMagazine(&gun,RIFLE_OK);
	}
	else if(self->info.MagazineSwitch == RP_NO){
	
		gun.ModifyMagazine(&gun,RIFLE_NO);
	}

	//����ģʽ����
	switch((uint8_t)self->info.RifleMode){
	
		case RIFLE_SET1:
			gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,1);
			break;
		
		case RIFLE_SET6:
			gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,6);
			break;		
		
		case RIFLE_STAY:
			//��������
			if(gun.data.HeatEnableNum > 5){
			
				gun.ModifyShootType(&gun,RIFLE_SHOOT_STAY,4000);		
			}
			else{
			
				gun.ModifyShootType(&gun,RIFLE_SHOOT_STAY,2000);		
			}
			break;
			
		case RIFLE_STOP:
			gun.ModifyShootType(&gun,RIFLE_SHOOT_STOP,0);					
			break;	
	}
}
/* ---------------------------Center_Updata��-------------------------- */


Center_State Center_Updata(center *self)
{
  //Init
	if(self->info.SysInit == RP_ING){
	
		Center_SysInit(self);
		
		return RP_ING;
	}

	Center_StateUpdata(self);
	
	Center_GimbalStrategy(self);
	
	Center_MoveStrategy(self);
	
	Center_ShootStrategy(self);

	return RP_OK;
}


	
/*-------------------------------------------*/
	  

/*
 * ִ�еײ�ģ��
**/
Center_State Center_Ctrl(center *self)
{

//	uint16_t ledtime = 0;
//	
//	if(omni.info.MotorState == CHAS_ERR){
//	
//		ledtime++;
//	}
//	if(head.info.MotorState == GIMB_MOTOR_ERR){
//	
//		ledtime++;
//	}
//	if(gun.info.MotorState == RIFLE_MOTOR_ERR){
//	
//		ledtime++;
//	}
//	if(judge.info.state == JUDGE_OFFLINE){
//	
//		ledtime++;
//	}
//	if(vision.info.state == VISION_OFFLINE){
//	
//		ledtime++;
//	}
//	if(cap.info.state == CAP_OFFLINE){
//	
//		ledtime++;
//	}
//	
//	self->info.ErrDevices = ledtime;
//	
//	
//	
//	if(ledtime == 0){
//		
//		led.running(50);
//	}
//	else{
//		
//		led.allShine(ledtime * 100);
//	}

/*-------------------------------------------*/	
//	
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
								head.info.AssemblyVector.Y*motor[GIMB_P].rx_info.angle_offset/22.755f,
								head.info.AssemblyVector.Z*motor[GIMB_Y].rx_info.angle_offset/22.755f,
								imu.data.acc_gyr.gyr_x,
								imu.data.acc_gyr.gyr_y,
								imu.data.acc_gyr.gyr_z);		
		
	}
	
	omni.Updata(&omni);
	gun.Updata(&gun);

/*---------------------------------------------------------------*/	
	
	head.Resolving(&head);
	omni.Resolving(&omni);
	gun.Resolving(&gun);
	
	//��̬�ط���
//	if(self->info.MoveMode == MOVE_MASTER){
//		
//		head.Translation(&head,
//										 master[M1].data.chasRPY.x,
//										 master[M1].data.chasRPY.y,
//										 master[M1].data.chasRPY.z);
//	}

/*---------------------------------------------------------------*/	
	
	head.Ctrl(&head);
	omni.Ctrl(&omni);
	gun.Ctrl(&gun);

	return RP_OK;
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

	if(self->info.CtrlInit != RP_ING){
	
		if(self->info.CtrlMode != state){
		
			self->info.CtrlModeTar = state;
			self->info.CtrlMode = CTRL_NULL;
			self->modifyState(&Center.info.CtrlInit, RP_ING);
		}	
	}
}

void Center_ModifiyMoveMode(center *self,Center_MoveMode state)
{
	if(self->info.MoveMode == state)return;	
	
	if(self->info.MoveInit != RP_ING){
	
		if(self->info.MoveMode != state){
		
			self->info.MoveModeTar = state;
			self->info.MoveMode = MOVE_NULL;
			self->modifyState(&Center.info.MoveInit, RP_ING);		
		}		
	}
}

void Center_ModifiyRifleMode(center *self,Center_RifleMode state)
{
	if(self->info.RifleMode == state)return;	
	
	if(self->info.RifleInit != RP_ING){
	
		if(self->info.RifleMode != state){
		
			self->info.RifleModeTar = state;
			self->info.RifleMode = RIFLE_NULL;
			self->modifyState(&Center.info.RifleInit, RP_ING);		
		}
	}
}

void Center_ModifiyVisionMode(center *self,Center_VisionMode state)
{
	if(self->info.VisionMode == state)return;	
	
	if(self->info.VisionInit != RP_ING){
		
		if(self->info.VisionMode != state){
		
			self->info.VisionModeTar = state;
			self->info.VisionMode = VISION_NULL;
			self->modifyState(&Center.info.VisionInit, RP_ING);		
		}
	}
}



























	
#if 0	
	float angle;

	/* �ƶ����� */
	if(abs(self->data.Channel[0]) > 10 || abs(self->data.Channel[2]) > 10 || abs(self->data.Channel[3]) > 10){
	
		self->info.MoveCommand = RP_OK;
	}
	else{
	
		self->info.MoveCommand = RP_NO;
	}
	
	if(abs(self->data.Channel[2]) > 10 || abs(self->data.Channel[3]) > 10){
	
		self->info.MoveXYCommand = RP_OK;
	}
	else{
	
		self->info.MoveXYCommand = RP_NO;
	}	
	
	//�ƶ�����
	if(self->info.MoveXYCommand == RP_OK){
	
		omni.ModifyDistribute(&omni,CHAS_LINEAR);
	}			
	else{
	
		omni.ModifyDistribute(&omni,CHAS_ROTATE);
	}
	
	//gimb
	if(RP_Limit(self->data.GimbPitTarget,360) > 0 && RP_Limit(self->data.GimbPitTarget,360) < 180){
	
		self->data.GimbPitTarget = (self->data.GimbPitTarget > head.info.depression ? head.info.depression : self->data.GimbPitTarget);
	}
	else if(RP_Limit(self->data.GimbPitTarget,360) > 180 && RP_Limit(self->data.GimbPitTarget,360) < 360){
	
		self->data.GimbPitTarget = (self->data.GimbPitTarget < 360 - head.info.elevation ? 360 - head.info.elevation : self->data.GimbPitTarget);
	}
	
	
	if(self->info.MoveMode == MOVE_MASTER){
		
		if(anti_constrain(RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360),head.info.depression,360 - head.info.elevation)){
		
			self->data.GimbPitTarget  = RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360);
		}
			
		self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget - (float)self->data.Channel[0]/100,360);

	}
	else if(self->info.MoveMode == MOVE_FOLLOW){
	
		if(anti_constrain(RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360),head.info.depression,360 - head.info.elevation)){
		
			self->data.GimbPitTarget  = RP_Limit(self->data.GimbPitTarget+(float)self->data.Channel[1]/100,360);
		}
	}

	head.ModifyXYZSet(&head,0,self->data.GimbPitTarget,self->data.GimbYawTarget);

	//chas
	if(self->info.MoveMode == MOVE_MASTER){

		if(self->info.RotateSwitch == RP_OK){
			
			self->data.VelocityX = (float)self->data.Channel[3];
			self->data.VelocityY =-(float)self->data.Channel[2];
		  self->data.VelocityZ = self->data.RotateVelocity;
			
			//С���ݹ�������Ӧ
			if(omni.data.PowerBuff > 40){
			
				self->data.RotateVelocity = (self->data.RotateVelocity >= 100 ? 100 : self->data.RotateVelocity+0.005f);
			}
			else{
			
				self->data.RotateVelocity = (self->data.RotateVelocity <= 30 ? 30 : self->data.RotateVelocity-0.01f);
			}
		}
		else{
		
			angle = omni.data.DirAngle;
			angle = RP_HalfTurn(angle,360);
			
			self->data.VelocityX =  (float)self->data.Channel[3];
			self->data.VelocityY = -(float)self->data.Channel[2];
		  self->data.VelocityZ =  angle/(180)*80;
		}
	}
	else if(self->info.MoveMode == MOVE_FOLLOW){
	
		self->data.VelocityX = (float)self->data.Channel[3];
		self->data.VelocityY =-(float)self->data.Channel[2];
		self->data.VelocityZ =-(float)self->data.Channel[0];
		
		//�Դ�ʱ�������Ϊԭ�㣬Ҳ����˵�͵���λ�ò������Ƕ�
		omni.ModifyOriginAngle(&omni,((float)motor[GIMB_Y].rx_info.angle_offset)/22.755f);
	}
	
	omni.ModifyXYZSet(&omni,self->data.VelocityX,self->data.VelocityY,self->data.VelocityZ);
	
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

	switch((uint8_t)self->info.RifleMode){
	
		case RIFLE_SET1:
			gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,1);
			break;
		
		case RIFLE_SET6:
			gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,6);
			break;		
		
		case RIFLE_STAY:
			if(gun.data.HeatEnableNum > 5){
			
				gun.ModifyShootType(&gun,RIFLE_SHOOT_STAY,4000);		
			}
			else{
			
				gun.ModifyShootType(&gun,RIFLE_SHOOT_STAY,2000);		
			}
			break;	
			
		case RIFLE_STOP:
			gun.ModifyShootType(&gun,RIFLE_SHOOT_STOP,0);					
			break;	
	}
	
	

  /* ����ʹ�ò��� */
	//���
	if(self->info.MoveCommand == RP_OK){
	
		cap.modifyLimit(&cap,cap.data.tx.output_power_limit,0);
	}
	else if(self->info.MoveCommand == RP_NO){
	
		cap.modifyLimit(&cap,cap.data.tx.output_power_limit,150);
	}
	
	//�ŵ�
	if(self->info.MoveCrazzy == RP_OK){
	
		cap.modifyLimit(&cap,150,cap.data.tx.input_power_limit);
	}
	else if(self->info.MoveCrazzy == RP_NO){
	
		cap.modifyLimit(&cap,75,cap.data.tx.input_power_limit);
	}
	
#endif	
	
	
	
		
//		self->info.SysInit    = RP_NO;
//		self->info.CtrlInit   = RP_NO;
//		self->info.MoveInit   = RP_NO;		
//		self->info.RifleInit  = RP_NO;
//		self->info.VisionInit = RP_NO;
//		
//		self->info.RifleLock = RP_NO;
//		
//		self->info.FrictionSwitch = RP_NO;
//		self->info.MagazineSwitch = RP_NO;
//		self->info.RotateSwitch   = RP_NO;
//		self->info.AutoFireSwitch = RP_NO;
//		
//		self->modifyCtrlMode(self,CTRL_NULL);
//		self->modifyMoveMode(self,MOVE_NULL);
//		self->modifyRifleMode(self,RIFLE_NULL);
//		
//		gun.ModifyLock(&gun,RIFLE_LOCK);
//		head.ModifyLock(&head,GIMB_LOCK);
//		omni.ModifyLock(&omni,CHAS_LOCK);

