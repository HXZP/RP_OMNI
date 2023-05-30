/*

遥控器控制：

sw1 = 2|3
3云台模式
2底盘模式

	sw2
	1连发
	3停止
	2单发

	滚轮控制
	遥控器模式下
	最上 摩擦轮
	偏上 弹舱盖
	云台模式
	往下 自瞄
	偏下 打符
	最下 小陀螺

sw1 = 1
1键盘-测试模式

	sw2
	1键盘模式
	3测试模式1
	2测试模式2

	滚轮
	最上 开启/退出测试模式
	偏上 软件复位
	偏下 软件复位
	最下 软件复位

************************************************

代码逻辑：
	任务：
		基于freertos进行任务调度资源分配
	任务优先级原则：
		信息交互为最高优先级
		控制为最低优先级
		看门狗为最低优先级
	文件：
		包含：
			单向包含
			高级包含低级
			同等级可包含
			.h包含标准头文件、低级头文件
			.c包含同级头文件、高级头文件（尽量别）
		  高级的内容不出现在低级，高级修改低级内容要通过内置函数修改（尽量）	
		内容：
			以模块为区分
			设备
			驱动
			模块
			调度


模式定义:
1.对于同一机械结构、电子设备
2.在同一时间不能同时执行的两种程序
3.称之为两种模式

************************************************

模式切换代码：
定义模式
定义模式切换函数，一个模式切换函数只能进行一种机构的切换
（当然也可以写成复用的，但可能存在资源相互使用的问题）
定义模式初始化函数

切换流程：
	使用切换函数
	比较目标模式和当前模式

		相同：
		跳出

		不同：
		当前模式为NULL
		初始化标志位亮起
		进行初始化
		初始化完成
		切换目标模式
	（关于初始化：只考虑目标模式下所需要的初始环境，不考虑上一模式的情况
		
执行：
	只执行当前模式
	为NULL时不执行
	(关于执行内容和初始化内容存在交集：把执行内容写在初始化
	
模块编写思路：
	主要包括四部分
	1、数据更新
	2、解算
	3、控制
	4、修改参数API
	主要是通过API函数进行控制，其他部分一直运行
	
	
	

	

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

/*核心*/
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
 *@note 初始化表
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
	
	//头部复位
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
	
	//超时强制复位云台
	if(HAL_GetTick() - self->time.InitTimeStart > CENTER_INIT_OUT_TIME){
		
		self->info.SysInit  = RP_OK;
	}
	
	//复位成功 进行下一步解锁
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
	
	//默认开启辅助射击
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
 *@note 控制开关中心 修改目标状态
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

			Remote_Ctrl(self);
		}
		/* 遥控模式end */

		/* 键盘模式 */
		else if(self->info.CtrlMode == CTRL_KM){
		
			self->info.RifleLock = RP_OK;

			KeyMouse_Ctrl(self);
		}
		/* 键盘模式end */
	}
	
	/* 控制模式end */
	
	return RP_OK;
}



/*
 *@note 子开关：遥控器
**/
Center_State Remote_Ctrl(center *self)
{
	
	//更新通道值
	self->data.Channel[0] = (float)rc.data.ch0/660.f*100;
	self->data.Channel[1] = (float)rc.data.ch1/660.f*100;
	self->data.Channel[2] = (float)rc.data.ch2/660.f*100;	
	self->data.Channel[3] = (float)rc.data.ch3/660.f*100;	
	
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

		gun.ModifyState(&gun.info.AutoSetEnable,RIFLE_NO);
		
		//开启小陀螺
		if(rc.data.tw_step[2]){
		
			self->modifyState(&self->info.RotateSwitch,RP_OK);
		}
		else{
			self->data.RotateVelocity = ROTATE_VELOCITY_DEFUALT;
		
      self->modifyState(&self->info.RotateSwitch,RP_NO);
		}

		//开启视觉
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
	/* 底盘模式 */
	else if(self->info.MoveMode == MOVE_FOLLOW){

		gun.ModifyState(&gun.info.AutoSetEnable,RIFLE_OK);
	}
	/* 移动模式end */
	

	/* 发射模式 */			
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
	
	/* 发射机构未退出保护 */
	if(self->info.RifleLock == RP_NO){
	
		self->modifyRifleMode(self,RIFLE_STOP);
	}
	
	/* 发射模式end */
	
	/* 滚轮控制 */
	
	//摩擦轮
	if(rc.data.tw_step[0]){
	
		self->modifyState(&self->info.FrictionSwitch,RP_OK);
	}
	else{
	
		self->modifyState(&self->info.FrictionSwitch,RP_NO);
	}
	//弹仓
	if(rc.data.tw_step[1]){
	
		self->modifyState(&self->info.MagazineSwitch,RP_OK);
	}
	else{
	
		self->modifyState(&self->info.MagazineSwitch,RP_NO);
	}
	/* 滚轮控制end */

	
	return RP_OK;
}



/*
 * @note 子开关：键盘
**/
Center_State KeyMouse_Ctrl(center *self)
{
	//更新通道值
	self->data.Channel[0] = rc.data.ch[0];
	self->data.Channel[1] = rc.data.ch[1];
	self->data.Channel[2] = rc.data.ch[2]/660.f*100;
	self->data.Channel[3] = rc.data.ch[3]/660.f*100;
	
	
	/* 快速转向 */

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
	
	/* 小陀螺 */
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
	
	/* 弹仓 */
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
	
	/* 榴莲发 */
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
	
	/* 复位 */
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
	
	/* 鼠标 */
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
 * 根据目标模式 调用底层API进行修改状态
**/


/* ---------------------------Center_Updata子-------------------------- */

void Center_StateUpdata(center *self)
{
	/* 移动命令 */
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
	
  /* 功率使用策略 */
	//充电
	if(self->info.MoveCommand == RP_OK){
	
		cap.modifyLimit(&cap,cap.data.tx.output_power_limit,0);
	}
	else if(self->info.MoveCommand == RP_NO){
	
		cap.modifyLimit(&cap,cap.data.tx.output_power_limit,150);
	}
	
	//放电
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
	//云台跟随
	if(self->info.MoveMode == MOVE_MASTER){
		
    //快速转向赋值
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
		//正常转向
		else{
		
			if(anti_constrain(RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360),head.info.depression,360 - head.info.elevation)){
			
				self->data.GimbPitTarget = RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360);
			}

			if(abs(head.data.AngleErr.Z) <= 90){
						
				self->data.GimbYawTarget = RP_Limit(self->data.GimbYawTarget - (float)self->data.Channel[0]/100,360);
			}
		}
	}
	//底盘跟随
	else if(self->info.MoveMode == MOVE_FOLLOW){

		if(anti_constrain(RP_Limit(self->data.GimbPitTarget + (float)self->data.Channel[1]/100,360),head.info.depression,360 - head.info.elevation)){
		
			self->data.GimbPitTarget  = RP_Limit(self->data.GimbPitTarget+(float)self->data.Channel[1]/100,360);
		}
	}
	
	
	//强制云台持平
	if(self->info.MagazineSwitch == RP_OK){
	
		self->data.GimbPitTarget = 0;
	}

	//pit限位
	if(RP_Limit(self->data.GimbPitTarget,360) > 0 && RP_Limit(self->data.GimbPitTarget,360) < 180){
	
		self->data.GimbPitTarget = (self->data.GimbPitTarget > head.info.depression ? head.info.depression : self->data.GimbPitTarget);
	}
	else if(RP_Limit(self->data.GimbPitTarget,360) > 180 && RP_Limit(self->data.GimbPitTarget,360) < 360){
	
		self->data.GimbPitTarget = (self->data.GimbPitTarget < 360 - head.info.elevation ? 360 - head.info.elevation : self->data.GimbPitTarget);
	}

	//赋值
	head.ModifyXYZSet(&head,0,self->data.GimbPitTarget,self->data.GimbYawTarget);
}



void Center_MoveStrategy(center *self)
{	
	float angle;	
	
	//移动策略
	if(self->info.MoveXYCommand == RP_OK){
	
		omni.ModifyDistribute(&omni,CHAS_LINEAR);
	}			
	else{
	
		omni.ModifyDistribute(&omni,CHAS_ROTATE);
	}
	
	//云台模式赋值
	if(self->info.MoveMode == MOVE_MASTER){

		if(self->info.RotateSwitch == RP_OK){
			
			self->data.VelocityX = (float)self->data.Channel[3];
			self->data.VelocityY =-(float)self->data.Channel[2];
		  self->data.VelocityZ = self->data.RotateVelocity;
			
			//小陀螺功率自适应
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
	//底盘模式赋值
	else if(self->info.MoveMode == MOVE_FOLLOW){
	
		self->data.VelocityX = (float)self->data.Channel[3];
		self->data.VelocityY =-(float)self->data.Channel[2];
		self->data.VelocityZ =-(float)self->data.Channel[0];
		
		//以此时电机坐标为原点，也就是说和底盘位置不产生角度
		omni.ModifyOriginAngle(&omni,((float)motor[GIMB_Y].rx_info.angle_offset)/22.755f);
	}
	
	omni.ModifyXYZSet(&omni,self->data.VelocityX,self->data.VelocityY,self->data.VelocityZ);
		
}


void Center_ShootStrategy(center *self)
{
	//摩擦轮
	if(self->info.FrictionSwitch == RP_OK){
	
		gun.ModifyFri(&gun,RIFLE_OK);
	}
	else if(self->info.FrictionSwitch == RP_NO){
	
		gun.ModifyFri(&gun,RIFLE_NO);
	}

	//弹仓盖
	if(self->info.MagazineSwitch == RP_OK){
	
		gun.ModifyMagazine(&gun,RIFLE_OK);
	}
	else if(self->info.MagazineSwitch == RP_NO){
	
		gun.ModifyMagazine(&gun,RIFLE_NO);
	}

	//发射模式设置
	switch((uint8_t)self->info.RifleMode){
	
		case RIFLE_SET1:
			gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,1);
			break;
		
		case RIFLE_SET6:
			gun.ModifyShootType(&gun,RIFLE_SHOOT_SET,6);
			break;		
		
		case RIFLE_STAY:
			//热量调整
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
/* ---------------------------Center_Updata子-------------------------- */


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
 * 执行底层模块
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
	
	//姿态重分配
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
模式切换

比较此时的模式和命令
如果相同则返回
如果不同则设未进行初始化，进入初始化中状态

如果此时处于初始化状态则进入初始化函数
初始化完成后，状态设为已完成初始化
初始化完成标志亮起，则修改此时的模式

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

	/* 移动命令 */
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
	
	//移动策略
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
			
			//小陀螺功率自适应
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
		
		//以此时电机坐标为原点，也就是说和底盘位置不产生角度
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
	
	

  /* 电容使用策略 */
	//充电
	if(self->info.MoveCommand == RP_OK){
	
		cap.modifyLimit(&cap,cap.data.tx.output_power_limit,0);
	}
	else if(self->info.MoveCommand == RP_NO){
	
		cap.modifyLimit(&cap,cap.data.tx.output_power_limit,150);
	}
	
	//放电
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

