/*
	云台控制
	坐标系（右手）：
	
	     x
	     |
	     |
	y----z
	
*/

#include "gimbal.h"
#include "motor.h"
#include "RP_FUNCTION.h"
#include "RP_CONFIG.h"
#include "GimbalRotationOutput.h"

#define PI 3.14159265358979f


static void Gimbal_ModifyLock(gimbal *gimb,gimbal_Lock type);
static void Gimbal_ModifyXYZSet(gimbal *gimb,float setX,float setY,float setZ);
static float Gimbal_ModifyDataRange360(float data,float min,float max);
static void Gimbal_ModifyXYZUpdata(gimbal *gimb,float ax,float ay,float az,float rx,float ry,float rz);

static void Gimbal_Updata(gimbal *gimb);
static void Gimbal_Resolving(gimbal* gimb);
static void Gimbal_Translation(gimbal* gimb,float chasX,float chasY,float chasZ);
static void Gimbal_Ctrl(gimbal *gimb);


gimbal head = {

	.info.MotorState = GIMB_MOTOR_ERR,
  .info.Lock       = GIMB_UNLOCK,	

	.info.elevation  = 40,
	.info.depression = 20,
	
	.info.YReach = GIMB_NO,
	.info.ZReach = GIMB_NO,
	
	.info.AssemblyVector.X = 1,
	.info.AssemblyVector.Y =-1,
	.info.AssemblyVector.Z = 1,
	
	.ModifyLock      = Gimbal_ModifyLock,
	.ModifyXYZSet    = Gimbal_ModifyXYZSet,
	.ModifyXYZUpdata = Gimbal_ModifyXYZUpdata,
	.ModifyRange     = Gimbal_ModifyDataRange360,
	
	.Updata      = Gimbal_Updata,
	.Resolving   = Gimbal_Resolving,
	.Translation = Gimbal_Translation,
	.Ctrl        = Gimbal_Ctrl,
	
};










/** @FUN  修改云台允许控制状态
  * @type GIMB_UNLOCK  GIMB_LOCK
  */
void Gimbal_ModifyLock(gimbal *gimb,gimbal_Lock type)
{
	if(type == GIMB_LOCK && gimb->info.Lock == GIMB_UNLOCK){
	
		gimb->info.Lock = GIMB_LOCK;
		gimb->time.LockTime = HAL_GetTick();
	}
	else if(type == GIMB_UNLOCK && gimb->info.Lock == GIMB_LOCK){
	
		gimb->info.Lock = GIMB_UNLOCK;
		gimb->time.unLockTime = HAL_GetTick();
	}	
}


/** @FUN  修改数据
  * 
  */
float Gimbal_ModifyDataRange360(float data,float min,float max)
{
	data = data + (max - min);
	data = data/(max - min) * 360.f;

	if(data > 360.f){
	
		data = data - 360;
	}
	
	return data;
}


/** @FUN  修改云台目标
  * @xyz  0~360
  */
void Gimbal_ModifyXYZSet(gimbal *gimb,float setX,float setY,float setZ)
{
	
	gimb->data.AngleSet.X = setX;
	gimb->data.AngleSet.Y = setY;
	gimb->data.AngleSet.Z = setZ;

}

/** @FUN  修改云台目标
  * @xyz  0~360
  */
void Gimbal_ModifyXYZUpdata(gimbal *gimb,float ax,float ay,float az,float rx,float ry,float rz)
{
	
	gimb->data.Angle.X = ax;
	gimb->data.Angle.Y = ay;
  gimb->data.Angle.Z = az;

	gimb->data.Speed.X = rx;
	gimb->data.Speed.Y = ry;
  gimb->data.Speed.Z = rz;

}




/** @FUN  更新云台信息
  * @ax   角度 0~360
  * @rx   速度 不限单位
  */
void Gimbal_Updata(gimbal *gimb)
{
	//云台电机失联判断
	if(motor[GIMB_Y].state.work_state == M_ONLINE && motor[GIMB_P].state.work_state == M_ONLINE){
		
		gimb->info.MotorState = GIMB_MOTOR_ONLINE;
	}	
	else{
	
		gimb->info.MotorState = GIMB_MOTOR_ERR;
	}

	gimb->data.AngleErr.X = RP_HalfTurn(gimb->data.AngleSet.X - gimb->data.Angle.X,360);
	gimb->data.AngleErr.Y = RP_HalfTurn(gimb->data.AngleSet.Y - gimb->data.Angle.Y,360);
	gimb->data.AngleErr.Z = RP_HalfTurn(gimb->data.AngleSet.Z - gimb->data.Angle.Z,360);
	
	if(gimb->data.AngleErr.Y  < 5){
	
		gimb->info.YReach = GIMB_OK;
	}
	else{
	
		gimb->info.YReach = GIMB_NO;
	}
	
	if(gimb->data.AngleErr.Z < 5){
	
		gimb->info.ZReach = GIMB_OK;
	}		
	else{
	
		gimb->info.ZReach = GIMB_NO;
	}	
	
}


/** @FUN  pid解算扭矩
  * @set  角度 360
  * 
  */
void Gimbal_Resolving(gimbal* gimb)
{

	//pid计算
	gimb->data.Torque.X = 0;
														 
	gimb->data.Torque.Y = 
	gimb->info.AssemblyVector.Y*
	motor[GIMB_P].c_pid2(&motor[GIMB_P].pid.angle,
											 &motor[GIMB_P].pid.angle_in,
											 gimb->data.Angle.Y,
											 gimb->data.Speed.Y,
											 gimb->data.AngleSet.Y,1);
	
//	gimb->data.Torque.Z = 
//	gimb->info.AssemblyVector.Z*
//	motor[GIMB_Y].c_pid2(&motor[GIMB_Y].pid.angle,
//											 &motor[GIMB_Y].pid.angle_in,
//											 gimb->data.Angle.Z,
//											 gimb->data.Speed.Z,
//											 gimb->data.AngleSet.Z,1);

	gimb->data.Torque.Z = 
	gimb->info.AssemblyVector.Z*
	motor[GIMB_Y].c_pidfuzzy(&fuzzyPidYaw,
													 &motor[GIMB_Y].pid.angle_in,
													 gimb->data.Angle.Z,
													 gimb->data.Speed.Z,
													 gimb->data.AngleSet.Z);

	gimb->data.Torque.Y = gimb->data.Torque.Y - 3000;

}


void Gimbal_Translation(gimbal* gimb,float chasX,float chasY,float chasZ)
{

 RP_RotationOutput_Chassis2Gimb(chasX/180*PI,
	                              chasY/180*PI,
	                              chasZ/180*PI,
	
	                              gimb->data.Angle.X/180*PI,
	                              gimb->data.Angle.Y/180*PI,
	                              gimb->data.Angle.Z/180*PI,
	
		                            &gimb->data.Torque.X,
	                              &gimb->data.Torque.Y,
	                              &gimb->data.Torque.Z);

}



void Gimbal_Ctrl(gimbal *gimb)
{
	int16_t Gimbal_CANBuff[4];

	if(gimb->info.Lock == GIMB_LOCK)
	{
		if(HAL_GetTick() - gimb->time.LockTime < 1000){
		
			Gimbal_CANBuff[motor[GIMB_Y].id.buff_p] = motor[GIMB_Y].c_speed(&motor[GIMB_Y],0);
			Gimbal_CANBuff[motor[GIMB_P].id.buff_p] = motor[GIMB_P].c_speed(&motor[GIMB_P],0);

			if(GIMBAL_GLOBAL)motor[GIMB_Y].tx(&motor[GIMB_Y],Gimbal_CANBuff);	
		}

	}
	else if(gimb->info.Lock == GIMB_UNLOCK)
	{
		Gimbal_CANBuff[motor[GIMB_Y].id.buff_p] = gimb->data.Torque.Z;
		Gimbal_CANBuff[motor[GIMB_P].id.buff_p] = gimb->data.Torque.Y;

		if(GIMBAL_GLOBAL)motor[GIMB_Y].tx(&motor[GIMB_Y],Gimbal_CANBuff);
	}	

}





















