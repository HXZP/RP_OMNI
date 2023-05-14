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
#include "GimbalRotationOutput.h"

void Gimbal_ModifyLock(gimbal *gimb,gimbal_Lock type);
void Gimbal_Updata(gimbal *gimb,float ax,float ay,float az,float rx,float ry,float rz);
void Gimbal_Resolving(gimbal* gimb,float *setX,float *setY,float *setZ);
void Gimbal_Translation(gimbal* gimb,float chasX,float chasY,float chasZ);
void Gimbal_Ctrl(gimbal *gimb);


gimbal head = {

	.info.MotorState = GIMB_MOTOR_ONLINE,
  .info.Lock       = GIMB_UNLOCK,	

	.info.elevation  = 30,
	.info.depression = 30,
	
	.ModifyLock  = Gimbal_ModifyLock,
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

/** @FUN  更新云台信息
  * @ax   角度 0-360
  * @rx   速度 不限单位
  */
void Gimbal_Updata(gimbal *gimb,float ax,float ay,float az,float rx,float ry,float rz)
{
	//云台电机失联判断
	if(motor[GIMB_Y].state.work_state && motor[GIMB_P].state.work_state){
		
		gimb->info.MotorState = GIMB_MOTOR_ONLINE;
	}	
	else{
	
		gimb->info.MotorState = GIMB_MOTOR_ERR;
	}

	gimb->data.Angle.X = ax;
	gimb->data.Angle.Y = ay;
  gimb->data.Angle.Z = az;

	gimb->data.Speed.X = rx;
	gimb->data.Speed.Y = ry;
  gimb->data.Speed.Z = rz;

}

/** @FUN  pid解算扭矩
  * @set  角度 360
  * 
  */
void Gimbal_Resolving(gimbal* gimb,float *setX,float *setY,float *setZ)
{
	*setY = anti_constrain(*setY,gimb->info.depression,360-gimb->info.elevation);
	
	//pid计算
	gimb->data.Torque.X = 0;
														 
	gimb->data.Torque.Y = 
	motor[GIMB_P].c_pid2(&motor[GIMB_P].pid.angle,
											 &motor[GIMB_P].pid.angle_in,
											 gimb->data.Angle.Y,
											 gimb->data.Speed.Y,
											 *setY,1);		
	
	gimb->data.Torque.Z = 
	motor[GIMB_Y].c_pid2(&motor[GIMB_Y].pid.angle,
											 &motor[GIMB_Y].pid.angle_in,
											 gimb->data.Angle.Z,
											 gimb->data.Speed.Z,
											 *setZ,1);

}


void Gimbal_Translation(gimbal* gimb,float chasX,float chasY,float chasZ)
{

 RP_RotationOutput_Chassis2Gimb(chasX,
	                              chasY,
	                              chasZ,
	
	                              gimb->data.Angle.X,
	                              gimb->data.Angle.Y,
	                              gimb->data.Angle.Z,
	
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

			motor[GIMB_Y].tx(&motor[GIMB_Y],Gimbal_CANBuff);	
		}

	}
	else if(gimb->info.Lock == GIMB_UNLOCK)
	{
		Gimbal_CANBuff[motor[GIMB_Y].id.buff_p] = gimb->data.Torque.Z;
		Gimbal_CANBuff[motor[GIMB_P].id.buff_p] = gimb->data.Torque.Y;

		motor[GIMB_Y].tx(&motor[GIMB_Y],Gimbal_CANBuff);	
	}	

}





















