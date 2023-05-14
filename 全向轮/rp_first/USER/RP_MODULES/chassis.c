/*
	底盘控制
	坐标系（右手）：
	
	     x
	     |
	     |
	y----z
	
*/

#include "DEVICES.h"
#include "chassis.h"



static void Chassis_ModifyLock(chassis *chas,chassis_Lock type);
static void Chassis_ModifyrpmMax(chassis *chas,float max);
static void Chassis_Updata(chassis *chas);
static void Chassis_Resolving(chassis *chas,float setX,float setY,float setZ);
static void Chassis_Ctrl(chassis *chas);
static void Chassis_Power_Limit(chassis *chas,int16_t *data);

chassis omni = {

	.info.Type       = CHAS_OMNI,
	.info.MotorState = CHAS_ONLINE,
	.info.Direction  = CHAS_FORWARD,
  .info.Lock       = CHAS_UNLOCK,

  .data.WheelrpmMax = 8000,
	
	.info.ReductionRatio = 14,
	.info.WheelRadius    = 10,
	.info.VehicleLength  = 1,
	.info.VehicleWide    = 1,
	
	.ModifyLock     = Chassis_ModifyLock,
	.ModifyrpmMax   = Chassis_ModifyrpmMax,
	.Updata         = Chassis_Updata,
	.Resolving      = Chassis_Resolving,
	.Ctrl           = Chassis_Ctrl,

	
};





/** @FUN  修改底盘允许控制状态
  * @type CHAS_UNLOCK  CHAS_LOCK
  */
void Chassis_ModifyLock(chassis *chas,chassis_Lock type)
{
	if(type == CHAS_LOCK && chas->info.Lock == CHAS_UNLOCK){
	
		chas->info.Lock = CHAS_LOCK;
		chas->time.LockTime = HAL_GetTick();
	}
	else if(type == CHAS_UNLOCK && chas->info.Lock == CHAS_LOCK){
	
		chas->info.Lock = CHAS_UNLOCK;
		chas->time.unLockTime = HAL_GetTick();
	}	
}

/** @FUN  修改底盘最大速度
  * @velocity m/s
  */
void Chassis_ModifyrpmMax(chassis *chas,float max)
{
	chas->data.WheelrpmMax = max;
}


/**
  * @xyz m/s
  * @position the gimb position:0~360  0-+90为前
  */
void Chassis_Updata(chassis *chas)
{
	int16_t position;
	
	//底盘电机失联判断
	if(motor[CHAS_1].state.work_state && motor[CHAS_2].state.work_state
	&& motor[CHAS_3].state.work_state && motor[CHAS_4].state.work_state){
		
		chas->info.MotorState = CHAS_ONLINE;
	}	
	else{
	
		chas->info.MotorState = CHAS_ERR;
	}
	
	
	//底盘方向判断
	position = motor[GIMB_Y].rx_info.angle;
	
	if(abs(position - 180) < 90)
	{
		chas->info.Direction = CHAS_BACKWARD;
	}
	else 
	{
		chas->info.Direction = CHAS_FORWARD;
	}	
	
	//底盘轮子最大速度计算
	chas->data.VelocityMax = chas->data.WheelrpmMax/chas->info.ReductionRatio/60.0f
	                         *2.0f*pi*chas->info.WheelRadius;
	
	//底盘电机速度更新
	chas->data.WheelReal[0] = motor[CHAS_1].rx_info.speed;
  chas->data.WheelReal[1] = motor[CHAS_2].rx_info.speed;
	chas->data.WheelReal[2] = motor[CHAS_3].rx_info.speed;
	chas->data.WheelReal[3] = motor[CHAS_4].rx_info.speed;
	
	//瞎写的
	chas->data.VelocityReal.x = chas->data.WheelReal[0]-chas->data.WheelReal[2]+chas->data.WheelReal[1]-chas->data.WheelReal[3];
	chas->data.VelocityReal.y =-chas->data.WheelReal[0]-chas->data.WheelReal[2]+chas->data.WheelReal[1]+chas->data.WheelReal[3];
	chas->data.VelocityReal.z =-chas->data.WheelReal[0]-chas->data.WheelReal[2]-chas->data.WheelReal[1]-chas->data.WheelReal[3];
	
}


/**
  * 解算轮子的转速 @xyz -100~100
  */
void Chassis_Resolving(chassis *chas,float setX,float setY,float setZ)
{
	float velocity[4],velocityAbsoluteMax;
	
	if(chas->info.Direction == CHAS_BACKWARD){
	
		chas->data.VelocitySet.x = -setX;
		chas->data.VelocitySet.y = -setY;
		chas->data.VelocitySet.z =  setZ;
	}
	else{
	
		chas->data.VelocitySet.x = setX;
		chas->data.VelocitySet.y = setY;
		chas->data.VelocitySet.z = setZ;
	}
	
	if(chas->info.Type == CHAS_OMNI){
    
		velocity[0] = chas->data.VelocitySet.x + chas->data.VelocitySet.y - chas->data.VelocitySet.z;
		velocity[1] = chas->data.VelocitySet.x - chas->data.VelocitySet.y - chas->data.VelocitySet.z;	
		velocity[2] =-chas->data.VelocitySet.x + chas->data.VelocitySet.y - chas->data.VelocitySet.z;
		velocity[3] =-chas->data.VelocitySet.x - chas->data.VelocitySet.y - chas->data.VelocitySet.z;
	}
	
	velocityAbsoluteMax = RP_GetAbsoluteMax(velocity,4);
	
	if(velocityAbsoluteMax > 100){
	
		for(char i = 0 ; i < 4 ; i++)
		{
			velocity[i] = velocity[i]/velocityAbsoluteMax;
			chas->data.WheelSet[i] = velocity[i] * chas->data.WheelrpmMax;
		}
	}
	else{
	
		for(char i = 0 ; i < 4 ; i++)
		{
			velocity[i] = velocity[i]/100;
			chas->data.WheelSet[i] = velocity[i] * chas->data.WheelrpmMax;
		}
	}
}


/**
  * 解算轮子的扭矩 功率控制
  */
void Chassis_Ctrl(chassis *chas)
{
	int16_t Chassis_CANBuff[4];
	
	if(chas->info.Lock == CHAS_LOCK)
	{
		if(HAL_GetTick() - chas->time.LockTime < 1000){
		
			Chassis_CANBuff[motor[CHAS_1].id.buff_p] = motor[CHAS_1].c_speed(&motor[CHAS_1],0);
			Chassis_CANBuff[motor[CHAS_2].id.buff_p] = motor[CHAS_2].c_speed(&motor[CHAS_2],0);
			Chassis_CANBuff[motor[CHAS_3].id.buff_p] = motor[CHAS_3].c_speed(&motor[CHAS_3],0);
			Chassis_CANBuff[motor[CHAS_4].id.buff_p] = motor[CHAS_4].c_speed(&motor[CHAS_4],0);	
			
			Chassis_Power_Limit(chas,Chassis_CANBuff);
			
			motor[CHAS_1].tx(&motor[CHAS_1],Chassis_CANBuff);	
		}

	}
	else if(chas->info.Lock == CHAS_UNLOCK)
	{
		Chassis_CANBuff[motor[CHAS_1].id.buff_p] = motor[CHAS_1].c_speed(&motor[CHAS_1],chas->data.WheelSet[0]);
		Chassis_CANBuff[motor[CHAS_2].id.buff_p] = motor[CHAS_2].c_speed(&motor[CHAS_2],chas->data.WheelSet[1]);
		Chassis_CANBuff[motor[CHAS_3].id.buff_p] = motor[CHAS_3].c_speed(&motor[CHAS_3],chas->data.WheelSet[2]);
		Chassis_CANBuff[motor[CHAS_4].id.buff_p] = motor[CHAS_4].c_speed(&motor[CHAS_4],chas->data.WheelSet[3]);
		
		Chassis_Power_Limit(chas,Chassis_CANBuff);
		
		motor[CHAS_1].tx(&motor[CHAS_1],Chassis_CANBuff);	
	}
	
}




/**
  * 功率算法
  */
void Chassis_Power_Limit(chassis *chas,int16_t *data)
{
	float buffer = judge.data.power_heat_data.chassis_power_buffer;
	
	float heat_rate,Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;
	
	uint16_t OUT_MAX = 0;
	
	if(judge.info.state == CAP_ONLINE && cap.info.state == CAP_ONLINE){
	
		OUT_MAX = 8000 * 4;
	}
	else{
	
		OUT_MAX = 4000 * 4;
	}
	
	if(buffer > 60)buffer = 60;//防止飞坡之后缓冲250J变为正增益系数
	
	Limit_k = buffer / 60;
	
	if(buffer < 25)
		Limit_k = Limit_k * Limit_k ;
	else
		Limit_k = Limit_k;
	
	if(buffer < 60)
		CHAS_LimitOutput = Limit_k * OUT_MAX;
	else 
		CHAS_LimitOutput = OUT_MAX;    
	
	CHAS_TotalOutput = abs(data[0]) + abs(data[1]) + abs(data[2]) + abs(data[3]) ;
	
	heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;
	
  if(CHAS_TotalOutput >= CHAS_LimitOutput)
  {
		for(char i = 0 ; i < 4 ; i++)
		{	
			data[i] = (int16_t)(data[i] * heat_rate);	
		}
	}
}



















