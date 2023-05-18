/*
	���̿���
	����ϵ�����֣���
	
	     x
	     |
	     |
	y----z
	
*/

#include "DEVICES.h"
#include "chassis.h"
#include "arm_math.h"


static void Chassis_ModifyLock(chassis *chas,chassis_Lock type);
static void Chassis_ModifyrpmMax(chassis *chas,float max);
static void Chassis_ModifyOriginAngle(chassis *chas,float angle);
static void Chassis_ModifyXYZSet(chassis *chas,float setX,float setY,float setZ);
static void Chassis_ModifyDistribute(chassis *chas,chassis_Distribute type);

static void Chassis_Updata(chassis *chas);
static void Chassis_Resolving(chassis *chas);
static void Chassis_Ctrl(chassis *chas);

static void Chassis_Power_Limit(chassis *chas,int16_t *data);

chassis omni = {

	.info.Type       = CHAS_OMNI,
	.info.MotorState = CHAS_ONLINE,
  .info.Lock       = CHAS_UNLOCK,

	.info.Direction  = CHAS_FORWARD,
	.info.Distribute = CHAS_FAIR,
  .data.WheelrpmMax = 8000,
	
	.info.ReductionRatio = 14,
	.info.WheelRadius    = 10,
	.info.VehicleLength  = 1,
	.info.VehicleWide    = 1,
	
	.info.OriginAngle = 0,
	
	.ModifyLock        = Chassis_ModifyLock,
	.ModifyrpmMax      = Chassis_ModifyrpmMax,
	.ModifyXYZSet      = Chassis_ModifyXYZSet,
	.ModifyOriginAngle = Chassis_ModifyOriginAngle,
	.ModifyDistribute  = Chassis_ModifyDistribute,
	
	
	.Updata         = Chassis_Updata,
	.Resolving      = Chassis_Resolving,
	.Ctrl           = Chassis_Ctrl,

	
};





/** @FUN  �޸ĵ����������״̬
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



/** @FUN  �޸ĵ�������ٶ�
  * @velocity m/s
  */
void Chassis_ModifyrpmMax(chassis *chas,float max)
{
	chas->data.WheelrpmMax = max;
}


/** @FUN  �޸ĵ���ԭ��Ƕ�
  * @angle 0-360
  */
void Chassis_ModifyOriginAngle(chassis *chas,float angle)
{
	chas->info.OriginAngle = angle;
}

/** @FUN  �޸ĵ����ٶȷ���ģʽ
  * @velocity m/s
  */
void Chassis_ModifyDistribute(chassis *chas,chassis_Distribute type)
{
	chas->info.Distribute = type;
}

/** @FUN  �޸ĵ����ٶ�Ŀ�� ���ҽ����ٶȷ��� �ٶ��ܺ�Ϊ100 ��ƽ���� �������� ��ת����
  * @xyz -100~100
  */
void Chassis_ModifyXYZSet(chassis *chas,float setX,float setY,float setZ)
{
	chassis_xyz temp,tempDis;
	float vel[3],velTotal,velRemain,xyTotal;
	
	float angle = chas->data.DirAngle;

	if(abs(setX) > 100 || abs(setY) > 100 || abs(setZ) > 100){
	
		setX = RP_GetSymbol(setX) * 100;
		setY = RP_GetSymbol(setY) * 100;
		setZ = RP_GetSymbol(setZ) * 100;
	}
	
	if(chas->info.Direction == CHAS_BACKWARD){
	
		temp.x = -setX;
		temp.y = -setY;
		temp.z =  setZ;
	}
	else{
	
		temp.x = setX;
		temp.y = setY;
		temp.z = setZ;
	}
	
	tempDis.x = temp.x * cos(angle) - temp.y * sin(angle);
	tempDis.y = temp.x * sin(angle) + temp.y * cos(angle);	
	tempDis.z = temp.z;


	vel[0] = tempDis.x;
	vel[1] = tempDis.y;
	vel[2] = tempDis.z;
	
	//x+y <= |x|+|y| 
	if(RP_GetAbsoluteTotal(vel,3) <= 100){
		
		chas->data.VelocitySet.x = tempDis.x;
		chas->data.VelocitySet.y = tempDis.y;
		chas->data.VelocitySet.z = tempDis.z;
		
		return;
	}
	
	//��������ٶȣ������ٶȷ���

	if(chas->info.Distribute == CHAS_FAIR){

    velTotal = RP_GetAbsoluteTotal(vel,3);
			
		for(char i = 0 ; i < 3 ; i++)
		{
			vel[i] = vel[i]/velTotal * 100;
		}
	}
	else if(chas->info.Distribute == CHAS_LINEAR){
	
		xyTotal = RP_GetAbsoluteTotal(vel,2);
		
		velRemain = 100 - xyTotal;
		
		//û��ʣ���ٶ� z���ٶ�Ϊ0
		if(velRemain < 0){
		
			for(char i = 0 ; i < 2 ; i++)
			{
				vel[i] = vel[i]/xyTotal * 100;
			}	
			vel[2] = 0;
		}
		//��ʣ���ٶ� z���ٶȿ�ʣ�����
		else{
		
			vel[2] = (abs(vel[2]) > velRemain? velRemain : vel[2]);

		}

	}
	else if(chas->info.Distribute == CHAS_ROTATE){
	
		xyTotal = RP_GetAbsoluteTotal(vel,2);
		
		velRemain = 100 - vel[2];
		
		//��ʣ���ٶ� �ٶȿ�ʣ�����
		if(velRemain > 0){
		
			if(xyTotal > velRemain){
			
				for(char i = 0 ; i < 2 ; i++)
				{
					vel[i] = vel[i]/velRemain;
				}	
			}
		}
		//��ʣ���ٶ� 
		else{
		
			if(xyTotal > velRemain){
			
				for(char i = 0 ; i < 2 ; i++)
				{
					vel[i] = vel[i]/xyTotal * 100;
				}	
			}			
			vel[2] = 0;
		}		
	}	

	tempDis.x = vel[0];
	tempDis.y = vel[1];	
	tempDis.z = vel[2];	
	
	chas->data.VelocitySet.x = tempDis.x;
	chas->data.VelocitySet.y = tempDis.y;
	chas->data.VelocitySet.z = tempDis.z;
	
}




/**
  * @xyz m/s
  * @position the gimb position:0~360  0-+90Ϊǰ
  */
void Chassis_Updata(chassis *chas)
{
	float position;
	
	//���̵��ʧ���ж�
	if(motor[CHAS_1].state.work_state && motor[CHAS_2].state.work_state
	&& motor[CHAS_3].state.work_state && motor[CHAS_4].state.work_state){
		
		chas->info.MotorState = CHAS_ONLINE;
	}	
	else{
	
		chas->info.MotorState = CHAS_ERR;
	}
	
	
	//���̷����ж� 0-360 ��ȡ��ԭ��ķ���н�
	position = ((float)motor[GIMB_Y].rx_info.angle)/22.5f;
	
	if(abs(position - 180) < 90){
		
		chas->info.Direction = CHAS_BACKWARD;
		position = RP_Limit(position - 180 - chas->info.OriginAngle,360);
		chas->data.DirAngle = position;
	}
	else{
		
		chas->info.Direction = CHAS_FORWARD;
		position = RP_Limit(position - chas->info.OriginAngle,360);
		chas->data.DirAngle = position;
	}
	
	//������������ٶȼ���
	chas->data.VelocityMax = chas->data.WheelrpmMax/chas->info.ReductionRatio/60.0f
	                         *2.0f*pi*chas->info.WheelRadius;
	
	//���̵���ٶȸ���
	chas->data.WheelReal[0] = motor[CHAS_1].rx_info.speed;
  chas->data.WheelReal[1] = motor[CHAS_2].rx_info.speed;
	chas->data.WheelReal[2] = motor[CHAS_3].rx_info.speed;
	chas->data.WheelReal[3] = motor[CHAS_4].rx_info.speed;
	
	//Ϲд��
	chas->data.VelocityReal.x = chas->data.WheelReal[0]-chas->data.WheelReal[2]+chas->data.WheelReal[1]-chas->data.WheelReal[3];
	chas->data.VelocityReal.y =-chas->data.WheelReal[0]-chas->data.WheelReal[2]+chas->data.WheelReal[1]+chas->data.WheelReal[3];
	chas->data.VelocityReal.z =-chas->data.WheelReal[0]-chas->data.WheelReal[2]-chas->data.WheelReal[1]-chas->data.WheelReal[3];
	
}


/**
  * �������ӵ�ת�� 
  */
void Chassis_Resolving(chassis *chas)
{
	float velocity[4];//velocityAbsoluteMax;
	
	if(chas->info.Type == CHAS_OMNI){
    
		velocity[0] = chas->data.VelocitySet.x + chas->data.VelocitySet.y - chas->data.VelocitySet.z;
		velocity[1] = chas->data.VelocitySet.x - chas->data.VelocitySet.y - chas->data.VelocitySet.z;	
		velocity[2] =-chas->data.VelocitySet.x + chas->data.VelocitySet.y - chas->data.VelocitySet.z;
		velocity[3] =-chas->data.VelocitySet.x - chas->data.VelocitySet.y - chas->data.VelocitySet.z;
	}
	else if(chas->info.Type == CHAS_MECA){
    
		velocity[0] = chas->data.VelocitySet.x + chas->data.VelocitySet.y - chas->data.VelocitySet.z;
		velocity[1] = chas->data.VelocitySet.x - chas->data.VelocitySet.y - chas->data.VelocitySet.z;	
		velocity[2] =-chas->data.VelocitySet.x + chas->data.VelocitySet.y - chas->data.VelocitySet.z;
		velocity[3] =-chas->data.VelocitySet.x - chas->data.VelocitySet.y - chas->data.VelocitySet.z;
	}
	else if(chas->info.Type == CHAS_HELM){
    
	}	
	
	for(char i = 0 ; i < 4 ; i++)
	{
		chas->data.WheelSet[i] = velocity[i]/100 * chas->data.WheelrpmMax;
	}	
	
	
}


/**
  * �������ӵ�Ť�� ���ʿ���
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
  * �����㷨
  */
void Chassis_Power_Limit(chassis *chas,int16_t *data)
{
	float buffer = judge.data.power_heat_data.chassis_power_buffer;
	
	float heat_rate, Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;
	
	uint16_t OUT_MAX = 0;
	
	if(judge.info.state == CAP_ONLINE && cap.info.state == CAP_ONLINE){
	
		OUT_MAX = 8000 * 4;
	}
	else{
	
		OUT_MAX = 4000 * 4;
	}
	
	if(buffer > 60)buffer = 60;//��ֹ����֮�󻺳�250J��Ϊ������ϵ��
	
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



















