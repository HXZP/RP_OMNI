#include "chassis.h"
#include "motor.h"



void Chassis_ModifyDirection(chassis *chas)
{



}


void Chassis_ModifyVelocityMax(chassis *chas,float max)
{
	chas->data.VelocityMax = max;
}

void Chassis_Updata(chassis *chas,float setX,float setY,float setZ)
{
	chas->data.WheelReal[0] = motor[CHAS_1].rx_info.speed;
  chas->data.WheelReal[1] = motor[CHAS_2].rx_info.speed;
	chas->data.WheelReal[2] = motor[CHAS_3].rx_info.speed;
	chas->data.WheelReal[3] = motor[CHAS_4].rx_info.speed;
	
	
//	chas->data.VelocityReal.x = 
//	chas->data.VelocityReal.y = 
//	chas->data.VelocityReal.z = 
	
	chas->data.VelocitySet.x = setX;
	chas->data.VelocitySet.y = setY;
  chas->data.VelocitySet.z = setZ;
}


void Chassis_Resolving(chassis *chas)
{


	
	
}

void Chassis_Ctrl(chassis *chas)
{
	if(chas->info.Lock == CHAS_LOCK)
	{
		return;
	}


	
	
}
























