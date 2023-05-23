#include "rp_math.h"


int RP_GetSymbol(float num)
{
	int symbol;
	if     (num > 0)symbol = +1;
	else if(num < 0)symbol = -1;
	else if(num== 0)symbol =  0;
	
	return symbol;
}


float RP_JudgeNull(float last, float now)
{
	if(last != 0)
	{
		if(now == NULL)
		{
			now = last;
		}
	}
	return now;
}


/*
		限制目标值 防止超过
*/
float RP_Limit(float tar,float range)
{
	while(abs(tar) > range){
	
		if(tar < 0)          tar += range;
		else if(tar >=range) tar -= range;	
	}
	
	if(tar < 0)          tar += range;
	else if(tar >=range) tar -= range;		

	
	return tar;
}




float RP_HalfTurn(float angle,float max)
{
	if(abs(angle) > (max/2))
	{	
		if(angle >= 0)
			angle += -max;		
		else
			angle +=  max;
	}
	return angle;
}


//返回值是绝对值
float RP_GetAbsoluteMax(float *data,uint16_t num)
{
	float max = 0;
	
	
	for(uint16_t i = 0 ; i < num ; i++)
	{
		if(abs(data[i]) > max)max = abs(data[i]);		
	}

	return max;
}

float RP_GetTotal(float *data,uint16_t num)
{
	float total = 0;
	
	
	for(uint16_t i = 0 ; i < num ; i++)
	{
		total += data[i];
	}

	return total;
}

float RP_GetAbsoluteTotal(float *data,uint16_t num)
{
	float total = 0;
	
	
	for(uint16_t i = 0 ; i < num ; i++)
	{
		total += abs(data[i]);
	}

	return total;
}


