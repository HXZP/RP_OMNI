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
float RP_LimitTarget(float tar,float range)
{
	if(abs(tar) > range){
	
		if(tar < 0)          tar += range;
		else if(tar >=range) tar -= range;	
	}
	
	return tar;
}




float Half_Turn(float angle,float max)
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

