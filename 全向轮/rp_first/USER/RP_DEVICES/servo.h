#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include "type.h"




typedef struct servo_struct{

	
	TIM_HandleTypeDef *htim;
	
	uint32_t Channel;
	
	void (*sleep)(struct servo_struct *self);
  void (*weak)(struct servo_struct *self);
		
	void (*modifyCCR)(struct servo_struct *self,uint32_t ccr);

}servo;



extern servo magazine;



#endif


