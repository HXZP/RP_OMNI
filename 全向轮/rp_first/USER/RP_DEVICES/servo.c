#include "servo.h"

//#define SERVO_CCR_1	  TIM1->CCR1

extern TIM_HandleTypeDef htim1;

static void Servo_Weak(servo *self);
static void Servo_Sleep(servo *self);
static void Servo_ModifyCCR(servo *self,uint32_t ccr);


servo magazine = {

	.htim = &htim1,
  .Channel = TIM_CHANNEL_1,
	
	.sleep = Servo_Sleep,
	.weak = Servo_Weak,
	.modifyCCR = Servo_ModifyCCR,
	

};





void Servo_Weak(servo *self)
{
	HAL_TIM_Base_Start_IT(self->htim);

	HAL_TIM_PWM_Start(self->htim, self->Channel);
}



void Servo_Sleep(servo *self)
{
	HAL_TIM_PWM_Stop(self->htim, self->Channel);
}



void Servo_ModifyCCR(servo *self,uint32_t ccr)
{
	if(self->Channel == TIM_CHANNEL_1){
	
		self->htim->Instance->CCR1 = ccr;
	}
	if(self->Channel == TIM_CHANNEL_2){
	
		self->htim->Instance->CCR2 = ccr;
	}
	if(self->Channel == TIM_CHANNEL_3){
	
		self->htim->Instance->CCR3 = ccr;
	}	
	if(self->Channel == TIM_CHANNEL_4){
	
		self->htim->Instance->CCR4 = ccr;
	}	
}





