#include "led.h"


led_t led = {

	.allLight = &Led_All_Light,
	.allShine = &Led_All_Shine,
  .running  = &Led_Running,
//	.breath   = &Led_Breath,
	
};

void Led_All_Light(void)
{
	HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
}

void Led_All_Shine(uint16_t time)
{
	static uint16_t shine_cnt = 0;
	
	shine_cnt++;
	
	if(shine_cnt == time){
		
		HAL_GPIO_TogglePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin);
		shine_cnt = 0;
	}
}

void Led_Running(uint16_t time)
{
	static uint16_t running_cnt = 0;
	
	running_cnt++;
	
	if(running_cnt == time){
		
		HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
	}
	else if(running_cnt == time*2){
		
		HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
	}	
	else if(running_cnt == time*3){
		
		HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);
		running_cnt = 0;
	}		
	
	
}




