

#include "led.h"
#include "gpio_drv.h"

void Led_All_Light(void);
void Led_All_DeLight(void);
void Led_All_Shine(uint16_t time);
void Led_Running(uint16_t time);
void Led_Breath(uint16_t time);

void Led_Light(char num);
void Led_DeLight(char num);
void Led_Shine(char num,uint16_t sw);



led_t led = {

	.allLight = &Led_All_Light,
	.allDeLight = &Led_All_DeLight,
		
	.allShine = &Led_All_Shine,
  .running  = &Led_Running,
	
  .Light = &Led_Light,
	.DeLight = &Led_DeLight,
	.Shine = &Led_Shine,
};

void Led_All_Light(void)
{
	HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
}

void Led_All_DeLight(void)
{
	HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);
}

void Led_Light(char num)
{
	switch(num){
	
		case 1:
			HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
		  break;
		case 2:
			HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_RESET);
		  break;	
		case 3:
			HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_RESET);
		  break;	
	}
}

void Led_DeLight(char num)
{
	switch(num){
	
		case 1:
			HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
		  break;
		case 2:
			HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);
		  break;	
		case 3:
			HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_SET);
		  break;	
	}
}


void Led_Shine(char num,uint16_t sw)
{
	if(sw){
		
		switch(num){
		
			case 1:
				HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
				break;
			case 2:
				HAL_GPIO_TogglePin(GPIOC, LED2_Pin);
				break;	
			case 3:
				HAL_GPIO_TogglePin(GPIOC, LED3_Pin);
				break;	
		}
	}
	
	

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




