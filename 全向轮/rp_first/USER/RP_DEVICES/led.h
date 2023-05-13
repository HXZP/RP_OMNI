#ifndef __LED_H
#define __LED_H

#include "gpio_drv.h"
#include "stdbool.h"

void Led_All_Light(void);
void Led_All_Shine(uint16_t time);
void Led_Running(uint16_t time);
void Led_Breath(uint16_t time);

typedef struct led_struct {
	
	void (*allLight)(void);
	void (*allShine)(uint16_t time);
	void (*running)(uint16_t time);
//	void (*breath)(uint16_t time);
	
} led_t;

extern led_t led;

#endif


















