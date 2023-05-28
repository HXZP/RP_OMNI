#ifndef __LED_H
#define __LED_H

#include "type.h"

#include "stdbool.h"


typedef struct led_struct {
	
	void (*allLight)(void);
	void (*allDelight)(void);
	void (*allShine)(uint16_t time);	
	
	void (*Light)(char num);
	void (*DeLight)(char num);	
	void (*Shine)(char num,uint16_t time);

	void (*running)(uint16_t time);

} led_t;

extern led_t led;

#endif


















