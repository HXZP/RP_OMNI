/*
*	 usart 通信接收接口
*/
#include "RP_CONFIG.h"
#include "DEVICES.h"
#include "usart_potocal.h"



void USART1_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_Send(rxBuf,1);
	
	//USER
	else{
	
		if(VISION_USART == 1)vision.update(&vision,rxBuf);
	
		if(JUDGE_USART == 1)judge.update(&judge,rxBuf);
	
	
	
	}
}


void USART2_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_Send(rxBuf,2);
	
	//USER
	else{
	
		rc.updata(&rc,rxBuf);
	
	}
}


void USART3_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_Send(rxBuf,3);
	
	//USER
	else{
	
	  if(VISION_USART == 3)vision.update(&vision,rxBuf);
	
		if(JUDGE_USART == 3)judge.update(&judge,rxBuf);
	
	
	
	}
}

void USART4_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_Send(rxBuf,4);
	
	//USER
	else{
	
		if(VISION_USART == 4)vision.update(&vision,rxBuf);
	
		if(JUDGE_USART == 4)judge.update(&judge,rxBuf);	
	
	
	
	}
}



void USART5_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_Send(rxBuf,5);
	
	//USER
	else{
	
		if(VISION_USART == 5)vision.update(&vision,rxBuf);
	
		if(JUDGE_USART == 5)judge.update(&judge,rxBuf);
	
	
	}	

}


