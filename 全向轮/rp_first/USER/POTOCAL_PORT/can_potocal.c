/*
*	 can 通信接收接口
*/

#include "DEVICES.h"
#include "can_potocal.h"


void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	RM_MotorCAN1(canId,rxBuf);
	
	cap.updata(&cap,canId,rxBuf);
	
	
}



void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	RM_MotorCAN2(canId,rxBuf);
	
	cap.updata(&cap,canId,rxBuf);
	
	master[M1].updata(&master[M1],rxBuf,canId);
}


