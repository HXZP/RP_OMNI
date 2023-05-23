/*
 *
 *@note 先初始化设备再初始化中断
 *
**/
#include "RP_INIT.h"
#include "RP_CONFIG.h"

void RP_IMU_INIT(void);



void RP_INIT(void)
{
	
#if (BMI_ENABLE == 1U)		
	
	RP_IMU_INIT();//must first
	HAL_Delay(100);
	
#endif
	
	
	RM_MotorInit();
	
	rc.init(&rc);
	judge.init(&judge);
	vision.init(&vision);	
	magazine.weak(&magazine);	
	
#if (USART_ENABLE == 1U)		
	
	USART1_Init();
	USART2_Init();
	USART3_Init();
	USART4_Init();
	USART5_Init();
	
#endif

	
#if (CAN_ENABLE == 1U)

	CAN1_Init();
	CAN2_Init();
	
#endif	



	

}


void RP_IMU_INIT(void)
{
	if(imu.info.tpye == IMU_SPI)Imu_SPI_Init();
	if(imu.info.tpye == IMU_IIC)IIC_Init();
	
	imu.init(&imu);
}


