#ifndef __MASTER_H
#define __MASTER_H

#include "type.h"
#include "string.h"
typedef enum
{
	M0 = 0,
	M1,
	
	MASTER_LIST,
	
}master_list_e;


typedef enum{

	MASTER_ONLINE,
	MASTER_OFFLINE,

}MASTER_State_e;

typedef enum{

	MASTER_CAN1,
	MASTER_CAN2,
	
	MASTER_USART1,
	MASTER_USART2,
	MASTER_USART3,
	
}MASTER_DriveType_e;

typedef struct master_info_struct {
	
	
	uint8_t   offline_cnt;
	uint8_t   offline_max_cnt;

	MASTER_DriveType_e  drive;
	MASTER_State_e      state;
	
} MASTER_Info_t;


typedef struct{
	
	int16_t x;
	int16_t y;
	int16_t z;
	
} MASTER_pack1_t;

typedef struct{
	
	float x;
	float y;
	float z;
	
} MASTER_pack1_solve_t;

typedef struct{
	
	uint16_t coolingLimit;
	
	uint16_t speedLimit;
	
  uint16_t powerLimit;	
	
	uint8_t  robot_id;
	
} MASTER_pack2_t;

typedef struct{
	
	uint16_t cooling_heat;

	uint16_t buffer;
	
	float bulletSpeed;
	
} MASTER_pack3_t;


typedef struct master_data_struct {
	
	MASTER_pack1_t       imuRPY;
  MASTER_pack1_solve_t chasRPY;
	
	MASTER_pack2_t judge1;
	MASTER_pack3_t judge2;
	
} MASTER_data_t;

typedef struct master_struct{

	MASTER_Info_t info;
	MASTER_data_t data;
	
	void	(*updata)(struct master_struct *self,uint8_t *buff,uint32_t canId);
	void	(*heart_beat)(struct master_struct *self);


}master_t;


extern master_t master[MASTER_LIST];


void MASTER_sendBuff(void);
void MASTER_HeartBeat(void);







#endif



