/*

主控间通讯

*/

#include "RP_CONFIG.h"
#include "DEVICES.h"
#include "DRIVERS.h"

#include "string.h"


static void master_heart_beat(struct master_struct *self);
static void master_updata(struct master_struct *self,uint8_t *buff,uint32_t canId);

master_t master[MASTER_LIST] = {

	[M0] = {
	
		.info.drive = MASTER_CAN2,
		
		.info.state = MASTER_OFFLINE,	
		.info.offline_max_cnt = 100,	
		
		.updata     = &master_updata,
		.heart_beat = &master_heart_beat,
	},

	[M1] = {
	
		.info.drive = MASTER_CAN2,
		
		.info.state = MASTER_OFFLINE,	
		.info.offline_max_cnt = 100,	
		
		.updata     = &master_updata,
		.heart_beat = &master_heart_beat,
	},
	
};




void master_updata(struct master_struct *self,uint8_t *buff,uint32_t canId)
{
#if (MASTER == 0)
	
	if(canId == 0x100){
	
		memcpy(&self->data.imu_q,buff,sizeof(self->data.imu_q));
	
		self->info.offline_cnt = 0;
	}
	
#endif



}

void MASTER_sendBuff(void)
{
	
#if (MASTER == 1)
	
	uint8_t buff[8];
	
	master[M1].data.imu_q.q0 = imu.data.q.q0*30000.f;
	master[M1].data.imu_q.q1 = imu.data.q.q1*30000.f;
	master[M1].data.imu_q.q2 = imu.data.q.q2*30000.f;
	master[M1].data.imu_q.q3 = imu.data.q.q3*30000.f;
	
	memcpy(buff,&master[M1].data.imu_q,sizeof(master[M1].data.imu_q));
	
	CAN_SendUint8(0x100,buff,2,8);
	
#endif



}


void MASTER_HeartBeat(void)
{
	for(int16_t i = 0;i<MASTER_LIST;i++){
	
	  master[i].heart_beat(&master[i]);
	}

}



static void master_heart_beat(struct master_struct *self)
{
	MASTER_Info_t *info = &self->info;

	info->offline_cnt++;
	if(info->offline_cnt > info->offline_max_cnt)//每次等待一段时间后自动离线
	{
		info->offline_cnt = info->offline_max_cnt;
		info->state = MASTER_OFFLINE;
	} 
	else //每次接收成功就清空计数
	{
		/* 离线->在线 */
		if(info->state == MASTER_OFFLINE)
		{
			info->state = MASTER_ONLINE;
		}
	}
}	

