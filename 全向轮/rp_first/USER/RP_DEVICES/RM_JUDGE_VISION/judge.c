/**
 * @file        judge_sensor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        5-November-2020
 * @brief       Device Judge.
 
 
 裁判系统的串口接口要接在靠近电源管理上靠近航空线的那一端接口 
 
 
 
 */
 
 
 
 
/* Includes ------------------------------------------------------------------*/
#include "RP_CONFIG.h"

#include "judge.h"
#include "string.h"
#include "uart_drv.h"


static void judge_sensor_init(judge_t *judge_sen);
static void judge_sensor_update(judge_t *judge_sen, uint8_t *rxBuf);
static void judge_sensor_heart_beat(judge_t *judge_sen);
static void judge_sendBuff(struct judge_struct *self,uint8_t *Data);


void Determine_ID(void);


judge_t	judge = {
	
	.info.state  = JUDGE_OFFLINE,	
	.info.offline_cnt     = 0,
	.info.offline_max_cnt = 5000,

	
	.init       = judge_sensor_init,
	.update     = judge_sensor_update,
	.heart_beat = judge_sensor_heart_beat,
	.send       = judge_sendBuff,

};




void judge_sensor_init(judge_t *judge_sen)
{
	judge_sen->info.offline_cnt = judge_sen->info.offline_max_cnt+1;
	judge_sen->info.state= JUDGE_OFFLINE;
}



/**
 *	@brief	裁判系统心跳包
 */
static void judge_sensor_heart_beat(judge_t *judge_sen)
{
	judge_sen->info.offline_cnt++;
	
	if(judge_sen->info.offline_cnt > judge_sen->info.offline_max_cnt) {
		judge_sen->info.offline_cnt = judge_sen->info.offline_max_cnt;
		judge_sen->info.state = JUDGE_OFFLINE;
	} 
	else {
		/* 离线->在线 */
		if(judge_sen->info.state == JUDGE_OFFLINE)
			judge_sen->info.state = JUDGE_ONLINE;
	}
}


void judge_sendBuff(struct judge_struct *self,uint8_t *Data)
{
	UART_Send(Data,JUDGE_USART);
}



void judge_sensor_update(judge_t *judge_sen, uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint16_t frame_length;
	uint16_t cmd_id;
//  uint16_t data_cmd_id;
	
	judge_data_t *judge_data = &judge_sen->data;


	if( rxBuf == NULL )
	{
		judge_data->data_valid = false;
		return;
	}
	
	memcpy(&judge_data->fream_header, rxBuf, LEN_FRAME_HEAD);//5个字节
	
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		/* 帧头CRC8校验 */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* 统计一帧的总数据长度，用于CRC16校验 */     // 长度两个字节
			frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_data->fream_header.data_length + LEN_FRAME_TAIL;
			
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
			{
				res = true;
				
				cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
				
				switch(cmd_id)
				{
//					case 0x0200:
//					memcpy(&judge_data->ext, (rxBuf+DATA_SEG), 2);
//						break;
//					
//					case ID_robot_interactive_header_data:
//					{
//						data_cmd_id = (rxBuf[DATA_SEG+1] << 8 | rxBuf[DATA_SEG]);
//						switch (data_cmd_id)
//						{
//							break;
//						}
//					}
					
					case ID_game_state: 
						memcpy(&judge_data->game_status, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
						break;
					
					case ID_game_result: 
						memcpy(&judge_data->game_result, (rxBuf+DATA_SEG), LEN_GAME_RESULT);
						break;
					
					//所有机器人的血量都可以获得
					case ID_game_robot_HP: 
						memcpy(&judge_data->game_robot_HP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
						break;
					
//					case ID_dart_status: 
//						memcpy(&judge_data->dart_status, (rxBuf+DATA_SEG), LEN_DART_STATUS);
//						break;
//					
					case ID_ICRA_buff_debuff_zone_status:
						memcpy(&judge_data->ICRA_buff,(rxBuf+DATA_SEG),LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS);
						break;
					
					case ID_event_data: 
						memcpy(&judge_data->event_data, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
						break;
					
					case ID_supply_projectile_action: 
						memcpy(&judge_data->supply_projectile_action, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
						break;
					
					case ID_referee_warning: 
						memcpy(&judge_data->referee_warning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
						break;
					
					case ID_dart_remaining_time: 
						memcpy(&judge_data->dart_remaining_time, (rxBuf+DATA_SEG), LEN_DART_REMAINING_TIME);
						break;
						
					case ID_game_robot_state: 
						memcpy(&judge_data->game_robot_status, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
						Determine_ID();
						break;
					
					case ID_power_heat_data: 
						memcpy(&judge_data->power_heat_data, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
						break;
					
					case ID_game_robot_pos: 
						memcpy(&judge_data->game_robot_pos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
						break;
					
					case ID_buff_musk: 
						memcpy(&judge_data->buff, (rxBuf+DATA_SEG), LEN_BUFF_MASK);
						break;
					
					case ID_aerial_robot_energy: 
						memcpy(&judge_data->aerial_robot_energy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
						break;
					
					case ID_robot_hurt: 
						memcpy(&judge_data->robot_hurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
						break;
					
					case ID_shoot_data: 
						memcpy(&judge_data->shoot_data, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
						break;
					
					case ID_bullet_remaining: 
						memcpy(&judge_data->bullet_remaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
						break;
					
					case ID_rfid_status: 
						memcpy(&judge_data->rfid_status, (rxBuf+DATA_SEG), LEN_RFID_STATUS);
						break;
						
					case ID_dart_client_directive:
						memcpy(&judge_data->dart_client,(rxBuf+DATA_SEG),LEN_DART_CLIENT_DIRECTIVE);
						break;
					
//					case ID_COMMUNICATION: 
//						//JUDGE_ReadFromCom();
//						break;
				}
			}
		}
		
		/* 帧尾CRC16下一字节是否为0xA5 */
		if(rxBuf[frame_length] == JUDGE_FRAME_HEADER)
		{
			/* 如果一个数据包出现了多帧数据就再次读取 */
			judge_sensor_update( judge_sen, &rxBuf[frame_length] );
		}
	}
	
	judge_data->data_valid = res;
	if(judge_data->data_valid != true)
		judge_data->err_cnt++;
	
	// 接收到数据表示在线
	judge_sen->info.offline_cnt = 0;
	
	
}


//判断自己是哪个队伍
void Determine_ID(void)
{
	if(judge.data.game_robot_status.robot_id < 10)//本机器人的ID，红方
	{ 
		judge.data.ids.teammate_hero 		 	= 1;
		judge.data.ids.teammate_engineer  = 2;
		judge.data.ids.teammate_infantry3 = 3;
		judge.data.ids.teammate_infantry4 = 4;
		judge.data.ids.teammate_infantry5 = 5;
		judge.data.ids.teammate_plane		 	= 6;
		judge.data.ids.teammate_sentry		= 7;
		
		judge.data.ids.client_hero 		 	= 0x0101;
		judge.data.ids.client_engineer  = 0x0102;
		judge.data.ids.client_infantry3 = 0x0103;
		judge.data.ids.client_infantry4 = 0x0104;
		judge.data.ids.client_infantry5 = 0x0105;
		judge.data.ids.client_plane			= 0x0106;
		
		if     (judge.data.game_robot_status.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
			judge.data.self_client = judge.data.ids.client_hero;
		else if(judge.data.game_robot_status.robot_id == engineer_red)
			judge.data.self_client = judge.data.ids.client_engineer;
		else if(judge.data.game_robot_status.robot_id == infantry3_red)
			judge.data.self_client = judge.data.ids.client_infantry3;
		else if(judge.data.game_robot_status.robot_id == infantry4_red)
			judge.data.self_client = judge.data.ids.client_infantry4;
		else if(judge.data.game_robot_status.robot_id == infantry5_red)
			judge.data.self_client = judge.data.ids.client_infantry5;
		else if(judge.data.game_robot_status.robot_id == plane_red)
			judge.data.self_client = judge.data.ids.client_plane;
	}
	else //蓝方
	{
		judge.data.ids.teammate_hero 		 	= 101;
		judge.data.ids.teammate_engineer  = 102;
		judge.data.ids.teammate_infantry3 = 103;
		judge.data.ids.teammate_infantry4 = 104;
		judge.data.ids.teammate_infantry5 = 105;
		judge.data.ids.teammate_plane		 	= 106;
		judge.data.ids.teammate_sentry		= 107;
		
		judge.data.ids.client_hero 		 	= 0x0165;
		judge.data.ids.client_engineer  = 0x0166;
		judge.data.ids.client_infantry3 = 0x0167;
		judge.data.ids.client_infantry4 = 0x0168;
		judge.data.ids.client_infantry5 = 0x0169;
		judge.data.ids.client_plane			= 0x016A;
		
		if     (judge.data.game_robot_status.robot_id == hero_blue)
			judge.data.self_client = judge.data.ids.client_hero;
		else if(judge.data.game_robot_status.robot_id == engineer_blue)
			judge.data.self_client = judge.data.ids.client_engineer;
		else if(judge.data.game_robot_status.robot_id == infantry3_blue)
			judge.data.self_client = judge.data.ids.client_infantry3;
		else if(judge.data.game_robot_status.robot_id == infantry4_blue)
			judge.data.self_client = judge.data.ids.client_infantry4;
		else if(judge.data.game_robot_status.robot_id == infantry5_blue)
			judge.data.self_client = judge.data.ids.client_infantry5;
		else if(judge.data.game_robot_status.robot_id == plane_blue)
			judge.data.self_client = judge.data.ids.client_plane;

	}

}
