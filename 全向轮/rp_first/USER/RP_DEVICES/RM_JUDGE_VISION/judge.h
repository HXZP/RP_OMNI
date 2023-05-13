#ifndef __JUDGE_H
#define __JUDGE_H

/* Includes ------------------------------------------------------------------*/
#include "judge_def.h"
#include "stdbool.h"
#include "crc.h"






//此为帧头的SOF,帧头分为 SOF,length,Seq,CRC8
#define JUDGE_FRAME_HEADER	0xA5  

/**********偏移位置**********/
//帧字节（单位/字节）
//帧头占五个字节，命令码占两个字节，从第七个开始为数据帧
enum judge_frame_offset_t {
	FRAME_HEADER	= 0,
	CMD_ID			  = 5,
	DATA_SEG		  = 7
};

//帧头字节（单位/bit）
enum judge_frame_header_offset_t {//帧头的详细位置
	SOF			    = 0,
	DATA_LENGTH	= 1,
	SEQ			    = 3,
	CRC8		    = 4
};

typedef enum {
	
	JUDGE_ONLINE,
	JUDGE_OFFLINE,

	JUDGE_INIT_ERR,	
	JUDGE_DATA_ERR,
	
} Judge_State_e;



typedef struct judge_data_struct{

	std_frame_header_t							fream_header;				// 帧头信息
	
	ext_game_status_t 							game_status;				// 0x0001
	ext_game_result_t 							game_result;				// 0x0002
	ext_game_robot_HP_t 						game_robot_HP;			// 0x0003
//	ext_dart_status_t								dart_status;				// 0x0004
	ext_ICRA_buff_debuff_zone_status_t	ICRA_buff;
	
	ext_event_data_t								event_data;					// 0x0101
	ext_supply_projectile_action_t	supply_projectile_action;		// 0x0102
	//ext_supply_projectile_booking_t supply_projectile_booking;// 0x0103
	ext_referee_warning_t						referee_warning;		// 0x0104
	ext_dart_remaining_time_t				dart_remaining_time;// 0x0105
	
	ext_game_robot_status_t					game_robot_status;	// 0x0201
	ext_power_heat_data_t						power_heat_data;		// 0x0202
	ext_game_robot_pos_t						game_robot_pos;			// 0x0203
	ext_buff_t											buff;								// 0x0204
	ext_aerial_robot_energy_t				aerial_robot_energy;// 0x0205
	ext_robot_hurt_t								robot_hurt;					// 0x0206
	ext_shoot_data_t								shoot_data;					// 0x0207
	ext_bullet_remaining_t					bullet_remaining;		// 0x0208	
	ext_rfid_status_t								rfid_status;				// 0x0209	
	ext_dart_client_cmd_t           dart_client;        // 0x020A
	
	ext_interact_id_t								ids;								//与本机交互的机器人id
	
	uint16_t                        self_client;        //本机客户端
	ext_sentry_t                    sentry_Info_rescue; //哨兵提供的信息
	
	bool	 		data_valid;	// 数据有效性
	bool			err_cnt;
	
}judge_data_t;

typedef struct judge_info_struct{

	Judge_State_e      state;
	
	uint8_t		 init_flag;
	uint16_t   offline_cnt;
	uint16_t   offline_max_cnt;

}judge_info_t;


typedef struct judge_struct{

	judge_info_t info;
	judge_data_t data;

	void		(*init)(struct judge_struct *self);
	void		(*update)(struct judge_struct *self, uint8_t *rxBuf);
	void		(*heart_beat)(struct judge_struct *self);
	void		(*send)(struct judge_struct *self,uint8_t *Data);
	
	
}judge_t;

extern judge_t	judge;
/* Exported functions --------------------------------------------------------*/

#endif
