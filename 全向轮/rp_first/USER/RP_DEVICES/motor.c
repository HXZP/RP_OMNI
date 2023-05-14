/*
* EXAMPLE
*	RP MOTOR_BAG
*	2022.11.6 
* AUTHOR CCB HXZP
*
*使用电机前必须配置好电机的类型、id号、驱动方式，以及init指针函数初始化
*使用pid前也需要进行pid初始化，否则无法使用
*可以直接使用内置函数进行pid控制 也可以直接使用库内定义函数或者自己定义的函数


使用步骤：
1、motor_list_e电机列表中添加电机名称，添加电机要排在MOTOR_TEST前
2、配置好电机的驱动方式、类型、id号、初始化函数
3、设置pid参数列表，初始化pid
4、定义发送数组，愉快玩耍


		以下三个为自带pid控制计算，自动调用以下pid组，要自己初始pid
		motor_pid_t            speed;
	
		motor_pid_t            angle;	
		motor_pid_t            angle_in;		
	
		motor_pid_t            position;
		motor_pid_t            position_in;	
		
		target值单位分别为
		8192 
		8192(函数内计算自动转360制) 
		rpm 

	float (*c_posit)(struct motor_class_t *motor,float target);
	float (*c_angle)(struct motor_class_t *motor,float target);	
	float (*c_speed)(struct motor_class_t *motor,float target);	
	

		用户自定义pid
		err_cal_mode为err处理方式， 0，1，2：一圈、半圈、四分之一圈
		meas 使用360度制 电机角度8192这里不会自动转360

	float (*c_pid2)(motor_pid_t *out, motor_pid_t *inn, float meas1, float meas2, float tar, char err_cal_mode);	
	float (*c_pid1)(motor_pid_t *out, float meas1, float tar);	
	
关于发送：
电机结构体内自带了发送函数，
但需要自己定义发送的数组，
并且处于同一数组的电机数据会统一发送，因此只要调用一次发送四个电机数据，请合理安排电机id
当数组数据都为0时不会发送，发送结束后数据包会清零，无需重复发送


*/
#include "RP_CONFIG.h"
#include "motor.h"


/*----------------------------------------用户接口begin----------------------------------------*/


/*
* 使用电机前必须配置好电机的类型、id号、驱动方式
*/
motor_t motor[MOTOR_LIST] =
{
	[CHAS_1] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = RM3508,
		.id.rx_id      = 0x201,

		.init = &motor_class_init,
	},
	[CHAS_2] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = RM3508,
		.id.rx_id      = 0x202,

		.init = &motor_class_init,
	},
	[CHAS_3] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = RM3508,
		.id.rx_id      = 0x203,

		.init = &motor_class_init,
	},
	[CHAS_4] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = RM3508,
		.id.rx_id      = 0x204,

		.init = &motor_class_init,
	},	
	
	
	[GIMB_Y] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = GM6020,
		.id.rx_id      = 0x205,

		.init = &motor_class_init,
		
		.mec_info.mec_mid = MOTOR_YAW_MID,
		
	},
	[GIMB_P] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = GM6020,
		.id.rx_id      = 0x206,
		
		.init = &motor_class_init,
		
		.mec_info.mec_mid = MOTOR_PIT_MID,
	},

	[FRI_R] = {
	
		.id.drive_type = M_CAN2,
		.id.motor_type = RM3508,
		.id.rx_id      = 0x201,

		.init = &motor_class_init,
	},	
	[FRI_L] = {
	
		.id.drive_type = M_CAN2,
		.id.motor_type = RM3508,
		.id.rx_id      = 0x202,

		.init = &motor_class_init,
	},	
	[BOX] = {
	
		.id.drive_type = M_CAN2,
		.id.motor_type = RM3508,
		.id.rx_id      = 0x203,

		.init = &motor_class_init,
	},
	
	[MOTOR_TEST] = {
	
		.id.drive_type = (motor_drive_e)RM_MOTOR_CAN_TYPR_TEST,
		.id.motor_type = (motor_type_e)RM_MOTOR_TYPE_TEST,
		.id.rx_id      = (motor_id_e)RM_MOTOR_CAN_ID_TEST,
		
		.init = &motor_class_init,
	}
};

/*--------------------------------------------------------------------------*/

/*pid参数列表*/

/*
	需要定义一个数组作为传入参数，不使用结构体是因为数组初始化展开为一维的看起来比较简约，但是要记住他们的位置和含义
	float	  kp;
	float 	ki;
	float 	kd;
	
	float  blind_err;	
	float  integral_max;	
	float  iout_max;
	float  out_max;
*/
float  test_speed_pid_param[7] = {5,0,0,0,0,15000,20000};

float chassis_speed_pid_param[7] = {8,0.6,0,0,5000,5000,8000};

float yaw_imu_out_pid_param[7] = {15,0,0,0,0,15000,20000};
float yaw_imu_inn_pid_param[7] = {100,0.3,0,0,15000,20000,20000};

float pit_imu_out_pid_param[7] = {20,0,0,0,0,15000,20000};
float pit_imu_inn_pid_param[7] = {300,0.2,0,0,15000,20000,20000};

float yaw_speed_pid_param[7] = {0,0,0,0,15000,20000,20000};
float pit_speed_pid_param[7] = {0,0,0,0,15000,20000,20000};

void RM_MotorInit(void)
{
	for(uint16_t i = 0;i < (uint16_t)MOTOR_LIST;i++)
	{
		motor[i].init(&motor[i]);
	}
	/*--------------------------------------------------------------*/
	/*电机速度pid初始化*/
	
	motor[MOTOR_TEST].pid_init(&motor[MOTOR_TEST].pid.speed,test_speed_pid_param);

	motor[CHAS_1].pid_init(&motor[CHAS_1].pid.speed,chassis_speed_pid_param);
	motor[CHAS_2].pid_init(&motor[CHAS_2].pid.speed,chassis_speed_pid_param);
	motor[CHAS_3].pid_init(&motor[CHAS_3].pid.speed,chassis_speed_pid_param);
	motor[CHAS_4].pid_init(&motor[CHAS_4].pid.speed,chassis_speed_pid_param);
	
	motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.speed,   yaw_imu_out_pid_param);
	motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.angle,   yaw_imu_out_pid_param);
	motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.angle_in,yaw_imu_inn_pid_param);	
	
	motor[GIMB_P].pid_init(&motor[GIMB_P].pid.speed,   pit_imu_out_pid_param);
	motor[GIMB_P].pid_init(&motor[GIMB_P].pid.angle,   pit_imu_out_pid_param);
	motor[GIMB_P].pid_init(&motor[GIMB_P].pid.angle_in,pit_imu_inn_pid_param);	
}


/*
	电机测试代码
*/
float targetTest = 300;
int16_t SendBuffTest[4];

void RM_MotorControl_Test(void)
{
		//pid计算
		SendBuffTest[motor[MOTOR_TEST].id.buff_p] = motor[MOTOR_TEST].c_speed(&motor[MOTOR_TEST],targetTest);

	  //控制数据的发送
		motor[MOTOR_TEST].tx(&motor[MOTOR_TEST],SendBuffTest);
	

}



/*----------------------------------------用户接口end----------------------------------------*/






/*----------------------------------------底层自动控制----------------------------------------*/




char RM_MotorHeartBeat(void)
{
	char offline_cnt = 0;
	
	for(uint16_t i = 0;i < (uint16_t)MOTOR_LIST;i++)
	{
		motor[i].heartbeat(&motor[i]);

		if(motor[i].state.work_state == M_OFFLINE)offline_cnt++;
	}
	
	//去掉测试电机的干扰
	if(motor[MOTOR_LIST-1].state.work_state == M_OFFLINE && offline_cnt)offline_cnt -= 1;
		
	return offline_cnt;
	
}


/*

电机接收数据接口

*/

void RM_MotorCAN1(uint32_t canId, uint8_t *rxBuf)
{
	for(uint16_t i = 0;i < (uint16_t)MOTOR_LIST;i++)
	{
		motor[i].rx(&motor[i],rxBuf,canId,M_CAN1);
	}
	
	motor[GIMB_Y].c_offset(&motor[GIMB_Y],8192);
	motor[GIMB_P].c_offset(&motor[GIMB_P],8192);
	
}

void RM_MotorCAN2(uint32_t canId, uint8_t *rxBuf)
{
	for(uint16_t i = 0;i < (uint16_t)MOTOR_LIST;i++)
	{
		motor[i].rx(&motor[i],rxBuf,canId,M_CAN2);
	}
}







