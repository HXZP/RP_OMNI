#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/*--------------------------------------

陀螺仪坐标系：
power为主控电源位置，按下图摆放找坐标系

	y
	|
	|
	z----x
 
小主控：
正面射出z轴，朝右x轴，朝上y轴
 ------------
|            | 
|power       |
|            |
 ------------

大主控：
背面射出z轴，朝右x轴，朝上y轴
 ------------
|            | 
|            |
|power       |
|            |
|            |
 ------------
全向轮功能拓扑：

底盘云台：机械模式、imu模式、小陀螺、视觉

发射：连发、单发、定数发射

自瞄：手打、自动打


代码思路：
文件从下到上层层包含
上面的文件、变量不出现在下层
上层控制下层数据不进行直接控制，使用接口函数


-----------------------------------------*/



/* 0上主控 1下主控  */
/*MASTER SERIAL NUMBER*/

#define MASTER 0U

/* imu 装配姿态  */
#if MASTER == 0U

	#define RP_CENTER 1

	#define IMU_POSE_ANGLE -90
	#define IMU_POSE_AX    0
	#define IMU_POSE_AY    0
	#define IMU_POSE_AZ    1
	#define IMU_PID_KP     10.f
	
	#define IMU_PID_KP_CONTROL     0.1f
	
	#define IMU_POTOCAL_TYPE IMU_SPI
	
#endif

#if MASTER == 1U

	#define IMU_POSE_ANGLE -90
	#define IMU_POSE_AX    0
	#define IMU_POSE_AY    0
	#define IMU_POSE_AZ    1
	#define IMU_PID_KP     10.f	
	
	#define IMU_PID_KP_CONTROL     1.f
	
	#define IMU_POTOCAL_TYPE IMU_SPI
	
#endif




/*数据*/
#define MOTOR_YAW_MID 5869
#define MOTOR_PIT_MID 1200

#define WHEEL_SPEED_MAX 8000
#define WHEEL_POWER_MAX 8000




/*开启中心控制*/
#define CENTER_GLOBAL 1U 



/*Devices Frivers Enable*/
#define BMI_ENABLE    1U		
#define CAN_ENABLE    1U
#define USART_ENABLE  1U

/*Usart Select*/
#define VISION_USART  1U
#define JUDGE_USART   5U

/*Test Enable*/
#define USART_TEST    0U //Enable Usart Test

#define RM_MOTOR_TEST            0U //Enable Motor Test
#define RM_MOTOR_CAN_TYPR_TEST   1U //1 CAN1 2 CAN2
#define RM_MOTOR_CAN_ID_TEST     0x205U //id
#define RM_MOTOR_TYPE_TEST       1U //1 6020 2 3508 3 2006

/*键盘读取使能*/
#define RC_KEY_MONITOR       1U 




#endif




