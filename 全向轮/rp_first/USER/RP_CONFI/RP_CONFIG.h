#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/*--------------------------------------
说明书：

2023.4.29 HXZP 1324146673@qq.com

驱动部分全是抄马哥的模板代码
陀螺仪部分抄了一些陈子龙的算法

马哥代码链接：
https://gitee.com/HaveALitttleSao/RP2021_HAL_Template/tree/master/

--------------------------------------

全局宏定义控制界面

尽量不要在头文件里包含，要在c文件里包含，负责会造成编译过久

--------------------------------------

DSP使用添加文件路径的方法，需要什么添加什么不要全部加进去，也不要勾选keil的DSP配置

使用cube后，需要用user中的cmsis文件替换drives中的cmsis

--------------------------------------

#define MASTER 1U

主控选择，使能不同的主控，并且切换相匹配的配置以及程序

--------------------------------------

陀螺仪坐标系：
power为主控电源位置，按下图摆放找坐标系
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

 主控坐标绕（x,y,z）旋转angle（角度制）
 	#define IMU_POSE_ANGLE 0 
	
	#define IMU_POSE_AX    1
	#define IMU_POSE_AY    0
	#define IMU_POSE_AZ    0
	
陀螺仪切换通信方式后必须断一次电
主控卡死大概率是imu初始化不成功
部分大主控无法实现SPI通信

-----------------------------------------*/



/* 0上主控 1下主控  */
/*MASTER SERIAL NUMBER*/

#define MASTER 0U

/* imu 装配姿态  */
#if MASTER == 0U
	#define IMU_POSE_ANGLE 0
	#define IMU_POSE_AX    1
	#define IMU_POSE_AY    0
	#define IMU_POSE_AZ    0
	#define IMU_PID_KP     10.f
	
	#define IMU_PID_KP_CONTROL     0.1f
	
	#define IMU_POTOCAL_TYPE IMU_SPI
	
#endif
#if MASTER == 1U
	#define IMU_POSE_ANGLE 180
	#define IMU_POSE_AX    1
	#define IMU_POSE_AY    1
	#define IMU_POSE_AZ    0
	#define IMU_PID_KP     10.f	
	
	#define IMU_POTOCAL_TYPE IMU_IIC
#endif


/*Devices Frivers Enable*/
#define BMI_ENABLE    1U		
#define CAN_ENABLE    1U
#define USART_ENABLE  1U

/*Usart Select*/
#define VISION_USART  1U
#define JUDGE_USART   5U

/*Test Enable*/
#define USART_TEST    0U //Enable Usart Test

#define RM_MOTOR_TEST            1U //Enable Motor Test
#define RM_MOTOR_CAN_TYPR_TEST   1U //1 CAN1 2 CAN2
#define RM_MOTOR_CAN_ID_TEST     0x205U //id
#define RM_MOTOR_TYPE_TEST       1U //1 6020 2 3508 3 2006


#define RC_KEY_MONITOR       1U 




#endif




