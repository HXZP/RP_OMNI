#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/*--------------------------------------

����������ϵ��
powerΪ���ص�Դλ�ã�����ͼ�ڷ�������ϵ

	y
	|
	|
	z----x
 
С���أ�
�������z�ᣬ����x�ᣬ����y��
 ------------
|            | 
|power       |
|            |
 ------------

�����أ�
�������z�ᣬ����x�ᣬ����y��
 ------------
|            | 
|            |
|power       |
|            |
|            |
 ------------
ȫ���ֹ������ˣ�

������̨����еģʽ��imuģʽ��С���ݡ��Ӿ�

���䣺��������������������

���飺�ִ��Զ���


����˼·��
�ļ����µ��ϲ�����
������ļ����������������²�
�ϲ�����²����ݲ�����ֱ�ӿ��ƣ�ʹ�ýӿں���


-----------------------------------------*/



/* 0������ 1������  */
/*MASTER SERIAL NUMBER*/

#define MASTER 0U

/* imu װ����̬  */
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




/*����*/
#define MOTOR_YAW_MID 5869
#define MOTOR_PIT_MID 1200

#define WHEEL_SPEED_MAX 8000
#define WHEEL_POWER_MAX 8000




/*�������Ŀ���*/
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

/*���̶�ȡʹ��*/
#define RC_KEY_MONITOR       1U 




#endif




