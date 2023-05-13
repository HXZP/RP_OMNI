/*

任务中心，任务都放在这个文件中

*/


#include "stm32f4xx.h"
#include "cmsis_os.h"

#include "main.h"
#include "arm_math.h"

#include "Tasks_Center.h"
#include "RP_CONFIG.h"
#include "DEVICES.h"
#include "RP_FUNCTION.h"




extern IWDG_HandleTypeDef hiwdg;

void StartMonitorTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {	
		
	  led.running(50);
		
		rc.heart_beat(&rc);
		
		cap.heart_beat(&cap);		
		
	  imu.heart_beat(&imu);
		
		judge.heart_beat(&judge);
		
		vision.heart_beat(&vision);
		
		RM_MotorHeartBeat();
		
		MASTER_HeartBeat();
		
#if (RC_KEY_MONITOR == 1)		
		
		rc.key(&rc);
		
#endif		

    osDelay(1);
  }
  /* USER CODE END 5 */
}



void StartCommunityTask(void const * argument)
{
  /* USER CODE BEGIN StartImuTask */
  /* Infinite loop */
  for(;;)
  {
		imu.updata(&imu);
		
		MASTER_sendBuff();
		
#if MASTER == 0U

		/*等待imu数据收敛切换更低的kp用于控制使用*/
		if(HAL_GetTick() > 1000){
		
			imu.algo.KP = IMU_PID_KP_CONTROL;
		}

#endif		

    osDelay(1);
  }
  /* USER CODE END StartImuTask */
}




void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */

  /* Infinite loop */
  for(;;)
  {
		
		HAL_IWDG_Refresh(&hiwdg);
		
#if (RM_MOTOR_TEST == 1U)		
		
		RM_MotorControl_Test();
		
#endif		

#if (RM_MOTOR_TEST == 0U)		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
#endif		
		
    osDelay(1);
  }
  /* USER CODE END StartControlTask */
}




#if 0
int16_t SendBuff[4];

float yaw_tar = 0;
float pit_tar = 0;
		
		/*关于三轴自吻云台的二轴测试程序*/
		
		if(rc.info.state == RC_ONLINE){
			
			yaw_tar -= ((float)rc.data.ch0)/660;
			pit_tar += ((float)rc.data.ch1)/660;
		}

		//pid计算
		outPut.x = 0;
		outPut.y = 0;
		outPut.z = 0;
		
															 
		outPut.y = 
		motor[GIMB_P].c_pid2(&motor[GIMB_P].pid.angle,
												 &motor[GIMB_P].pid.angle_in,
		                     imu.data.rpy.pitch,
		                     imu.data.worldGyr.y,
												 pit_tar,1);		
		
		outPut.z = 
		motor[GIMB_Y].c_pid2(&motor[GIMB_Y].pid.angle,
												 &motor[GIMB_Y].pid.angle_in,
		                     imu.data.rpy.yaw,
		                     imu.data.worldGyr.z,
												 yaw_tar,1);
												 
		//获取底盘角度
		RP_Quaternion_2_EulerAngle((float)master[M1].data.imu_q.q0/30000,
			                         (float)master[M1].data.imu_q.q1/30000,
				                       (float)master[M1].data.imu_q.q2/30000,
					                     (float)master[M1].data.imu_q.q3/30000, 
			                          &chasRPY.roll,&chasRPY.pitch,&chasRPY.yaw);
		
															 
    //获取电机角度、四元数								 
    gimb.x = 0;
		gimb.y = (float)motor_half_cycle(motor[GIMB_P].rx_info.angle - 1440, 8192)/8192*3.141592654f*2;
    gimb.z = (float)motor_half_cycle(motor[GIMB_Y].rx_info.angle - 1980, 8192)/8192*3.141592654f*2;
														 
//		gimb.y = (float)motor_half_cycle(gimb.y - imu.data.rpy.pitch/180*3.141592654f, 3.14*2);													 
															 
		
    RP_RotationOutput_Chassis2Gimb(chasRPY.roll,chasRPY.pitch,chasRPY.yaw,
															     gimb.x,gimb.y,gimb.z,
															     &outPut.x,&outPut.y,&outPut.z);	
#if 0					
//正常可用
    RP_EulerAngle_2_Quaternion(&motorQ.q0,&motorQ.q1,&motorQ.q2,&motorQ.q3,gimb.x,gimb.y,gimb.z);

    //转换相对云台的底盘姿态  //剔除z轴影响	 
	  chasRPY.yaw = 0;													 
		RP_QuaternionMartix_RightMult(motorQ.q0,
																  motorQ.q1,
																  motorQ.q2,
																  motorQ.q3,	
															 
																 chasRPY.roll,chasRPY.pitch,chasRPY.yaw,
																 &c2gRPY.roll,&c2gRPY.pitch,&c2gRPY.yaw); 
		//获取底盘相对四元数			  														 
		RP_EulerAngle_2_Quaternion(&c2gQ.q0,
															 &c2gQ.q1,
															 &c2gQ.q2,
															 &c2gQ.q3,
															 
															 c2gRPY.roll,c2gRPY.pitch,c2gRPY.yaw);															
													
    //解算实际输出															 
		RP_QuaternionMartix_RightMult(c2gQ.q0,
																  c2gQ.q1,
																  c2gQ.q2,
																  c2gQ.q3,

																 outPut.x,outPut.y,outPut.z,
																 &outPut.x,&outPut.y,&outPut.z);

#endif

    SendBuff[motor[GIMB_P].id.buff_p] = outPut.y;
		SendBuff[motor[GIMB_Y].id.buff_p] = outPut.z;
				
		
		
    //控制数据的发送
		if(rc.info.state == RC_ONLINE && HAL_GetTick()>1000){
		
			motor[GIMB_Y].tx(&motor[GIMB_Y],SendBuff);
		}

		else{
			
		  outPut.x = 0;
			outPut.y = 0;
			outPut.z = 0;
			
			yaw_tar = imu.data.rpy.yaw;
		  pit_tar = 0;
		}
		
#endif		
		







/*-------------------------------------------------------------------------------
草稿纸

		angle_z = (float)(motor[GIMB_Y].rx_info.angle - 1980)/8192*360;
		
		RP_Quaternion_2_EulerAngle(q0,q1,q2,q3,&chas_r,&chas_p,&chas_y);

	  motor_q.q0 = arm_cos_f32(angle_y);
		motor_q.q1 = arm_cos_f32(angle_y)*gimb_x,
		motor_q.q2 = arm_cos_f32(angle_y)*gimb_y,
		motor_q.q3 = arm_cos_f32(angle_y)*gimb_z,		
		
		RP_QuaternionMartix_RightMult(imu.data.q.q0,
																	imu.data.q.q1,
																	imu.data.q.q2,
																	imu.data.q.q3,
																	
																	rollOut,pitchOut,yawOut,
																	&rollOut,&pitchOut,&yawOut);

-------------------------------------------------------------------------
		//获取底盘四元数
		RP_Quaternion_2_EulerAngle((float)master[M0].data.imu_q.q0/30000,
			                         (float)master[M0].data.imu_q.q1/30000,
				                       (float)master[M0].data.imu_q.q2/30000,
					                     (float)master[M0].data.imu_q.q3/30000, 
			                          &chas_r,&chas_p,&chas_y);
	  //剔除z轴影响													 
		RP_EulerAngle_2_Quaternion(&q0,&q1,&q2,&q3,chas_r,chas_p,0);
															 
    //获取电机四元数								 
    angle_x = 0;
		angle_y = (float)motor_half_cycle(motor[GIMB_P].rx_info.angle - 1440,8192)/8192*3.141592654f*2;
    angle_z = 0;//(float)motor_half_cycle(motor[GIMB_Y].rx_info.angle - 1980,8192)/8192*3.141592654f*2;
														 
		RP_EulerAngle_2_Quaternion(&motorQ.q0,&motorQ.q1,&motorQ.q2,&motorQ.q3,angle_x,angle_y,angle_z);
		
//		RP_QuaternionMartix_RightMult(motorQ.q0,
//																  motorQ.q1,
//																  motorQ.q2,
//																  motorQ.q3,
//																 
//																 rollOut,pitchOut,yawOut,
//																 &rollOut,&pitchOut,&yawOut);

															 
															 
															 
		RP_QuaternionMartix_RightMult(q0,
																  q1,
																  q2,
																  q3,

																 rollOut,pitchOut,yawOut,
																 &rollOut,&pitchOut,&yawOut);

------------------------------------------------------------------------------

------------------------------------------------------------------------------

	  RP_QuaternionMartix_RightMult(q0,q1,q2,q3,&rollOut,&pitchOut,&yawOut);


	  RP_RotationTransform_IMU2Pose(imu.data.q.q0,
																	imu.data.q.q1,
																	imu.data.q.q2,
																	imu.data.q.q3,
																	&rollOut,&pitchOut,&yawOut);
		
		angle_y = (float)(motor[GIMB_P].rx_info.angle - 1440)/8192*360/180.f*3.141592654f/2;
		angle_z = (float)(motor[GIMB_Y].rx_info.angle - 1980)/8192*360/180.f*3.141592654f/2;
		
	  RP_RotationTransform_IMU2Pose(arm_cos_f32(gimb_angle),
																	arm_cos_f32(gimb_angle)*gimb_x,
																	arm_cos_f32(gimb_angle)*gimb_y,
																	arm_cos_f32(gimb_angle)*gimb_z,
																	&rollOut,&pitchOut,&yawOut);
											
//	  RP_Quaternion_2_EulerAngle(imu.data.qInit.q0,
//														  	imu.data.qInit.q1,
//																	imu.data.qInit.q2,
//																	imu.data.qInit.q3,
//																	&chas_r,&chas_p,&chas_y);
		
	  RP_Quaternion_2_EulerAngle(0,
														  	0.707,
																	0.707,
																	0,
																	&chas_r,&chas_p,&chas_y);


-------------------------------------------------------------------------------*/



#if 0	
		
		/*关于三轴自吻云台的二轴测试程序*/
		
		if(rc.info.state == RC_ONLINE){
			
			yaw_tar -= ((float)rc.data.ch0)/660;
			pit_tar += ((float)rc.data.ch1)/660;
		}

		//pid计算
		outPut.x = 0;
		outPut.y = 0;
		outPut.z = 0;
		
															 
		outPut.y = 
		motor[GIMB_P].c_pid2(&motor[GIMB_P].pid.angle,
												 &motor[GIMB_P].pid.angle_in,
		                     imu.data.rpy.pitch,
		                     imu.data.worldGyr.y,
												 pit_tar,1);		
		
		outPut.z = 
		motor[GIMB_Y].c_pid2(&motor[GIMB_Y].pid.angle,
												 &motor[GIMB_Y].pid.angle_in,
		                     imu.data.rpy.yaw,
		                     imu.data.worldGyr.z,
												 yaw_tar,1);
												 
		//获取底盘角度
		RP_Quaternion_2_EulerAngle((float)master[M1].data.imu_q.q0/30000,
			                         (float)master[M1].data.imu_q.q1/30000,
				                       (float)master[M1].data.imu_q.q2/30000,
					                     (float)master[M1].data.imu_q.q3/30000, 
			                          &chasRPY.roll,&chasRPY.pitch,&chasRPY.yaw);
							 									
															 
    //获取电机角度、四元数								 
    gimb.x = 0;
		gimb.y = (float)motor_half_cycle(motor[GIMB_P].rx_info.angle - 1440,8192)/8192*3.141592654f*2;
    gimb.z = (float)motor_half_cycle(motor[GIMB_Y].rx_info.angle - 1980,8192)/8192*3.141592654f*2;
														 
		RP_EulerAngle_2_Quaternion(&motorQ.q0,&motorQ.q1,&motorQ.q2,&motorQ.q3,gimb.x,gimb.y,gimb.z);
									 	
															 
    //转换相对云台的底盘姿态  //剔除z轴影响	 
	  chasRPY.yaw = 0;													 
		RP_QuaternionMartix_RightMult(motorQ.q0,
																  motorQ.q1,
																  motorQ.q2,
																  motorQ.q3,	
															 
																 chasRPY.roll,chasRPY.pitch,chasRPY.yaw,
																 &c2gRPY.roll,&c2gRPY.pitch,&c2gRPY.yaw); 
		//获取底盘相对四元数			  														 
		RP_EulerAngle_2_Quaternion(&c2gQ.q0,
															 &c2gQ.q1,
															 &c2gQ.q2,
															 &c2gQ.q3,
															 
															 c2gRPY.roll,c2gRPY.pitch,c2gRPY.yaw);															
													
    //解算实际输出															 
		RP_QuaternionMartix_RightMult(c2gQ.q0,
																  c2gQ.q1,
																  c2gQ.q2,
																  c2gQ.q3,

																 outPut.x,outPut.y,outPut.z,
																 &outPut.x,&outPut.y,&outPut.z);



    SendBuff[motor[GIMB_P].id.buff_p] = outPut.y;
		SendBuff[motor[GIMB_Y].id.buff_p] = outPut.z;
				
		
		
    //控制数据的发送
		if(rc.info.state == RC_ONLINE && HAL_GetTick()>1000){
		
			motor[GIMB_Y].tx(&motor[GIMB_Y],SendBuff);
		}

		else{
			
		  outPut.x = 0;
			outPut.y = 0;
			outPut.z = 0;
			
			yaw_tar = imu.data.rpy.yaw;
		  pit_tar = 0;
		}
		
#endif		




