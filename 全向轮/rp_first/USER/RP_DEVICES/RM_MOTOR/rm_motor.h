#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H

#include "rm_motor_def.h"


#define m_abs(x) 					((x)>0? (x):(-(x)))
#define m_constrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define m_anti_constrain(x, min, max)	(((x<max) && (x>min))?(0):x)


typedef struct motor_class_t
{
	motor_rx_info_t        rx_info;
	motor_state_info_t     state;
	motor_id_info_t        id;
  motor_mec_info_t       mec_info;
	motor_pid_all_t        pid;
	
	void (*init)(struct motor_class_t *motor);
	void (*heartbeat)(struct motor_class_t *motor);
  void (*pid_init)(motor_pid_t *pid, float *buff);	
	
	uint8_t (*tx)(struct motor_class_t *motor, int16_t *buff);
	uint8_t (*rx)(struct motor_class_t *motor, uint8_t *buff,uint32_t id,motor_drive_e driver);
	
	/*以下为控制函数*/
	/*
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
	*/
	float (*c_posit)(struct motor_class_t *motor,float target);
	float (*c_angle)(struct motor_class_t *motor,float target);	
	float (*c_speed)(struct motor_class_t *motor,float target);	
	
	/*
		用户自定义pid
		err_cal_mode为err处理方式， 0，1，2：一圈、半圈、四分之一圈
		meas 使用360度制 电机角度8192这里不会自动转360
	*/
	float (*c_pid2)(motor_pid_t *out, motor_pid_t *inn, float meas1, float meas2, float tar, char err_cal_mode);	
	float (*c_pid1)(motor_pid_t *out, float meas1, float tar);	
	
	void  (*c_judge_dir)(struct motor_class_t *motor,uint16_t range);	
  void  (*c_offset)(struct motor_class_t *motor, uint16_t range);	
	
  uint8_t (*c_stuck_flag)(struct motor_class_t *motor,uint16_t torque_limit);	
	
}motor_t;




void motor_class_init(struct motor_class_t *motor);
void motor_class_heartbeat(struct motor_class_t *motor);
void motor_class_pid_init(motor_pid_t *pid, float *buff);
uint8_t motor_class_stucking_flag(struct motor_class_t *motor,uint16_t torque_limit);

void get_rm_can_drvie(struct motor_class_t *motor);
void get_rm_info(struct motor_class_t *motor, uint8_t *rxBuf);

float motor_cycle(float tar,float cycle);
float motor_half_cycle(float angle,float max);
void  motor_judge_dir(struct motor_class_t *motor,uint16_t range);
void  motor_offset(struct motor_class_t *motor, uint16_t range);

float motor_pid_err(motor_pid_t *pid,float measure);
float motor_pid_cal(motor_pid_t *pid);
float motor_pid_ctrl(motor_pid_t *out, motor_pid_t *inn, float meas1, float meas2, char err_cal_mode);

float motor_pid_position(struct motor_class_t *motor,float target);
float motor_pid_angle(struct motor_class_t *motor,float target);
float motor_pid_speed(struct motor_class_t *motor,float target);

float motor_pid_double(motor_pid_t *out, motor_pid_t *inn, float meas1, float meas2, float tar, char err_cal_mode);
float motor_pid_single(motor_pid_t *out, float meas1, float tar);



#endif

