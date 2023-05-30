#ifndef __AUTO_H
#define __AUTO_H

#include "type.h"

typedef enum { 
	
	AUTO_NO,
	AUTO_ING,
	AUTO_OK,
	
} auto_State;

typedef enum {

	AUTO_NULL,
	
	AUTO_AIM,
	AUTO_BIG_BUFF,
	AUTO_MIN_BUFF,

}auto_VisionType;

typedef enum { 
	
	AUTO_ONLINE,
	AUTO_ERR,
	
} auto_UpperState;

typedef struct auto_time_struct{

	uint32_t auto_Time;
	

}auto_time;

typedef struct auto_info_struct{

  auto_VisionType type;
	auto_UpperState UpperState;
  
	auto_State Auxiliary;//辅助中
  auto_State BadTiming;//时机不对

}auto_info;


typedef struct auto_data_struct{

	
	float ii;



}auto_data;


typedef struct auto_struct{

	auto_info info;
	auto_data data;
	auto_time time;



}auto_t;






#endif





