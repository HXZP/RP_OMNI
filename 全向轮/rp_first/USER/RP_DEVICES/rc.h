#ifndef __RC_H
#define __RC_H

#include "type.h"
#include "stdbool.h"

#define r_abs(x) 					((x)>0? (x):(-(x)))

#define    RC_CH_VALUE_MIN        ((uint16_t)364 )
#define    RC_CH_VALUE_OFFSET     ((uint16_t)1024)
#define	   RC_CH_VALUE_MAX        ((uint16_t)1684)

#define	   RC_CH_VALUE_SIDE_WIDTH	((RC_CH_VALUE_MAX-RC_CH_VALUE_MIN)/2)

#define    CH_MIN       ((int16_t)-660)
#define    CH_OFFSET    ((int16_t)   0)
#define	   CH_MAX       ((int16_t) 660)

/* ----------------------- RC Switch Definition-------------------------------*/

#define    SW_UP              ((uint16_t)1)
#define    SW_MID             ((uint16_t)3)
#define    SW_DOWN            ((uint16_t)2)

/* ----------------------- Function Definition-------------------------------- */
/* 遥控摇杆通道偏移值 */
#define			SW1						(rc.data.s1)
#define			SW2						(rc.data.s2)
#define			CH2_VALUE			(rc.data.ch2)
#define			CH3_VALUE			(rc.data.ch3)
#define			CH0_VALUE			(rc.data.ch0)
#define			CH1_VALUE			(rc.data.ch1)
#define			TW_VALUE			(rc.data.thumbwheel)

/*键盘值对应通道*/
#define			CH2_VALUE_K		(rc.data->ch[2])
#define			CH3_VALUE_K		(rc.data->ch[3])
#define			CH0_VALUE_K		(rc.data->ch[0])
#define			CH1_VALUE_K		(rc.data->ch[1])

/* 检测遥控器开关状态 */
#define     SW1_UP      (rc.data.s1 == SW_UP)
#define     SW1_MID     (rc.data.s1 == SW_MID)
#define     SW1_DOWN    (rc.data.s1 == SW_DOWN)
#define     SW2_UP      (rc.data.s2 == SW_UP)
#define     SW2_MID     (rc.data.s2 == SW_MID)
#define     SW2_DOWN    (rc.data.s2 == SW_DOWN)

/* 键盘 */

#define KEY_W                rc.data.kb.bit.W		
#define KEY_S                rc.data.kb.bit.S		
#define KEY_A                rc.data.kb.bit.A		
#define KEY_D                rc.data.kb.bit.D	
#define KEY_SHIFT            rc.data.kb.bit.SHIFT	
#define KEY_CTRL             rc.data.kb.bit.CTRL		
#define KEY_Q                rc.data.kb.bit.Q		
#define KEY_E                rc.data.kb.bit.E		
#define KEY_R                rc.data.kb.bit.R		
#define KEY_F                rc.data.kb.bit.F		
#define KEY_G                rc.data.kb.bit.G		
#define KEY_Z                rc.data.kb.bit.Z		
#define KEY_X                rc.data.kb.bit.X		
#define KEY_C                rc.data.kb.bit.C		
#define KEY_V                rc.data.kb.bit.V		
#define KEY_B                rc.data.kb.bit.B		

#define KEY_ALL_CODE         rc.data.kb.key_code

/*鼠标三轴的移动速度*/
#define    MOUSE_X_MOVE_SPEED    (rc.data.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (rc.data.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (rc.data.mouse.z)

/* 检测鼠标按键状态 */
#define    MOUSE_LEFT    (rc.data.mouse.press_l )
#define    MOUSE_RIGH    (rc.data.mouse.press_r )
/*按键标志位*/

#define W_FLAG       rc.data->Key_Flg[W]
#define S_FLAG       rc.data->Key_Flg[S]
#define A_FLAG       rc.data->Key_Flg[A]
#define D_FLAG       rc.data->Key_Flg[D]
#define SHIFT_FLAG   rc.data->Key_Flg[SHIFT]
#define CTRL_FLAG    rc.data->Key_Flg[CTRL]
#define Q_FLAG       rc.data->Key_Flg[Q]
#define E_FLAG       rc.data->Key_Flg[E]
#define R_FLAG       rc.data->Key_Flg[R]
#define F_FLAG       rc.data->Key_Flg[F]
#define G_FLAG       rc.data->Key_Flg[G]
#define Z_FLAG       rc.data->Key_Flg[Z]
#define X_FLAG       rc.data->Key_Flg[X]
#define C_FLAG       rc.data->Key_Flg[C]
#define V_FLAG       rc.data->Key_Flg[V]
#define B_FLAG       rc.data->Key_Flg[B]
/*按键时间 长短按的判断*/
#define CHANGE_TIM                      5    //ms
#define LONG_CHANGE_TIM_W               1000   //ms
#define LONG_CHANGE_TIM_S               1000   //ms
#define LONG_CHANGE_TIM_A               1000   //ms
#define LONG_CHANGE_TIM_D               1000   //ms
#define LONG_CHANGE_TIM_Q               500    //ms
#define LONG_CHANGE_TIM_E               500    //ms
#define LONG_CHANGE_TIM_R               500    //ms
#define LONG_CHANGE_TIM_F               500    //ms
#define LONG_CHANGE_TIM_G               500    //ms
#define LONG_CHANGE_TIM_Z               500    //ms
#define LONG_CHANGE_TIM_X               500    //ms
#define LONG_CHANGE_TIM_C               500    //ms
#define LONG_CHANGE_TIM_V               500    //ms
#define LONG_CHANGE_TIM_B               500    //ms
#define LONG_CHANGE_TIM_CTRL            500   //ms
#define LONG_CHANGE_TIM_SHIFT           500    //ms
#define LONG_CHANGE_TIM_MOUSE_L         230    //ms
#define LONG_CHANGE_TIM_MOUSE_R         500    //ms

/*鼠标速度最大值限制*/
#define Xmax    300
#define Ymax    300
/*鼠标滑动滤波长度*/
#define SF_LENGTH 15

/* 检测键盘按键状态 */
#define KEY_UP                    0x00
#define KEY_DOWN                  0x01

/* 检测按键或鼠标是否有按下 */
#define KEY_CTRL_STATE (KEY_ALL_CODE || MOUSE_LEFT || MOUSE_RIGH)


/* 键盘标志位枚举 */
typedef enum{
	W,
	S,
	A,
	D,
	Q,      
	E,      
	R,      
	F,      
	G,      
	Z,
	X,
	C,      
	V,      
	B,       
	CTRL,    
	SHIFT,   
	Mouse_L,
	Mouse_R,  	
	KEY_CNT,
}key_e;

typedef enum{

	RC_OFFLINE,
	RC_ONLINE,
	
}rc_state_e;

/*      NDJ6遥控数据结构       */

typedef struct rc_data_struct {
	int16_t 	ch0;
	int16_t 	ch1;
	int16_t 	ch2;
	int16_t 	ch3;
	uint8_t  	s1;
	uint8_t  	s2;
	int16_t 	thumbwheel;
	int16_t   tw_step[4];
	
	struct
  {
      int16_t x;
      int16_t y;
      int16_t z;
      uint8_t press_l;
      uint8_t press_r;
  }mouse;	
	union 
	{
		uint16_t key_code;
		struct
		{
			uint16_t W : 1;//冒号表示位域，
			uint16_t S : 1;
			uint16_t A : 1;
			uint16_t D : 1;
			uint16_t SHIFT : 1;
			uint16_t CTRL : 1;
			uint16_t Q : 1;
			uint16_t E : 1;
			uint16_t R : 1;
			uint16_t F : 1;
			uint16_t G : 1;
			uint16_t Z : 1;
			uint16_t X : 1;
			uint16_t C : 1;
			uint16_t V : 1;
			uint16_t B : 1;
		} bit;
	} kb;	
	float ch[4];	           /*键盘模拟通道*/
	
	char  key_flg[KEY_CNT];  /*键盘标志位*/
	
} rc_data_t;


typedef struct rc_info_struct{

	rc_state_e state;
	char       rc_wake;	
	
	int16_t    tw_step_value[4];

	
	int16_t		 offline_cnt;
	int16_t		 offline_max_cnt;
	
}rc_info_t;


/*
*  键盘信息结构体
*/
typedef enum 
{
  UP = 0,   //0
  PRESS,    //0->1
  SHORT_DOWN,//1短按
  DOWN,      //1长按
  RELAX,    //1->0
}KEY_State_t;

typedef struct 
{
  int         state_cnt ;
  KEY_State_t State;
  KEY_State_t prev_State;
  uint8_t     prev_KEY_PRESS;
  uint8_t     KEY_PRESS;
}KEY_Info_t;

typedef struct
{
  float Slope;
  float MoveData;
  float K;
}KEY_DirInfo_t;

typedef struct
{
  KEY_DirInfo_t FB_dir;
  KEY_DirInfo_t LR_dir;
}KEY_MoveInfo_t;

typedef struct
{
  float SFX[SF_LENGTH]; 
  float SFY[SF_LENGTH]; 
  float SFZ[SF_LENGTH];	
}Mouse_SF_t;

typedef struct 
{
  KEY_Info_t     W;
  KEY_Info_t     S;
  KEY_Info_t     A;
  KEY_Info_t     D;
  KEY_Info_t     Q;       //左转90度
  KEY_Info_t     E;       //右转90度
  KEY_Info_t     R;       //打开弹舱
  KEY_Info_t     F;       //小陀螺
  KEY_Info_t     G;       //快速抬头
  KEY_Info_t     Z;
  KEY_Info_t     X;
  KEY_Info_t     C;       //视觉模式切换
  KEY_Info_t     V;       //从右后转180度
  KEY_Info_t     B;       //对位模式？
  KEY_Info_t     CTRL;    //机械模式切换
  KEY_Info_t     SHIFT;   //极速模式切换
  KEY_Info_t     Mouse_L;
  KEY_Info_t     Mouse_R;  
  KEY_MoveInfo_t MoveInfo;
  Mouse_SF_t     MouseSF; 
}Keyboard_Info_t;

typedef struct rc_struct {
	
	
	rc_info_t       info;
	rc_data_t	      data;
  Keyboard_Info_t keyMouse;
	
	void				     (*init)(struct rc_struct *self);
	void				     (*heart_beat)(struct rc_struct *self);	
	void				     (*updata)(struct rc_struct *self, uint8_t *rxBuf);
	void				     (*key)(struct rc_struct *self);

} rc_t;



extern rc_t      rc;


#endif
